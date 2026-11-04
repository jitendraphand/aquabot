"""Simulation scenarios. Each returns a metrics dict; with video=True it also renders an MP4."""
from __future__ import annotations

import os
os.environ.setdefault("MUJOCO_GL", "egl")

from pathlib import Path

import mujoco
import numpy as np

from .. import params as P
from ..kinematics import arm as AK, legs as LK
from .runtime import AquaSim, STOW

ROOT = Path(__file__).resolve().parents[2]
VID = ROOT / "sim" / "videos"

POT_R, POT_H = 0.055, 0.080


def plant_xml(i, x, y):
    """Terracotta pot with soil and a seedling (static)."""
    zs = POT_H
    leaves = []
    rng = np.random.default_rng(i + 3)
    for k in range(5):
        a = k * 2 * np.pi / 5 + rng.uniform(-0.3, 0.3)
        r = 0.022 + rng.uniform(0, 0.008)
        h = zs + 0.05 + rng.uniform(0, 0.03)
        leaves.append(f'<geom type="ellipsoid" size="0.020 0.009 0.003" pos="{x + r*np.cos(a):.4f} {y + r*np.sin(a):.4f} {h:.4f}" '
                      f'euler="0.3 {0.4*np.cos(a):.3f} {a:.3f}" material="leaf" contype="0" conaffinity="0"/>')
    return f"""
    <geom name="pot{i}" type="cylinder" size="{POT_R:.3f} {POT_H/2:.3f}" pos="{x:.4f} {y:.4f} {POT_H/2:.4f}" material="terracotta" contype="1" conaffinity="2"/>
    <geom name="soil{i}" type="cylinder" size="{POT_R-0.006:.3f} 0.002" pos="{x:.4f} {y:.4f} {POT_H+0.001:.4f}" material="soil" contype="0" conaffinity="0"/>
    <geom type="capsule" size="0.003" fromto="{x:.4f} {y:.4f} {POT_H:.4f} {x:.4f} {y:.4f} {POT_H+0.07:.4f}" material="leaf" contype="0" conaffinity="0"/>
    {''.join(leaves)}"""


def row_xml(n=6, x0=-0.4, y0=-0.5, dx=0.5, dy=0.0):
    """Background crop rows for context."""
    out = []
    for k in range(n):
        out.append(f'<geom type="box" size="0.02 1.8 0.004" pos="{x0 + k*dx:.3f} {y0:.3f} 0.004" rgba="0.36 0.27 0.18 1" contype="0" conaffinity="0"/>')
    return "\n".join(out)


def _hud_common(sim: AquaSim, title: str, mode: str):
    v = 0.0
    n = int(P.CRAWL["period"] / sim.dt)              # average over one full gait cycle
    if len(sim.log["base"]) > n:
        v = (sim.log["base"][-1][0] - sim.log["base"][-n][0]) / (n * sim.dt)
    lines = [title, f"t = {sim.d.time:5.1f} s   mode: {mode}",
             f"tank: {sim.water_ml:5.1f} ml   pump: {'ON ' if sim.pump_on else 'off'}   v = {v*100:4.1f} cm/s"]
    return lines


# ---------------------------------------------------------------- walk
def walk(video=False, seconds=None):
    sim = AquaSim(env_xml=row_xml(), drops=False)
    mode = {"m": "stand"}
    sim.hud = lambda s: _hud_common(s, "AquaBot — crawl gait (3 feet always down)", mode["m"])
    sim.record = video
    sim.cam.distance, sim.cam.azimuth, sim.cam.elevation = 1.25, 135, -16
    sim.init_pose()
    sim.stand(1.0)
    x0 = sim.d.qpos[:2].copy(); yaw0 = sim.rpy()[2]; t0 = sim.d.time
    mode["m"] = f"crawl forward (cmd {P.CRAWL['v_cmd']*100:.1f} cm/s)"
    sim.crawl_walk(12.0, v=(P.CRAWL["v_cmd"], 0.0))
    x1 = sim.d.qpos[:2].copy(); t1 = sim.d.time
    mode["m"] = "turn in place 0.12 rad/s"
    sim.cam.azimuth = 120
    sim.crawl_walk(7.2, v=(0.0, 0.0), wz=0.12)
    yaw1 = sim.rpy()[2]
    mode["m"] = "stand"
    sim.stand(1.0)
    m = sim.metrics(t0)
    m.update(speed_m_s=float(np.linalg.norm(x1 - x0) / (t1 - t0)),
             heading_drift_deg=float(np.degrees(abs(np.arctan2(x1[1] - x0[1], x1[0] - x0[0])))),
             turn_deg=float(np.degrees((yaw1 - yaw0 + np.pi) % (2 * np.pi) - np.pi)),
             ik_clamps=int(getattr(sim, "ik_clamps", 0)))
    if video:
        sim.save_video(VID / "01_crawl_walk_turn.mp4")
    sim.close()
    return m


# -------------------------------------------------------------- watering
def watering(video=False, n_plants=3, dose_ml=10.0):
    spacing = 0.30
    py = 0.235
    xs = [0.06 + k * spacing for k in range(n_plants)]
    env = row_xml() + "".join(plant_xml(i, x, py) for i, x in enumerate(xs))
    pots = [(x, py, POT_H + 0.002, POT_R - 0.006, f"soil{i}") for i, x in enumerate(xs)]
    sim = AquaSim(env_xml=env, pots=pots, drops=True)
    mode = {"m": "stand"}

    def hud(s):
        lines = _hud_common(s, "AquaBot — autonomous watering mission", mode["m"])
        dl = "   ".join(f"plant {i+1}: {s.delivered.get(i, 0.0):4.1f} ml" for i in range(n_plants))
        lines.append(dl)
        return lines
    sim.hud = hud
    sim.record = video
    sim.cam.distance, sim.cam.azimuth, sim.cam.elevation = 1.15, 205, -22
    sim.cam_offset = np.array([0.06, 0.08, -0.08])
    sim.init_pose()
    sim.stand(1.0)
    t0 = sim.d.time
    misses = []
    for i, x in enumerate(xs):
        # walk until the plant is level with the arm's yaw axis
        mode["m"] = f"crawl to plant {i+1}"
        dx = x - (sim.d.qpos[0] + P.ARM_YAW_XY[0] / 1000.0) - 0.0
        if dx > 0.01:
            T = max(P.CRAWL["period"], dx / 0.025)
            sim.crawl_walk(T, v=(min(P.CRAWL["v_cmd"], dx / T / 0.72), 0.0))
        # crouch & lean towards the plant to bring the soil within reach
        mode["m"] = f"aim at plant {i+1} (ballistic IK)"
        WS = P.WATER_STANCE
        sim.stand(1.2, height=WS["height"], rpy=(np.radians(WS["roll_deg"]), 0.0, 0.0))
        target_w = np.array([x, py, POT_H + 0.002]) * 1000.0
        tb = sim.world_to_body(target_w)
        robot_xy_b = np.array([P.ARM_YAW_XY[0], 0.0, 0.0])
        approach = robot_xy_b - tb
        tip, pitch = AK.aim_nozzle(tb, approach, pitch=np.radians(WS["pitch_deg"]), height=WS["drop"])
        try:
            q_aim = AK.ik(tip, pitch, tool=AK.NOZZLE, q_ref=np.radians([90, 40, -100, -20]))
        except AK.Unreachable as ex:
            misses.append(str(ex)); q_aim = None
        if q_aim is not None:
            sim.move_arm(np.radians([90.0, 60.0, -110.0, -40.0]), 1.4)
            sim.move_arm(q_aim, 1.4)
            sim.stand(0.4)
            mode["m"] = f"watering plant {i+1}: {dose_ml:.0f} ml"
            sim.water(dose_ml)
            mode["m"] = "stow arm"
            sim.move_arm(np.radians([90.0, 60.0, -110.0, -40.0]), 1.2)
            sim.move_arm(STOW, 1.4)
        sim.stand(1.0, height=P.STAND_HEIGHT, rpy=(0, 0, 0))
    mode["m"] = "mission complete"
    sim.stand(1.5)
    delivered = sum(sim.delivered.values())
    m = sim.metrics(t0)
    m.update(emitted_ml=float(sim.emitted_ml), delivered_ml=float(delivered),
             delivery_fraction=float(delivered / max(sim.emitted_ml, 1e-9)),
             per_plant_ml=[float(sim.delivered.get(i, 0.0)) for i in range(n_plants)],
             tank_left_ml=float(sim.water_ml), water_balance_err_ml=float(
                 P.TANK_TARGET_ML - sim.water_ml - sim.emitted_ml), aim_failures=misses,
             ik_clamps=int(getattr(sim, "ik_clamps", 0)))
    if video:
        sim.save_video(VID / "02_watering_mission.mp4")
    sim.close()
    return m


# -------------------------------------------------------------- pick & place
def pick_place(video=False):
    cube = 0.025
    obj = np.array([0.060, 0.150, cube / 2])
    tray = np.array([-0.012, 0.172])
    env = row_xml() + f"""
    <body name="stone" pos="{obj[0]:.4f} {obj[1]:.4f} {obj[2]+0.0005:.4f}">
      <freejoint/>
      <inertial pos="0 0 0" mass="0.040" diaginertia="4.2e-6 4.2e-6 4.2e-6"/>
      <geom name="stone" type="box" size="{cube/2:.4f} {cube/2:.4f} {cube/2:.4f}" mass="0.040" rgba="0.55 0.55 0.58 1"
            friction="1.2 0.02 0.002" condim="4" contype="1" conaffinity="3"/>
    </body>
    <geom name="tray_floor" type="box" size="0.045 0.045 0.003" pos="{tray[0]:.4f} {tray[1]:.4f} 0.003" rgba="0.15 0.35 0.65 1" contype="1" conaffinity="3"/>
    <geom type="box" size="0.045 0.003 0.012" pos="{tray[0]:.4f} {tray[1]+0.042:.4f} 0.012" rgba="0.15 0.35 0.65 1" contype="1" conaffinity="3"/>
    <geom type="box" size="0.045 0.003 0.012" pos="{tray[0]:.4f} {tray[1]-0.042:.4f} 0.012" rgba="0.15 0.35 0.65 1" contype="1" conaffinity="3"/>
    <geom type="box" size="0.003 0.045 0.012" pos="{tray[0]+0.042:.4f} {tray[1]:.4f} 0.012" rgba="0.15 0.35 0.65 1" contype="1" conaffinity="3"/>
    <geom type="box" size="0.003 0.045 0.012" pos="{tray[0]-0.042:.4f} {tray[1]:.4f} 0.012" rgba="0.15 0.35 0.65 1" contype="1" conaffinity="3"/>
    """
    sim = AquaSim(env_xml=env, drops=False)
    mode = {"m": "stand"}
    sim.hud = lambda s: _hud_common(s, "AquaBot — pick & place with the 4-DOF arm + gripper", mode["m"]) + [
        f"gripper opening: {2*s.grip + 1:4.1f} mm   stone z: {s.d.body('stone').xpos[2]*1000:5.1f} mm"]
    sim.record = video
    sim.cam.distance, sim.cam.azimuth, sim.cam.elevation = 0.95, 200, -28
    sim.cam_offset = np.array([0.0, 0.12, -0.10])
    sim.init_pose()
    sim.stand(1.0)
    PS = P.PICK_STANCE
    mode["m"] = "re-step feet (wider base for the crouch)"
    wide = {l: np.array([PS["foot_dx_front"] if l[0] == "F" else PS["foot_dx_rear"], 0.0, 0.0]) for l in P.LEGS}
    sim.step_feet(wide)
    mode["m"] = "crouch + lean left"
    sim.stand(1.6, height=PS["height"], rpy=(np.radians(PS["roll_deg"]), 0.0, 0.0), shift=(0.0, PS["shift_y"], 0.0))
    pitch = np.radians(PS["pitch_deg"])
    above = obj * 1000 + np.array([0, 0, 55.0])
    grasp = obj * 1000 + np.array([0, 0, 5.0])
    mode["m"] = "reach above stone"
    sim.move_arm(np.radians([90.0, 55.0, -100.0, -40.0]), 1.5, grip=17.0)
    qa = AK.ik(sim.world_to_body(above), pitch, q_ref=np.radians([90, 30, -90, -30]))
    sim.move_arm(qa, 1.5, grip=17.0)
    mode["m"] = "descend (Cartesian line)"
    sim.arm_line(grasp, pitch, 1.5)
    mode["m"] = "close gripper"
    sim.move_arm(sim.arm_q, 0.9, grip=0.0)
    sim.stand(0.4)
    z_before = sim.d.body("stone").xpos[2]
    mode["m"] = "lift"
    sim.arm_line(grasp + np.array([0, 0, 72.0]), pitch, 1.4)
    lift = (sim.d.body("stone").xpos[2] - z_before) * 1000.0
    mode["m"] = "carry to tray"
    place_above = np.array([tray[0] * 1000, tray[1] * 1000, 70.0 + 22.0])
    sim.arm_line(place_above, pitch, 2.2)
    sim.arm_line(place_above - np.array([0, 0, 44.0]), pitch, 1.2)
    mode["m"] = "release"
    sim.move_arm(sim.arm_q, 0.8, grip=15.0)
    sim.stand(0.6)
    sim.arm_line(place_above + np.array([0, 0, 20.0]), pitch, 1.0)
    mode["m"] = "stow + stand up"
    sim.move_arm(np.radians([90.0, 55.0, -100.0, -40.0]), 1.2)
    sim.move_arm(STOW, 1.4)
    sim.stand(1.6, height=P.STAND_HEIGHT, rpy=(0, 0, 0), shift=(0, 0, 0))
    mode["m"] = "re-step feet to walking stance"
    sim.step_feet({l: np.zeros(3) for l in P.LEGS})
    sim.stand(1.0)
    stone = sim.d.body("stone").xpos.copy()
    err = float(np.linalg.norm(stone[:2] - tray) * 1000.0)
    m = sim.metrics(1.0)
    m.update(lift_mm=float(lift), place_err_mm=err, stone_final_mm=(stone * 1000).round(1).tolist(),
             in_tray=bool(abs(stone[0] - tray[0]) < 0.04 and abs(stone[1] - tray[1]) < 0.04),
             unreachable_waypoints=int(getattr(sim, "arm_unreachable", 0)),
             ik_clamps=int(getattr(sim, "ik_clamps", 0)))
    if video:
        sim.save_video(VID / "03_pick_and_place.mp4")
    sim.close()
    return m


# ------------------------------------------------------------- IK showcase
def ik_showcase(video=False):
    trail = "\n".join(f'<body name="mk{i}" mocap="true" pos="0 0 -5"><geom type="sphere" size="0.004" rgba="1 0.35 0.1 0.9" contype="0" conaffinity="0"/></body>' for i in range(60))
    sim = AquaSim(env_xml=row_xml() + trail, drops=False)
    info = {"m": "body-pose IK (feet fixed)", "err": 0.0}

    def hud(s):
        r = np.degrees(s.body_rpy_cmd)
        return ["AquaBot — whole-body & arm inverse kinematics",
                f"t = {s.d.time:5.1f} s   {info['m']}",
                f"body cmd: roll {r[0]:+5.1f}°  pitch {r[1]:+5.1f}°  h {s.height:5.1f} mm",
                f"arm IK |FK(IK(p)) - p| = {info['err']:.1e} mm"]
    sim.hud = hud
    sim.record = video
    sim.cam.distance, sim.cam.azimuth, sim.cam.elevation = 1.2, 150, -14
    sim.init_pose()
    sim.stand(1.0)
    T = 9.0
    n = int(T / sim.dt)
    for i in range(n):
        t = i * sim.dt
        a = min(1.0, t / 1.0, (T - t) / 1.0)
        sim.body_rpy_cmd = np.array([np.radians(10) * np.sin(2 * np.pi * t / 3.0),
                                     np.radians(8) * np.sin(2 * np.pi * t / 4.5 + 1.0), 0.0]) * a
        sim.height = P.STAND_HEIGHT - 25.0 * a * (0.5 - 0.5 * np.cos(2 * np.pi * t / 4.5))
        sim.body_shift = np.array([18 * np.sin(2 * np.pi * t / 6.0), 0.0, 0.0]) * a
        feet = {leg: LK.neutral_foot(leg, sim.height) for leg in P.LEGS}
        # body-frame feet for a commanded pose (no attitude feedback here: open-loop body IK)
        from ..kinematics.body import rpy_matrix
        R = rpy_matrix(*sim.body_rpy_cmd)
        fb = {leg: R.T @ (p - sim.body_shift) for leg, p in feet.items()}
        sim.leg_targets_from_feet(fb)
        sim.step()
    info["m"] = "arm IK: TCP traces a circle"
    sim.body_rpy_cmd[:] = 0; sim.body_shift[:] = 0
    sim.stand(0.8, height=P.STAND_HEIGHT)
    c = np.array([230.0, 0.0, 170.0])      # QA iter 9: re-centred after the shoulder moved up 8 mm
    R0 = 45.0
    tool_pitch = np.radians(-35.0)
    pts = []
    sim.move_arm(AK.ik(c + np.array([0, R0, 0]), tool_pitch), 1.2, grip=6.0)
    T2 = 6.0
    n = int(T2 / sim.dt)
    errs = []
    for i in range(n):
        th = 2 * np.pi * i / n
        p = c + np.array([0.0, R0 * np.cos(th), R0 * np.sin(th)])
        q = AK.ik(p, tool_pitch, q_ref=sim.arm_q)
        errs.append(np.linalg.norm(AK.fk(q)[0] - p))
        info["err"] = errs[-1]
        sim.set_arm(q)
        feet = {leg: LK.neutral_foot(leg, sim.height) for leg in P.LEGS}
        sim.leg_targets_from_feet(sim._balanced_feet(feet))
        sim.step()
        if i % int(n / 60) == 0 and len(pts) < 60:
            pts.append(sim.body_to_world(p) / 1000.0)
            for k, pp in enumerate(pts):
                sim.d.mocap_pos[k] = pp
    tcp_w = sim.site("tcp")[0] * 1000
    sim.move_arm(STOW, 1.4)
    sim.stand(0.8)
    m = sim.metrics(1.0)
    m.update(arm_ik_max_err_mm=float(max(errs)))
    if video:
        sim.save_video(VID / "04_ik_showcase.mp4")
    sim.close()
    return m


# -------------------------------------------------------- exploded assembly
def exploded_assembly(video=True, seconds=10.0):
    from .. import robot as R
    from ..cad.catalog import all_parts
    from ..kinematics import legs as LKm
    from . import mjcf
    from ..render import _mesh_assets
    import imageio.v2 as imageio
    q = {}
    for leg in P.LEGS:
        qq = LKm.ik(leg, LKm.neutral_foot(leg))
        q[f"{leg}_hip_abd"], q[f"{leg}_hip_pitch"], q[f"{leg}_knee"] = qq
    q.update(arm_yaw=STOW[0], arm_shoulder=STOW[1], arm_elbow=STOW[2], arm_wrist=STOW[3], grip_a=8.0)
    inst = R.world_instances(q)
    parts = all_parts()
    bodies, offs = [], []
    for k, (n, key, M, link) in enumerate(inst):
        c = M[:3, 3]
        if link == "base":
            z = 0.0
            if key in ("top_deck", "electronics_tray"): z = 120
            if key in ("electronics_cover",): z = 220
            if key in ("arm_pedestal", "arm_bearing_plate", "servo_arm_yaw", "bearing_yaw"): z = 170
            if key in ("tank_lid", "fill_cap", "battery_2s_2200", "pump_peristaltic", "ubec_6v_10a", "buck_5v", "imu_mpu6050"): z = 60 if key == "tank_lid" else 85
            if key == "tank_body": z = -10
            if key in ("pca9685", "esp32_devkitc", "mosfet_module"): z = 160
            o = np.array([np.sign(c[0]) * 70 if key.startswith(("bulkhead", "servo_", "tof", "rocker", "xt60")) else 0.0, 0, z])
        elif link.startswith(("F", "R")) and link[2] == "_":
            e, s = P.sign_e(link), P.sign_s(link)
            depth = {"hip": 1, "femur": 2, "tibia": 3}[link.split("_")[1]]
            o = np.array([e * 50 * depth, s * 65 * depth, -20 * depth])
        else:
            depth = {"arm_turntable": 1, "arm_upper": 2, "arm_fore": 3, "arm_wrist": 4, "jaw_a": 4.6, "jaw_b": 4.6, "pinion": 4.3}[link]
            o = np.array([25 * depth, 0, 60 + 38 * depth])
        offs.append(o)
        rgba = parts[key].rgba
        qq = R.rot_to_quat(M[:3, :3])
        bodies.append(f'<body name="p{k}" mocap="true" pos="{f3(M[:3,3]/1000)}" quat="{f3(qq)}"><geom type="mesh" mesh="m_{key}" '
                      f'rgba="{rgba[0]:.3f} {rgba[1]:.3f} {rgba[2]:.3f} 1" contype="0" conaffinity="0"/></body>')
    xml = f"""<mujoco><visual><global offwidth="1280" offheight="720" fovy="35"/><quality shadowsize="4096" offsamples="8"/>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.45 0.45 0.45" specular="0.05 0.05 0.05"/></visual>
    <asset><texture name="sky" type="skybox" builtin="gradient" rgb1="0.96 0.97 1.0" rgb2="0.78 0.84 0.92" width="64" height="64"/>
    <texture name="g" type="2d" builtin="checker" rgb1="0.92 0.93 0.94" rgb2="0.86 0.87 0.89" width="300" height="300"/>
    <material name="g" texture="g" texrepeat="10 10"/>{_mesh_assets([i[1] for i in inst])}</asset>
    <worldbody><light pos="0.8 -0.8 2" dir="-0.4 0.4 -1" castshadow="true" diffuse="0.6 0.6 0.6"/>
    <geom type="plane" size="3 3 .1" pos="0 0 {-P.STAND_HEIGHT/1000:.3f}" material="g"/>{''.join(bodies)}</worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, 720, 1280)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = (0.0, 0.0, 0.0); cam.distance = 1.65; cam.elevation = -16
    frames = []
    nf = int(seconds * 30)
    base = [i[2][:3, 3] / 1000.0 for i in inst]
    for fi in range(nf):
        t = fi / nf
        a = 1.0 - np.clip((t - 0.15) / 0.55, 0, 1)
        a = a * a * (3 - 2 * a)
        for k in range(len(inst)):
            d.mocap_pos[k] = base[k] + offs[k] / 1000.0 * a
        cam.azimuth = 120 + 80 * t
        cam.lookat[:] = (0.0, 0.0, -0.04 + 0.06 * a)
        mujoco.mj_forward(m, d)
        r.update_scene(d, cam)
        img = r.render().copy()
        from .runtime import draw_hud
        frames.append(draw_hud(img, ["AquaBot — assembly", f"{len(inst)} part instances · "
                                     f"{len(set(i[1] for i in inst))} unique parts"]))
    r.close()
    out = VID / "05_exploded_assembly.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    w = imageio.get_writer(str(out), fps=30, codec="libx264", quality=None,
                           ffmpeg_params=["-crf", "23", "-preset", "veryfast", "-pix_fmt", "yuv420p"])
    for fr in frames:
        w.append_data(fr)
    w.close()
    return {"frames": nf}


def f3(v):
    return " ".join(f"{x:.6f}" for x in np.ravel(v))

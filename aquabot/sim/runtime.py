"""Simulation runtime: servo model, gait/balance controller, water jet, recording."""
from __future__ import annotations

import os
os.environ.setdefault("MUJOCO_GL", "egl")

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import mujoco
import numpy as np

from .. import params as P
from ..kinematics import legs as LK, arm as AK, body as BK
from ..kinematics.gait import Trot, GaitParams, Crawl, CrawlParams
from . import mjcf
from ..massprops import water_mass

STOW = np.radians([0.0, 50.0, -115.0, -80.0])
LEG_J = ("hip_abd", "hip_pitch", "knee")


def quat_to_rpy(q):
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


@dataclass
class Droplet:
    p: np.ndarray
    v: np.ndarray
    vol: float
    alive: bool = True
    landed: bool = False
    target: Optional[int] = None
    t_land: float = 0.0


class AquaSim:
    """High-level wrapper. All public commands use mm / rad / ml."""

    N_DROPS = 240

    def __init__(self, env_xml: str = "", water_ml: float = P.TANK_TARGET_ML, pots=None,
                 width=1280, height=720, fps=30, drops=True):
        drops_xml = ""
        if drops:
            drops_xml = "\n".join(
                f'<body name="drop{i}" mocap="true" pos="0 0 -5"><geom type="sphere" size="0.0026" '
                f'rgba="0.25 0.55 0.95 0.85" contype="0" conaffinity="0"/></body>' for i in range(self.N_DROPS))
        self.xml = mjcf.scene_xml(env_xml + "\n" + drops_xml)
        self.m = mujoco.MjModel.from_xml_string(self.xml)
        self.d = mujoco.MjData(self.m)
        self.dt = self.m.opt.timestep
        self.W, self.H, self.fps = width, height, fps
        self.renderer = None
        self._writer = None
        self.frames: List[np.ndarray] = []
        self.hud: Callable[["AquaSim"], List[str]] | None = None
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 1.3, 125, -18
        self.cam_follow = True
        self.cam_offset = np.array([0.0, 0.0, -0.06])
        self.record = False
        self._last_frame_t = -1.0
        # actuator index map
        self.act = {self.m.actuator(i).name: i for i in range(self.m.nu)}
        self.jq = {self.m.joint(i).name: self.m.joint(i).qposadr[0] for i in range(self.m.njnt)}
        self.jv = {self.m.joint(i).name: self.m.joint(i).dofadr[0] for i in range(self.m.njnt)}
        self.target = np.zeros(self.m.nu)
        self.cmd = np.zeros(self.m.nu)          # slew-limited servo command
        self.cmd_v = np.zeros(self.m.nu)
        names = [self.m.actuator(i).name for i in range(self.m.nu)]
        self.qadr = np.array([self.jq[n] for n in names])
        self.vadr = np.array([self.jv[n] for n in names])
        is_mg = np.array([n == "arm_wrist" for n in names])
        is_sl = np.array([n == "grip_a" for n in names])
        self.slew = np.where(is_mg, P.MG90S.speed, P.DS3218.speed * 0.95)
        self.slew = np.where(is_sl, 0.06, self.slew)                 # m/s jaw speed
        self.tau_max = np.where(is_mg, P.MG90S.stall_torque, P.DS3218.stall_torque)
        self.tau_max = np.where(is_sl, 18.0, self.tau_max)
        self.w0 = np.where(is_mg, P.MG90S.speed * 1.15, P.DS3218.speed * 1.15)   # no-load speed
        self.w0 = np.where(is_sl, 0.08, self.w0)
        self.i_stall = np.where(is_mg, P.MG90S.stall_current, P.DS3218.stall_current)
        self.i_stall = np.where(is_sl, P.MG90S.stall_current, self.i_stall)
        self.i_idle = np.where(is_mg, P.MG90S.idle_current, P.DS3218.idle_current)
        self.kp = np.where(is_mg, 3.5, 32.0); self.kp = np.where(is_sl, 900.0, self.kp)
        self.kd = np.where(is_mg, 0.03, 0.30); self.kd = np.where(is_sl, 12.0, self.kd)
        # water state
        self.water_ml = water_ml
        self.water_body = self.m.body("water").id
        self.pump_on = False
        self.drops: List[Droplet] = []
        self.emitted_ml = 0.0
        self.delivered = {}
        self.pots = pots or []                 # [(x, y, soil_z, radius, geom_name)] metres
        self._emit_acc = 0.0
        self.has_drops = drops
        self.log: Dict[str, list] = {k: [] for k in ("t", "base", "rpy", "tau", "qvel", "power", "contact", "water")}
        self.gait = Trot(GaitParams())
        self.crawl = Crawl(CrawlParams())
        self.height = P.STAND_HEIGHT
        self.body_rpy_cmd = np.zeros(3)
        self.body_shift = np.zeros(3)
        self.balance = True
        self.integ = np.zeros(2)
        self.arm_q = STOW.copy()
        self.foot_offset = {leg: np.zeros(3) for leg in P.LEGS}
        self.grip = P.GRIP_STROKE * 0.6
        self._set_water_mass()

    # ----------------------------------------------------------- low level
    def set_joint_targets(self, q: Dict[str, float]):
        for k, v in q.items():
            self.target[self.act[k]] = v

    def leg_targets_from_feet(self, feet_body: Dict[str, np.ndarray]):
        out = {}
        for leg, p in feet_body.items():
            try:
                q = LK.ik(leg, p)
            except LK.Unreachable as ex:
                q = LK.ik(leg, p, check_limits=False, clamp=True)
                self.ik_clamps = getattr(self, "ik_clamps", 0) + 1
                self.clamp_log = getattr(self, "clamp_log", [])
                if len(self.clamp_log) < 2000:
                    self.clamp_log.append((round(self.d.time, 3), str(ex)))
                lo, hi = LK.limits(leg)
                q = np.clip(q, lo, hi)
            for n, v in zip(LEG_J, q):
                out[f"{leg}_{n}"] = v
        self.set_joint_targets(out)

    def set_arm(self, q4, grip=None):
        self.arm_q = np.asarray(q4, float)
        self.set_joint_targets(dict(arm_yaw=q4[0], arm_shoulder=q4[1], arm_elbow=q4[2], arm_wrist=q4[3]))
        if grip is not None:
            self.grip = grip
        self.target[self.act["grip_a"]] = self.grip / 1000.0

    def init_pose(self, height=None, arm=None):
        height = height or self.height
        self.height = height
        feet = {leg: LK.neutral_foot(leg, height) for leg in P.LEGS}
        self.leg_targets_from_feet(feet)
        self.set_arm(STOW if arm is None else arm, self.grip)
        self.d.qpos[self.qadr] = self.target
        self.cmd[:] = self.target
        self.d.qpos[2] = height / 1000.0 + 0.002
        mujoco.mj_forward(self.m, self.d)

    def _set_water_mass(self):
        self.m.body_mass[self.water_body] = max(self.water_ml, 0.0) / 1000.0 + 1e-4
        _, c = water_mass(max(self.water_ml, 1e-3) / P.TANK_TARGET_ML)
        self.m.body_pos[self.water_body] = c / 1000.0

    # -------------------------------------------------------------- state
    def base_pose(self):
        return self.d.qpos[:3].copy(), self.d.qpos[3:7].copy()

    def rpy(self):
        return quat_to_rpy(self.d.qpos[3:7])

    def site(self, name):
        sid = self.m.site(name).id
        return self.d.site_xpos[sid].copy(), self.d.site_xmat[sid].reshape(3, 3).copy()

    # -------------------------------------------------------------- water
    def _emit(self):
        if not self.pump_on or self.water_ml <= 0:
            return
        q_ml_s = P.PUMP["flow_ml_min"] / 60.0
        rate = 45.0                                   # droplets / s (visual discretisation)
        self._emit_acc += rate * self.dt
        while self._emit_acc >= 1.0:
            self._emit_acc -= 1.0
            p, R = self.site("nozzle_tip")
            dirn = R @ np.array([0, 0, -1.0])
            v = dirn * AK.water_jet_speed() + self._site_velocity("nozzle_tip")
            vol = q_ml_s / rate
            vol = min(vol, self.water_ml)
            self.water_ml -= vol
            self.emitted_ml += vol
            jitter = np.random.default_rng(len(self.drops)).normal(0, 0.012, 3)
            self.drops.append(Droplet(p.copy(), v * (1 + jitter), vol))
        self._set_water_mass()

    def _site_velocity(self, name):
        sid = self.m.site(name).id
        res = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_SITE, sid, res, 0)
        return res[3:]

    def _update_drops(self):
        g = np.array([0, 0, -9.81])
        for dr in self.drops:
            if not dr.alive or dr.landed:
                continue
            dr.v = dr.v + g * self.dt
            dr.p = dr.p + dr.v * self.dt
            zg = 0.0
            hit = None
            for i, (x, y, sz, r, _) in enumerate(self.pots):
                if (dr.p[0] - x) ** 2 + (dr.p[1] - y) ** 2 <= r * r:
                    zg = sz
                    hit = i
                    break
            if dr.p[2] <= zg + 0.0025:
                dr.p[2] = zg + 0.0025
                dr.landed = True
                dr.target = hit
                dr.t_land = self.d.time
                if hit is not None:
                    self.delivered[hit] = self.delivered.get(hit, 0.0) + dr.vol
        # wet soil colouring
        for i, (_, _, _, _, gname) in enumerate(self.pots):
            if gname:
                w = min(1.0, self.delivered.get(i, 0.0) / 10.0)
                dry, wet = np.array([0.30, 0.21, 0.13]), np.array([0.13, 0.08, 0.05])
                self.m.geom_rgba[self.m.geom(gname).id, :3] = dry * (1 - w) + wet * w
        if self.has_drops:
            vis = [d for d in self.drops if d.alive][-self.N_DROPS:]
            for i in range(self.N_DROPS):
                if i < len(vis):
                    self.d.mocap_pos[i] = vis[i].p
                else:
                    self.d.mocap_pos[i] = (0, 0, -5)

    # ----------------------------------------------------------- stepping
    def step(self, n=1):
        for _ in range(n):
            # hobby-servo model: speed-limited command, PD + velocity feed-forward,
            # DC-motor torque/speed envelope
            dmax = self.slew * self.dt
            new = self.cmd + np.clip(self.target - self.cmd, -dmax, dmax)
            self.cmd_v = (new - self.cmd) / self.dt
            self.cmd = new
            q = self.d.qpos[self.qadr]
            qd = self.d.qvel[self.vadr]
            tau = self.kp * (self.cmd - q) + self.kd * (self.cmd_v - qd)
            avail = self.tau_max * np.clip(1.0 - np.abs(qd) / self.w0, 0.0, 1.0)
            lim = np.where(tau * qd > 0, avail, self.tau_max)
            self.d.ctrl[:] = np.clip(tau, -lim, lim)
            mujoco.mj_step(self.m, self.d)
            self._emit()
            self._update_drops()
            self._log()
            if self.record and self.d.time - self._last_frame_t >= 1.0 / self.fps - 1e-9:
                self._grab()
                self._last_frame_t = self.d.time

    def _log(self):
        L = self.log
        L["t"].append(self.d.time)
        L["base"].append(self.d.qpos[:3].copy())
        L["rpy"].append(self.rpy())
        tau = self.d.actuator_force.copy()
        L["tau"].append(tau)
        qv = np.array([self.d.qvel[self.jv[self.m.actuator(i).name]] for i in range(self.m.nu)])
        L["qvel"].append(qv)
        L["power"].append(float(np.sum(np.abs(tau * qv))))
        # servo supply current: DC motor current ∝ torque (holding load costs current, not just motion)
        L.setdefault("current", []).append(float(np.sum(self.i_idle + self.i_stall * np.abs(tau) / self.tau_max)))
        L["contact"].append([self.d.sensor(f"{leg}_contact").data[0] > 0.05 for leg in P.LEGS])
        L["water"].append(self.water_ml)

    def wait(self, seconds, hold=True):
        self.step(int(round(seconds / self.dt)))

    # --------------------------------------------------------- behaviours
    def neutral(self, leg):
        return LK.neutral_foot(leg, self.height) + self.foot_offset[leg]

    def step_feet(self, offsets: Dict[str, np.ndarray], sway=0.7, swing=0.45, lift=24.0):
        """Statically stable re-stepping: for each leg in crawl order sway the CoG into the
        support triangle, lift the foot, place it at neutral+offset, then move on."""
        from ..kinematics.gait import CRAWL_ORDER, _swing_xy, _swing_z
        for leg in CRAWL_ORDER:
            if leg not in offsets:
                continue
            s_ = self.crawl._shift_for(leg, self.height)
            self.stand(sway, shift=(s_[0], s_[1], 0.0), ramp=sway)
            p0 = self.foot_offset[leg].copy()
            p1 = np.asarray(offsets[leg], float)
            n = int(round(swing / self.dt))
            for i in range(n):
                u = (i + 1) / n
                self.foot_offset[leg] = p0 + (p1 - p0) * _swing_xy(u) + np.array([0, 0, lift * _swing_z(u)])
                feet = {l: self.neutral(l) for l in P.LEGS}
                self.leg_targets_from_feet(self._balanced_feet(feet))
                self.step()
            self.foot_offset[leg] = p1.copy()
            self.stand(0.15)
        self.stand(sway, shift=(0, 0, 0), ramp=sway)

    def _balanced_feet(self, feet_body):
        """Attitude feedback: rotate foot targets so the body levels to body_rpy_cmd."""
        r = self.rpy()
        err = self.body_rpy_cmd[:2] - r[:2]
        self.integ = np.clip(self.integ + err * self.dt, -0.15, 0.15)
        corr = 0.6 * err + 1.5 * self.integ if self.balance else np.zeros(2)
        R = BK.rpy_matrix(self.body_rpy_cmd[0] + corr[0], self.body_rpy_cmd[1] + corr[1], 0.0)
        out = {}
        for leg, p in feet_body.items():
            out[leg] = R.T @ (p - self.body_shift)
        return out

    def stand(self, seconds, height=None, rpy=(0, 0, 0), shift=(0, 0, 0), ramp=0.6):
        h0, h1 = self.height, (height or self.height)
        r0, r1 = self.body_rpy_cmd.copy(), np.asarray(rpy, float)
        s0, s1 = self.body_shift.copy(), np.asarray(shift, float)
        n = int(round(seconds / self.dt))
        for i in range(n):
            a = min(1.0, (i * self.dt) / max(ramp, 1e-6))
            a = 3 * a * a - 2 * a * a * a
            self.height = h0 + (h1 - h0) * a
            self.body_rpy_cmd = r0 + (r1 - r0) * a
            self.body_shift = s0 + (s1 - s0) * a
            feet = {leg: self.neutral(leg) for leg in P.LEGS}
            self.leg_targets_from_feet(self._balanced_feet(feet))
            self.step()

    def walk(self, seconds, v=(0.0, 0.0), wz=0.0, ramp=0.5):
        """Trot with velocity command v (m/s, body frame) and yaw rate wz (rad/s)."""
        n = int(round(seconds / self.dt))
        t0 = self.d.time
        vmm = np.asarray(v, float) * 1000.0
        for i in range(n):
            t = self.d.time - t0
            amp = min(1.0, t / ramp, (seconds - t) / ramp)
            amp = max(amp, 0.0)
            tg = self.gait.targets(t, vmm, wz, height=self.height, amp=amp)
            feet = {leg: p for leg, (p, c) in tg.items()}
            self.leg_targets_from_feet(self._balanced_feet(feet))
            self.step()
        # settle into stance
        self.stand(0.3)

    def crawl_walk(self, seconds, v=(0.0, 0.0), wz=0.0, heading_hold=True, sway_time=0.8):
        """Statically stable crawl with CoG sway. v in m/s (body frame), wz rad/s.

        pre-sway (4 feet down) -> whole gait cycles with velocity ramp -> un-sway,
        so every foot/body target stays continuous."""
        cp = self.crawl.cp
        cp.height = self.height
        period_saved = cp.period
        if abs(v[0]) < 1e-6 and abs(v[1]) < 1e-6 and abs(wz) > 0:
            cp.period = P.CRAWL.get("turn_period", cp.period)     # slower cycle for turning in place
        cycles = max(1, int(round(seconds / cp.period)))
        s0 = self.crawl._shift_for("RL", self.height)
        self.stand(sway_time, shift=(s0[0], s0[1], 0.0), ramp=sway_time)
        self.body_shift = np.zeros(3)
        T = cycles * cp.period
        n = int(round(T / self.dt))
        t0 = self.d.time
        vmm = np.asarray(v, float) * 1000.0
        yaw0 = self.rpy()[2]
        ramp = 0.5 * cp.period
        for i in range(n):
            t = i * self.dt
            amp = max(0.0, min(1.0, t / ramp, (T - t) / ramp))
            w = wz
            if heading_hold:
                yaw_ref = yaw0 + wz * min(t, T) * amp
                err = (yaw_ref - self.rpy()[2] + np.pi) % (2 * np.pi) - np.pi
                w = wz + float(np.clip(0.8 * err, -0.06, 0.06))
            tg = self.crawl.targets(t, vmm, w, height=self.height, amp=amp)
            feet = {leg: p for leg, (p, c) in tg.items()}
            self.leg_targets_from_feet(self._balanced_feet(feet))
            self.step()
        self.body_shift = np.array([s0[0], s0[1], 0.0])
        self.stand(sway_time, shift=(0, 0, 0), ramp=sway_time)
        cp.period = period_saved

    def move_arm(self, q_goal, seconds, grip=None, hold_legs=True):
        q0 = self.arm_q.copy()
        g0 = self.grip
        g1 = self.grip if grip is None else grip
        n = int(round(seconds / self.dt))
        for i in range(n):
            s = (i + 1) / n
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            self.set_arm(q0 + (np.asarray(q_goal) - q0) * s, g0 + (g1 - g0) * s)
            if hold_legs:
                feet = {leg: self.neutral(leg) for leg in P.LEGS}
                self.leg_targets_from_feet(self._balanced_feet(feet))
            self.step()

    def arm_line(self, p_goal_world, pitch, seconds, tool=AK.TCP, grip=None):
        """Straight-line Cartesian move of the tool point (world frame, mm)."""
        p0 = self.tool_world(tool)
        g0 = self.grip
        g1 = self.grip if grip is None else grip
        n = int(round(seconds / self.dt))
        for i in range(n):
            s = (i + 1) / n
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            pw = p0 + (np.asarray(p_goal_world) - p0) * s
            pb = self.world_to_body(pw)
            try:
                q = AK.ik(pb, pitch, tool=tool, q_ref=self.arm_q)
                self.set_arm(q, g0 + (g1 - g0) * s)
            except AK.Unreachable as ex:
                self.arm_unreachable = getattr(self, "arm_unreachable", 0) + 1
                self.unreach_log = getattr(self, "unreach_log", [])
                if len(self.unreach_log) < 50 or i == n - 1:
                    self.unreach_log.append((round(self.d.time, 2), pb.round(1).tolist(), str(ex)[:60]))
            feet = {leg: self.neutral(leg) for leg in P.LEGS}
            self.leg_targets_from_feet(self._balanced_feet(feet))
            self.step()

    def world_to_body(self, pw_mm):
        pos, quat = self.base_pose()
        Rm = np.zeros(9); mujoco.mju_quat2Mat(Rm, quat); Rm = Rm.reshape(3, 3)
        return Rm.T @ (np.asarray(pw_mm) / 1000.0 - pos) * 1000.0

    def body_to_world(self, pb_mm):
        pos, quat = self.base_pose()
        Rm = np.zeros(9); mujoco.mju_quat2Mat(Rm, quat); Rm = Rm.reshape(3, 3)
        return (Rm @ (np.asarray(pb_mm) / 1000.0) + pos) * 1000.0

    def tool_world(self, tool=AK.TCP):
        p, _ = AK.fk(self.arm_q, tool)
        return self.body_to_world(p)

    def water(self, ml):
        start = self.emitted_ml
        self.pump_on = True
        while self.emitted_ml - start < ml and self.water_ml > 0:
            feet = {leg: self.neutral(leg) for leg in P.LEGS}
            self.leg_targets_from_feet(self._balanced_feet(feet))
            self.step()
        self.pump_on = False
        self.stand(0.9)   # let the jet finish falling

    # ---------------------------------------------------------- recording
    def _grab(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.m, self.H, self.W)
            self.opt = mujoco.MjvOption()
        if self.cam_follow:
            pos = self.d.qpos[:3]
            target = pos + self.cam_offset
            self.cam.lookat[:] = self.cam.lookat * 0.9 + target * 0.1 if self.frames else target
        self.renderer.update_scene(self.d, self.cam, self.opt)
        img = self.renderer.render().copy()
        if self.hud:
            img = draw_hud(img, self.hud(self))
        self.n_frames = getattr(self, "n_frames", 0) + 1
        if self._writer is None:                       # stream to a temp file: constant memory
            import imageio.v2 as imageio, tempfile
            self._tmp_video = Path(tempfile.mkstemp(suffix=".mp4")[1])
            self._writer = imageio.get_writer(str(self._tmp_video), fps=self.fps, codec="libx264", quality=None,
                                              ffmpeg_params=["-crf", "23", "-preset", "veryfast", "-pix_fmt", "yuv420p",
                                                             "-movflags", "+faststart"])
        self._writer.append_data(img)
        if not self.frames:
            self.frames.append(img)                    # keep first frame only (thumbnail)

    def save_video(self, path: Path, crf=23):
        import shutil
        path.parent.mkdir(parents=True, exist_ok=True)
        if self._writer is not None:
            self._writer.close()
            self._writer = None
            shutil.move(str(self._tmp_video), str(path))
        return path

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    # ------------------------------------------------------------ metrics
    def metrics(self, t0=0.0):
        t = np.array(self.log["t"])
        sel = t >= t0
        tau = np.array(self.log["tau"])[sel]
        names = [self.m.actuator(i).name for i in range(self.m.nu)]
        stall = np.array([P.DS3218.stall_torque if n not in ("arm_wrist", "grip_a") else
                          (P.MG90S.stall_torque if n == "arm_wrist" else 18.0) for n in names])
        frac = np.abs(tau) / stall
        k = max(1, int(round(0.05 / self.dt)))
        ker = np.ones(k) / k
        filt = np.array([np.convolve(frac[:, i], ker, mode="valid") for i in range(16)]).T if len(frac) > k else frac[:, :16]
        rpy = np.array(self.log["rpy"])[sel]
        base = np.array(self.log["base"])[sel]
        return dict(
            peak_torque_frac_50ms=float(filt.max()),
            peak_joint_50ms=names[int(np.argmax(filt.max(axis=0)))],
            p99_torque_frac=float(np.percentile(frac[:, :16], 99)),
            peak_torque_frac=float(frac[:, :16].max()),
            peak_joint=names[int(np.argmax(frac[:, :16].max(axis=0)))],
            rms_torque_frac=float(np.sqrt((frac[:, :16] ** 2).mean())),
            max_tilt_deg=float(np.degrees(np.abs(rpy[:, :2]).max())),
            min_base_z_mm=float(base[:, 2].min() * 1000),
            mean_mech_power_W=float(np.mean(np.array(self.log["power"])[sel])),
            mean_servo_current_A=float(np.mean(np.array(self.log["current"])[sel])),
            peak_servo_current_A=float(np.max(np.array(self.log["current"])[sel])),
        )


def draw_hud(img, lines, origin=(24, 22)):
    from PIL import Image, ImageDraw, ImageFont
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im, "RGBA")
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 22)
        small = ImageFont.truetype("DejaVuSans.ttf", 18)
    except Exception:
        font = small = ImageFont.load_default()
    x, y = origin
    w = max(dr.textlength(l, font=font if i == 0 else small) for i, l in enumerate(lines)) + 28
    h = 30 + 26 * (len(lines) - 1) + 14
    dr.rounded_rectangle([x - 12, y - 10, x - 12 + w, y - 10 + h], 10, fill=(15, 22, 30, 170))
    for i, l in enumerate(lines):
        dr.text((x, y), l, fill=(235, 245, 255, 255) if i else (120, 210, 255, 255), font=font if i == 0 else small)
        y += 30 if i == 0 else 26
    return np.asarray(im)

"""AquaBot link tree — the single kinematic definition.

CAD assembly, analytic IK (validated against this FK), the MuJoCo model,
the URDF and the BOM quantities are all generated from `build_tree()`.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import params as P


# ------------------------------------------------------------- transforms
def T(x=0.0, y=0.0, z=0.0, R=None) -> np.ndarray:
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    M[:3, 3] = (x, y, z)
    return M


def Rx(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def cols(*v) -> np.ndarray:
    """Rotation from column vectors (images of local X, Y, Z)."""
    R = np.array(v, float).T
    assert abs(np.linalg.det(R) - 1.0) < 1e-9, R
    return R


def axis_angle(axis, ang_rad) -> np.ndarray:
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang_rad) * K + (1 - np.cos(ang_rad)) * K @ K


def rot_to_quat(R) -> Tuple[float, float, float, float]:
    """(w, x, y, z)"""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S; x = (m[2, 1] - m[1, 2]) / S; y = (m[0, 2] - m[2, 0]) / S; z = (m[1, 0] - m[0, 1]) / S
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / S; x = 0.25 * S; y = (m[0, 1] + m[1, 0]) / S; z = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / S; x = (m[0, 1] + m[1, 0]) / S; y = 0.25 * S; z = (m[1, 2] + m[2, 1]) / S
    else:
        S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / S; x = (m[0, 2] + m[2, 0]) / S; y = (m[1, 2] + m[2, 1]) / S; z = 0.25 * S
    q = np.array([w, x, y, z]); q /= np.linalg.norm(q)
    if q[0] < 0: q = -q
    return tuple(float(v) for v in q)


# Standard servo orientations used throughout
R_SERVO_L = cols((0, 0, 1), (1, 0, 0), (0, 1, 0))      # X_s->+Z, Y_s->+X, Z_s(shaft)->+Y
R_SERVO_R = cols((0, 0, 1), (-1, 0, 0), (0, -1, 0))    # mirror: shaft -> -Y
R_J1_FRONT = cols((0, 0, -1), (0, 1, 0), (1, 0, 0))    # shaft -> +X, bulk down
R_YAW = Rz(-90)                                         # shaft +Z, bulk -> -Y
R_SHOULDER_SERVO = cols((-1, 0, 0), (0, 0, -1), (0, -1, 0))
R_UPPER_ARM = cols((0, 0, -1), (0, -1, 0), (-1, 0, 0))  # link X'=down, Y'=-y, Z'=-x(back)
R_GRIP_SERVO = cols((0, 1, 0), (1, 0, 0), (0, 0, -1))
R_BRG_PY = Rx(-90)    # bearing axis +Z -> +Y
R_BRG_NY = Rx(90)


@dataclass
class Joint:
    name: str
    type: str                 # revolute | prismatic
    axis: Tuple[float, float, float]
    lower: float              # rad or mm
    upper: float
    actuator: Optional[str] = None     # servo spec name
    mimic: Optional[Tuple[str, float]] = None   # (joint, multiplier)


@dataclass
class Instance:
    name: str
    part: str
    T: np.ndarray


@dataclass
class Link:
    name: str
    parent: Optional[str]
    origin: np.ndarray         # 4x4 in parent frame at q = 0
    joint: Optional[Joint]
    instances: List[Instance] = field(default_factory=list)


def _deg(a):
    return np.radians(a)


@lru_cache(maxsize=1)
def build_tree() -> Dict[str, Link]:
    from .cad import parts_body as PB, parts_arm as PA
    S = P.DS3218
    F = P.FORK
    links: Dict[str, Link] = {}

    # --------------------------------------------------------------- base
    base = Link("base", None, np.eye(4), None)
    I = np.eye(4)
    add = lambda nm, part, M=I: base.instances.append(Instance(nm, part, M))
    add("chassis_tub", "chassis_tub")
    add("bulkhead_front", "bulkhead_front")
    add("bulkhead_rear", "bulkhead_rear", T(R=Rz(180)))
    add("top_deck", "top_deck")
    add("tank_body", "tank_body")
    add("tank_lid", "tank_lid")
    add("fill_cap", "fill_cap")
    add("electronics_tray", "electronics_tray")
    add("electronics_cover", "electronics_cover")
    add("arm_pedestal", "arm_pedestal")
    add("arm_bearing_plate", "arm_bearing_plate")
    for leg in P.LEGS:
        e, s = P.sign_e(leg), P.sign_s(leg)
        R = R_J1_FRONT if e > 0 else Rz(180) @ R_J1_FRONT
        add(f"servo_{leg}_J1", "servo_ds3218", T(e * P.J1_CASE_TOP_X, s * P.Y_J1, 0.0, R))
    add("servo_arm_yaw", "servo_ds3218", T(PB.YAW["x"], PB.YAW["y"], PB.YAW_CASE_TOP, R_YAW))
    add("bearing_yaw", "bearing_61806", T(PB.YAW["x"], PB.YAW["y"], PB.BEAR_Z0))
    bt = P.BATTERY
    add("battery", "battery_2s_2200", T(bt["x0"], 0.0, bt["z0"]))
    zl = P.TANK["z1"] + P.LID_T + 0.5
    add("pump", "pump_peristaltic", T(-64.0, -14.0, zl))
    add("ubec_L", "ubec_6v_10a", T(20.0, 19.0, zl))
    add("ubec_R", "ubec_6v_10a", T(20.0, -42.5, zl))
    add("buck_5v", "buck_5v", T(2.0, 22.0, zl))
    add("imu", "imu_mpu6050", T(-26.0, 22.0, zl))
    add("tof", "tof_vl53l0x", T(86.0, -12.5, -19.0, cols((0, 1, 0), (0, 0, 1), (1, 0, 0))))
    add("rocker_switch", "rocker_switch", T(-P.TUB_X - P.BULKHEAD_T, 0.0, -12.5, Rz(180)))
    add("xt60_port", "xt60_panel_mount", T(-P.TUB_X - P.BULKHEAD_T, 0.0, 24.0, Rz(180)))
    tz = PB.TRAY["z0"] + PB.TRAY["t"]
    add("pca9685", "pca9685", T(PB.PCA_POS[0], PB.PCA_POS[1], tz + 3.0))
    add("esp32", "esp32_devkitc", T(PB.ESP_POS[0], PB.ESP_POS[1], tz + 5.0))
    add("mosfet", "mosfet_module", T(PB.MOSFET_POS[0], PB.MOSFET_POS[1], tz))
    links["base"] = base

    # --------------------------------------------------------------- legs
    jx, jy, jz = P.J2_DX, P.J2_DY, P.J2_DZ
    for leg in P.LEGS:
        e, s = P.sign_e(leg), P.sign_s(leg)
        hand = "L" if (e * s) > 0 else "R"       # FL & RR use the L bracket
        Rh = np.eye(3) if e > 0 else Rz(180)     # rear legs: bracket rotated 180° about Z
        # contents of the hip link in an FL-like (hand) frame, then rotated
        Rsv = R_SERVO_L if hand == "L" else R_SERVO_R
        sy = 1.0 if hand == "L" else -1.0
        hip_items = [
            Instance(f"shoulder_bracket_{leg}", f"shoulder_bracket_{hand}", I),
            Instance(f"servo_{leg}_J2", "servo_ds3218", T(jx, sy * (jy + F["case_top"]), jz, Rsv)),
            Instance(f"horn_{leg}_J1", "horn_ds3218", T(-S.horn_top, 0.0, 0.0, R_J1_FRONT)),
            Instance(f"bearing_{leg}_J2", "bearing_625",
                     T(jx, sy * (jy + F["idler"][1] - P.IDLER_BEARING["B"]), jz, R_BRG_PY if hand == "L" else R_BRG_NY)),
        ]
        hip = Link(f"{leg}_hip", "base", T(e * P.J1_HORN_X, s * P.Y_J1, 0.0),
                   Joint(f"{leg}_hip_abd", "revolute", (1, 0, 0), *_abd_limits(leg), "DS3218"))
        for it in hip_items:
            hip.instances.append(Instance(it.name, it.part, T(R=Rh) @ it.T))
        links[hip.name] = hip

        side = "L" if s > 0 else "R"
        Rs = R_SERVO_L if side == "L" else R_SERVO_R
        ss = 1.0 if side == "L" else -1.0
        # Rear knees point forward: use the opposite-hand femur/tibia turned 180° about Z,
        # which keeps the horn outboard but moves the knee idler bridge to the front.
        pside = side if e > 0 else ("R" if side == "L" else "L")
        Mlink = I if e > 0 else T(R=Rz(180))
        lo, hi = P.hip_pitch_limits(leg)
        fem = Link(f"{leg}_femur", hip.name, T(e * jx, s * jy, jz),
                   Joint(f"{leg}_hip_pitch", "revolute", (0, 1, 0), _deg(lo), _deg(hi), "DS3218"))
        fem.instances += [
            Instance(f"femur_{leg}", f"femur_{pside}", Mlink),
            Instance(f"servo_{leg}_J3", "servo_ds3218", T(0, ss * F["case_top"], -P.L_FEMUR, Rs)),
            Instance(f"horn_{leg}_J2", "horn_ds3218", T(0, ss * F["case_top"], 0, Rs)),
            Instance(f"bearing_{leg}_J3", "bearing_625",
                     T(0, ss * (F["idler"][1] - P.IDLER_BEARING["B"]), -P.L_FEMUR, R_BRG_PY if side == "L" else R_BRG_NY)),
        ]
        links[fem.name] = fem
        lo, hi = P.knee_limits(leg)
        tib = Link(f"{leg}_tibia", fem.name, T(0, 0, -P.L_FEMUR),
                   Joint(f"{leg}_knee", "revolute", (0, 1, 0), _deg(lo), _deg(hi), "DS3218"))
        tib.instances += [
            Instance(f"tibia_{leg}", f"tibia_{pside}", Mlink),
            Instance(f"foot_{leg}", "foot_pad", I),
            Instance(f"horn_{leg}_J3", "horn_ds3218", T(0, ss * F["case_top"], 0, Rs)),
        ]
        links[tib.name] = tib

    # ---------------------------------------------------------------- arm
    tt0 = PA.TT_ORIGIN
    zsh = P.ARM_SHOULDER_Z - tt0[2]
    lo, hi = P.LIMITS["arm_yaw"]
    tt = Link("arm_turntable", "base", T(*tt0),
              Joint("arm_yaw", "revolute", (0, 0, 1), _deg(lo), _deg(hi), "DS3218"))
    tt.instances += [
        Instance("arm_turntable", "arm_turntable", I),
        Instance("horn_arm_yaw", "horn_ds3218", T(0, 0, -S.horn_top, R_YAW)),
        Instance("servo_arm_shoulder", "servo_ds3218", T(0, -F["case_top"], zsh, R_SHOULDER_SERVO)),
        Instance("bearing_arm_shoulder", "bearing_625", T(0, -F["idler"][1], zsh, R_BRG_PY)),
    ]
    links[tt.name] = tt
    lo, hi = P.LIMITS["arm_shoulder"]
    ua = Link("arm_upper", tt.name, T(0, 0, zsh, R_UPPER_ARM),
              Joint("arm_shoulder", "revolute", (0, 1, 0), _deg(lo), _deg(hi), "DS3218"))
    ua.instances += [
        Instance("upper_arm", "upper_arm", I),
        Instance("servo_arm_elbow", "servo_ds3218", T(0, F["case_top"], -P.L_ARM1, R_SERVO_L)),
        Instance("horn_arm_shoulder", "horn_ds3218", T(0, F["case_top"], 0, R_SERVO_L)),
        Instance("bearing_arm_elbow", "bearing_625", T(0, F["idler"][1] - P.IDLER_BEARING["B"], -P.L_ARM1, R_BRG_PY)),
    ]
    links[ua.name] = ua
    lo, hi = P.LIMITS["arm_elbow"]
    fa = Link("arm_fore", ua.name, T(0, 0, -P.L_ARM1),
              Joint("arm_elbow", "revolute", (0, 1, 0), _deg(lo), _deg(hi), "DS3218"))
    fa.instances += [
        Instance("forearm", "forearm", I),
        Instance("servo_arm_wrist", "servo_mg90s", T(0, PA.MG_CASE_TOP_Y, -P.L_ARM2, R_SERVO_L)),
        Instance("horn_arm_elbow", "horn_ds3218", T(0, F["case_top"], 0, R_SERVO_L)),
    ]
    links[fa.name] = fa
    lo, hi = P.LIMITS["arm_wrist"]
    wr = Link("arm_wrist", fa.name, T(0, 0, -P.L_ARM2),
              Joint("arm_wrist", "revolute", (0, 1, 0), _deg(lo), _deg(hi), "MG90S"))
    wr.instances += [
        Instance("gripper_base", "gripper_base", I),
        Instance("horn_arm_wrist", "horn_mg90s", T(0, PA.MG_CASE_TOP_Y, 0, R_SERVO_L)),
        Instance("servo_gripper", "servo_mg90s", T(0, 0, PA.GR["servo_top"], R_GRIP_SERVO)),
        Instance("rod_gripper_upper", "rod_3x74", T(PA.GR["rod_x"], 0, PA.GR["rod_z"])),
        Instance("rod_gripper_lower", "rod_3x74", T(-PA.GR["rod_x"], 0, PA.GR["rod_z"])),
        Instance("nozzle", "nozzle_brass", T(PA.GR["nozzle_x"], 0, -P.NOZZLE_OFF[0])),
    ]
    links[wr.name] = wr
    lo, hi = P.LIMITS["grip"]
    ja = Link("jaw_a", wr.name, T(), Joint("grip_a", "prismatic", (0, -1, 0), lo, hi, "MG90S"))
    ja.instances.append(Instance("gripper_jaw_a", "gripper_jaw", I))
    jb = Link("jaw_b", wr.name, T(R=Rz(180)), Joint("grip_b", "prismatic", (0, -1, 0), lo, hi, None,
                                                       mimic=("grip_a", 1.0)))
    jb.instances.append(Instance("gripper_jaw_b", "gripper_jaw", I))
    pn = Link("pinion", wr.name, T(), Joint("grip_pinion", "revolute", (0, 0, 1), -4.0, 4.0, None,
                                            mimic=("grip_a", -1.0 / PA.R_PITCH)))
    pn.instances.append(Instance("gripper_pinion", "gripper_pinion", I))
    pn.instances.append(Instance("horn_gripper", "horn_mg90s", T(0, 0, PA.GR["servo_top"], R_GRIP_SERVO)))
    for l in (ja, jb, pn):
        links[l.name] = l
    return links


def _abd_limits(leg):
    lo, hi = P.LIMITS["hip_abd"]
    inward = 12.0     # inward abduction limited by left/right idler-nut clearance
    if P.sign_s(leg) > 0:
        return _deg(-inward), _deg(hi)
    return _deg(lo), _deg(inward)


def joint_names(include_mimic=False) -> List[str]:
    out = []
    for l in build_tree().values():
        if l.joint and (include_mimic or l.joint.mimic is None):
            out.append(l.joint.name)
    return out


def link_world(q: Dict[str, float] | None = None, base_T: np.ndarray | None = None) -> Dict[str, np.ndarray]:
    """World transform of every link (mm). q: joint values (rad / mm)."""
    q = dict(q or {})
    tree = build_tree()
    for l in tree.values():   # mimic joints
        if l.joint and l.joint.mimic:
            src, k = l.joint.mimic
            q[l.joint.name] = k * q.get(src, 0.0)
    out = {}
    base_T = np.eye(4) if base_T is None else base_T
    for name, l in tree.items():          # insertion order is topological
        parent = out[l.parent] if l.parent else base_T
        M = parent @ l.origin
        if l.joint:
            v = q.get(l.joint.name, 0.0)
            J = np.eye(4)
            if l.joint.type == "revolute":
                J[:3, :3] = axis_angle(l.joint.axis, v)
            else:
                J[:3, 3] = np.asarray(l.joint.axis) * v
            M = M @ J
        out[name] = M
    return out


def world_instances(q=None, base_T=None) -> List[Tuple[str, str, np.ndarray, str]]:
    """[(instance_name, part_key, T_world, link_name)]"""
    lw = link_world(q, base_T)
    res = []
    for name, l in build_tree().items():
        for it in l.instances:
            res.append((it.name, it.part, lw[name] @ it.T, name))
    return res


def foot_point(leg: str, q=None, base_T=None) -> np.ndarray:
    lw = link_world(q, base_T)
    return (lw[f"{leg}_tibia"] @ np.array([0, 0, -P.L_TIBIA, 1.0]))[:3]


def tcp_point(q=None, base_T=None) -> np.ndarray:
    lw = link_world(q, base_T)
    return (lw["arm_wrist"] @ np.array([0, 0, -P.L_TCP, 1.0]))[:3]


def nozzle_tip(q=None, base_T=None):
    lw = link_world(q, base_T)
    M = lw["arm_wrist"]
    from .cad.parts_arm import GR
    p = (M @ np.array([GR["nozzle_x"], 0, -P.NOZZLE_OFF[0], 1.0]))[:3]
    d = M[:3, :3] @ np.array([0, 0, -1.0])
    return p, d

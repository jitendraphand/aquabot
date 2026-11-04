"""Analytic 4-DOF arm kinematics (yaw + 3 pitch joints) with tool offsets.

Planar convention (after yaw): r = horizontal distance from the yaw axis,
z = height above the shoulder axis. Absolute link angles measured up from
horizontal: phi1 = q_shoulder, phi2 = phi1 + q_elbow, phi3 = phi2 + q_wrist.
Tool frame: +a along the gripper (towards the fingers), +b 'above' the tool.
"""
from __future__ import annotations

import numpy as np

from .. import params as P
from ..cad import parts_body as PB

L1, L2 = P.L_ARM1, P.L_ARM2
SHOULDER = np.array([PB.YAW["x"], PB.YAW["y"], P.ARM_SHOULDER_Z])
TCP = (P.L_TCP, 0.0)
NOZZLE = P.NOZZLE_OFF            # (a, b)


class Unreachable(ValueError):
    pass


def limits():
    from ..robot import build_tree
    t = build_tree()
    names = ["arm_turntable", "arm_upper", "arm_fore", "arm_wrist"]
    return (np.array([t[n].joint.lower for n in names]), np.array([t[n].joint.upper for n in names]))


def fk(q, tool=TCP):
    """Return (tool point in body frame, tool pitch phi3)."""
    qy, qs, qe, qw = q
    p1, p2, p3 = qs, qs + qe, qs + qe + qw
    r = L1 * np.cos(p1) + L2 * np.cos(p2) + tool[0] * np.cos(p3) - tool[1] * np.sin(p3)
    z = L1 * np.sin(p1) + L2 * np.sin(p2) + tool[0] * np.sin(p3) + tool[1] * np.cos(p3)
    p = SHOULDER + np.array([r * np.cos(qy), r * np.sin(qy), z])
    return p, p3


def pitch_outward(q, tool=TCP) -> float:
    """Tool pitch referenced to the outward horizontal (yaw axis -> tool point).

    `fk` reports the pitch in the arm's own yaw frame; when the tool point lies
    behind the yaw axis (r < 0) the outward-referenced pitch is pi - phi3.
    This is the convention `ik` expects."""
    qy, qs, qe, qw = q
    p1, p2, p3 = qs, qs + qe, qs + qe + qw
    r = L1 * np.cos(p1) + L2 * np.cos(p2) + tool[0] * np.cos(p3) - tool[1] * np.sin(p3)
    ph = p3 if r >= 0 else np.pi - p3
    return (ph + np.pi) % (2 * np.pi) - np.pi


def ik_all(p, phi3, tool=TCP):
    """All analytic solutions (yaw branch x elbow branch) for tool point p and pitch phi3."""
    d = np.asarray(p, float) - SHOULDER
    sols = []
    for flip in (False, True):
        qy = np.arctan2(d[1], d[0]) + (np.pi if flip else 0.0)
        qy = (qy + np.pi) % (2 * np.pi) - np.pi
        r = np.hypot(d[0], d[1]) * (-1.0 if flip else 1.0)
        ph = (np.pi - phi3) if flip else phi3
        z = d[2]
        a, b = tool
        wr = r - a * np.cos(ph) + b * np.sin(ph)
        wz = z - a * np.sin(ph) - b * np.cos(ph)
        D = (wr * wr + wz * wz - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        if abs(D) > 1.0:
            continue
        for up in (True, False):
            qe = -np.arccos(D) if up else np.arccos(D)
            qs = np.arctan2(wz, wr) - np.arctan2(L2 * np.sin(qe), L1 + L2 * np.cos(qe))
            qw = ph - qs - qe
            q = np.array([qy, qs, qe, qw])
            q[1:] = (q[1:] + np.pi) % (2 * np.pi) - np.pi
            sols.append(q)
    return sols


def ik(p, phi3, tool=TCP, elbow_up: bool = True, check_limits: bool = True, q_ref=None):
    """Tool point p (body frame, mm) and tool pitch phi3 (rad, + = up) -> q (4,).

    Picks the solution inside the joint limits closest to q_ref (default: the
    elbow-up, forward-yaw branch)."""
    sols = ik_all(p, phi3, tool)
    if not sols:
        raise Unreachable("arm: wrist out of reach")
    # keep only exact solutions
    sols = [q for q in sols if np.linalg.norm(fk(q, tool)[0] - np.asarray(p)) < 1e-6]
    if not sols:
        raise Unreachable("arm: no consistent branch")
    lo, hi = limits()
    ok = [q for q in sols if np.all(q >= lo - 1e-9) and np.all(q <= hi + 1e-9)]
    if check_limits and not ok:
        raise Unreachable(f"arm: joint limits (best {np.degrees(sols[0]).round(1)})")
    pool = ok if ok else sols
    if q_ref is None:
        pool = sorted(pool, key=lambda q: (0 if (q[2] < 0) == elbow_up else 1, abs(q[0])))
        return pool[0]
    return min(pool, key=lambda q: np.linalg.norm(q - q_ref))


def water_jet_speed() -> float:
    """Nozzle exit velocity (m/s) from pump flow and orifice."""
    Q = P.PUMP["flow_ml_min"] * 1e-6 / 60.0
    A = np.pi * (P.NOZZLE_ORIFICE * 1e-3 / 2) ** 2
    return Q / A


def aim_nozzle(target, approach_from, pitch=np.radians(-50.0), height=60.0, v0=None, g=9.81):
    """Ballistic aiming: nozzle pose so that the jet lands on `target` (body frame, mm).

    approach_from: unit horizontal vector pointing from the target towards the robot.
    Returns (nozzle_tip_point, pitch).
    """
    v0 = water_jet_speed() if v0 is None else v0
    vh = v0 * np.cos(pitch)
    vz = v0 * np.sin(pitch)
    h = height / 1000.0
    t = (vz + np.sqrt(vz * vz + 2 * g * h)) / g          # time to fall `height`
    dist = vh * t * 1000.0
    u = np.asarray(approach_from, float); u[2] = 0.0; u /= np.linalg.norm(u)
    tip = np.asarray(target, float) + u * dist + np.array([0, 0, height])
    return tip, pitch


def jacobian(q, tool=TCP, eps=1e-6):
    J = np.zeros((3, 4))
    for i in range(4):
        dq = np.zeros(4); dq[i] = eps
        J[:, i] = (fk(np.asarray(q) + dq, tool)[0] - fk(np.asarray(q) - dq, tool)[0]) / (2 * eps)
    return J

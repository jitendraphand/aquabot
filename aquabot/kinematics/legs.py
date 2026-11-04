"""Analytic 3-DOF leg kinematics (body frame, mm, rad).

Chain (per leg, e = ±1 front/rear, s = ±1 left/right):
  J1 hip abduction : axis +X through  H1 = (e·X1, s·Y1, 0)
  J2 hip pitch     : axis +Y at       H1 + Rx(q1)·(e·DX, s·DY, DZ)
  J3 knee          : axis +Y at       J2 + Rx(q1)Ry(q2)·(0, 0, -L1)
  foot centre      :                  J3 + Rx(q1)Ry(q2+q3)·(0, 0, -L2)
Knee layout is ANYmal-style 'X': front knees point backwards (q3 < 0),
rear knees point forwards (q3 > 0).
See docs/kinematics.md for the derivation.
"""
from __future__ import annotations

import numpy as np

from .. import params as P

L1, L2 = P.L_FEMUR, P.L_TIBIA
X1, Y1 = P.J1_HORN_X, P.Y_J1
DX, DY, DZ = P.J2_DX, P.J2_DY, P.J2_DZ


class Unreachable(ValueError):
    pass


def hip_origin(leg: str) -> np.ndarray:
    return np.array([P.sign_e(leg) * X1, P.sign_s(leg) * Y1, 0.0])


def j2_offset(leg: str) -> np.ndarray:
    return np.array([P.sign_e(leg) * DX, P.sign_s(leg) * DY, DZ])


def neutral_foot(leg: str, height: float = P.STAND_HEIGHT) -> np.ndarray:
    """Foot centre directly below J2 (plus configured stance offsets) at a body height."""
    e, s = P.sign_e(leg), P.sign_s(leg)
    x = e * (X1 + DX - P.GAIT["foot_in"]) + P.GAIT["stance_dx"]
    y = s * (Y1 + DY + P.GAIT["stance_dy"])
    return np.array([x, y, -height + P.FOOT_R])


def fk(leg: str, q) -> np.ndarray:
    q1, q2, q3 = q
    c1, s1 = np.cos(q1), np.sin(q1)
    xs = -L1 * np.sin(q2) - L2 * np.sin(q2 + q3)
    zs = -L1 * np.cos(q2) - L2 * np.cos(q2 + q3)
    o = j2_offset(leg)
    v = np.array([o[0] + xs, o[1], o[2] + zs])          # in the un-rolled hip frame
    Rx = np.array([[1, 0, 0], [0, c1, -s1], [0, s1, c1]])
    return hip_origin(leg) + Rx @ v


def ik(leg: str, p, check_limits: bool = True, clamp: bool = False) -> np.ndarray:
    """Foot centre position (body frame) -> joint angles (q1, q2, q3).

    clamp=True returns the nearest straight-leg solution for out-of-reach
    targets instead of raising (used by the real-time controller)."""
    v = np.asarray(p, float) - hip_origin(leg)
    o = j2_offset(leg)
    a = o[1]
    r2 = v[1] ** 2 + v[2] ** 2 - a ** 2
    if r2 <= 0:
        if not clamp:
            raise Unreachable(f"{leg}: target inside abduction radius")
        r2 = 1.0
    b = -np.sqrt(r2)                                     # z of foot in un-rolled frame (below)
    q1 = np.arctan2(v[2], v[1]) - np.arctan2(b, a)
    q1 = (q1 + np.pi) % (2 * np.pi) - np.pi
    xs = v[0] - o[0]
    zs = b - o[2]
    u, w = -xs, -zs
    D = (u * u + w * w - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(D) > 1.0:
        if not clamp:
            raise Unreachable(f"{leg}: target out of reach (D={D:.3f})")
        D = float(np.clip(D, -0.999, 0.999))
    q3 = P.knee_sign(leg) * np.arccos(D)
    q2 = np.arctan2(u, w) - np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))
    q = np.array([q1, q2, q3])
    if check_limits:
        lo, hi = limits(leg)
        if np.any(q < lo - 1e-9) or np.any(q > hi + 1e-9):
            raise Unreachable(f"{leg}: joint limit {np.degrees(q).round(1)}")
    return q


def limits(leg: str):
    from ..robot import build_tree
    t = build_tree()
    names = [f"{leg}_hip", f"{leg}_femur", f"{leg}_tibia"]
    lo = np.array([t[n].joint.lower for n in names])
    hi = np.array([t[n].joint.upper for n in names])
    return lo, hi


def jacobian(leg: str, q, eps: float = 1e-6) -> np.ndarray:
    """d(foot)/dq, 3x3 (mm/rad), central differences on the analytic FK."""
    J = np.zeros((3, 3))
    for i in range(3):
        dq = np.zeros(3); dq[i] = eps
        J[:, i] = (fk(leg, np.asarray(q) + dq) - fk(leg, np.asarray(q) - dq)) / (2 * eps)
    return J


def static_torques(leg: str, q, F_world_N) -> np.ndarray:
    """Joint torques (N·m) holding a ground reaction force F (N, body frame) at the foot."""
    J = jacobian(leg, q) / 1000.0     # m/rad
    return J.T @ np.asarray(F_world_N, float)

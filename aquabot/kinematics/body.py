"""Whole-body IK: body pose (6-DOF) + world foot positions -> 12 leg angles."""
from __future__ import annotations

import numpy as np

from .. import params as P
from . import legs


def rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def body_ik(feet_world: dict, pos, rpy) -> dict:
    """feet_world: {leg: xyz (mm, world)}; pos: body origin (mm, world); rpy (rad)."""
    R = rpy_matrix(*rpy)
    q = {}
    for leg, pw in feet_world.items():
        pb = R.T @ (np.asarray(pw, float) - np.asarray(pos, float))
        q[leg] = legs.ik(leg, pb)
    return q


def stance_feet_world(height=P.STAND_HEIGHT):
    """Neutral foot positions in a world frame whose origin is under the body origin."""
    out = {}
    for leg in P.LEGS:
        p = legs.neutral_foot(leg, height)
        out[leg] = np.array([p[0], p[1], P.FOOT_R])
    return out

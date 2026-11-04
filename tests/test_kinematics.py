import numpy as np
import pytest

from aquabot import params as P, robot as R
from aquabot.kinematics import legs as LK, arm as AK
from aquabot.kinematics.gait import Crawl


@pytest.mark.parametrize("leg", P.LEGS)
def test_leg_ik_roundtrip_and_tree(leg):
    rng = np.random.default_rng(0)
    lo, hi = LK.limits(leg)
    for _ in range(500):
        q = rng.uniform(lo, hi)
        p = LK.fk(leg, q)
        assert np.linalg.norm(LK.fk(leg, LK.ik(leg, p, check_limits=False)) - p) < 1e-6
        qd = {f"{leg}_hip_abd": q[0], f"{leg}_hip_pitch": q[1], f"{leg}_knee": q[2]}
        assert np.linalg.norm(R.foot_point(leg, qd) - p) < 1e-6


def test_knee_layout():
    for leg in P.LEGS:
        q = LK.ik(leg, LK.neutral_foot(leg))
        assert np.sign(q[2]) == P.knee_sign(leg)


@pytest.mark.parametrize("tool", [AK.TCP, AK.NOZZLE])
def test_arm_ik_roundtrip(tool):
    rng = np.random.default_rng(1)
    lo, hi = AK.limits()
    for _ in range(500):
        q = rng.uniform(lo, hi)
        p, _ = AK.fk(q, tool)
        q2 = AK.ik(p, AK.pitch_outward(q, tool), tool=tool, check_limits=False)
        assert np.linalg.norm(AK.fk(q2, tool)[0] - p) < 1e-6


def test_aim_nozzle_lands_on_target():
    tgt = np.array([55.0, 235.0, -113.0])
    tip, pitch = AK.aim_nozzle(tgt, np.array([0.0, -1.0, 0.0]), np.radians(-50), 55.0)
    v0 = AK.water_jet_speed()
    v = v0 * np.array([0.0, np.cos(pitch), np.sin(pitch)])   # towards +y (away from robot)
    p = tip / 1000.0
    dt = 1e-4
    while p[2] > tgt[2] / 1000.0:
        v[2] -= 9.81 * dt
        p = p + v * dt
    assert np.linalg.norm(p[:2] * 1000 - tgt[:2]) < 1.0


def test_crawl_keeps_three_feet_down():
    c = Crawl()
    for t in np.linspace(0, c.cp.period, 200, endpoint=False):
        tg = c.targets(t, (35.0, 0.0), 0.0, P.STAND_HEIGHT, 1.0)
        assert sum(1 for (_, contact) in tg.values() if contact) >= 3

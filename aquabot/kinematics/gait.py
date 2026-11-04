"""Trot gait generator (body-frame foot targets) with Bezier swing trajectories."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .. import params as P
from . import legs

PHASE_OFFSET = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}


@dataclass
class GaitParams:
    period: float = P.GAIT["period"]
    duty: float = P.GAIT["duty"]
    step_height: float = P.GAIT["step_height"]
    height: float = P.STAND_HEIGHT


def _swing_xy(s):
    """Smooth 0..1 horizontal progress (min-jerk)."""
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def _swing_z(s):
    """Lift profile 64 s^3 (1-s)^3: zero velocity AND acceleration at lift-off/touchdown."""
    return 64.0 * s ** 3 * (1.0 - s) ** 3 if 0 <= s <= 1 else 0.0


class Trot:
    """Velocity-commanded trot. Call `targets(t, v, wz)` each control tick."""

    def __init__(self, gp: GaitParams | None = None):
        self.gp = gp or GaitParams()

    def foot_target(self, leg: str, phase: float, v_xy, wz: float, height: float | None = None):
        gp = self.gp
        h = gp.height if height is None else height
        p0 = legs.neutral_foot(leg, h)
        Ts = gp.duty * gp.period
        # foot velocity relative to body during stance
        vrel = -(np.array([v_xy[0], v_xy[1], 0.0]) + np.cross([0, 0, wz], [p0[0], p0[1], 0.0]))
        d = vrel * Ts                              # stance displacement (mm)
        ph = (phase + PHASE_OFFSET[leg]) % 1.0
        if ph < gp.duty:                           # stance: p0 - d/2 -> p0 + d/2
            s = ph / gp.duty
            p = p0 + d * (s - 0.5)
            contact = True
        else:                                      # swing: p0 + d/2 -> p0 - d/2
            s = (ph - gp.duty) / (1.0 - gp.duty)
            p = p0 + d * (0.5 - _swing_xy(s))
            p = p + np.array([0, 0, gp.step_height * _swing_z(s)])
            contact = False
        return p, contact

    def targets(self, t: float, v_xy=(0.0, 0.0), wz: float = 0.0, height=None, amp: float = 1.0):
        """Return {leg: (body-frame foot target, in_contact)}; amp ramps the stride/lift 0..1."""
        phase = (t / self.gp.period) % 1.0
        out = {}
        for leg in P.LEGS:
            p, c = self.foot_target(leg, phase, np.asarray(v_xy) * amp, wz * amp, height)
            if amp < 1.0 and not c:
                h0 = legs.neutral_foot(leg, self.gp.height if height is None else height)[2]
                p[2] = h0 + (p[2] - h0) * amp
            out[leg] = (p, c)
        return out


# ---------------------------------------------------------------------------
# Statically-stable crawl (wave) gait with CoG sway — AquaBot's default gait.
# QA iteration 4 showed that an open-loop trot saturates the DS3218 hip-pitch
# servos (diagonal support + 0.005 kg·m² reflected rotor inertia), so the
# default locomotion keeps three feet on the ground at all times.
# ---------------------------------------------------------------------------
CRAWL_ORDER = ("RL", "FL", "RR", "FR")


@dataclass
class CrawlParams:
    period: float = field(default_factory=lambda: P.CRAWL["period"])      # s per cycle (4 swings)
    swing: float = field(default_factory=lambda: P.CRAWL["swing"])        # fraction of cycle a leg is airborne
    step_height: float = field(default_factory=lambda: P.CRAWL["step_height"])   # mm
    margin: float = field(default_factory=lambda: P.CRAWL["margin"])      # mm CoG inside support at lift-off
    height: float = P.STAND_HEIGHT


class Crawl:
    def __init__(self, cp: CrawlParams | None = None, cog_xy=(12.0, 0.0)):
        self.cp = cp or CrawlParams()
        self.cog = np.asarray(cog_xy, float)

    def _lift_phase(self, leg):
        return CRAWL_ORDER.index(leg) / 4.0

    def _planned_foot(self, leg, phase, height, v_xy, wz):
        """Foot position (body frame, no sway) the gait plans for `leg` at `phase`."""
        cp = self.cp
        p0 = legs.neutral_foot(leg, height)
        vrel = -(np.array([v_xy[0], v_xy[1], 0.0]) + np.cross([0, 0, wz], [p0[0], p0[1], 0.0]))
        d = vrel * cp.period * (1.0 - cp.swing)
        ph = (phase - self._lift_phase(leg)) % 1.0
        if ph < cp.swing:
            return p0 + d * (0.5 - _swing_xy(ph / cp.swing))
        return p0 + d * ((ph - cp.swing) / (1.0 - cp.swing) - 0.5)

    def _shift_for(self, swing_leg, height, v_xy=(0.0, 0.0), wz=0.0):
        """Body shift (x, y) that keeps the CoG `margin` inside the support triangle
        for the whole swing of `swing_leg` (uses the planned, moving stance feet)."""
        cp = self.cp
        ph0 = self._lift_phase(swing_leg)
        diag = ("FL", "RR") if swing_leg in ("RL", "FR") else ("FR", "RL")
        need = []
        for u in np.linspace(0.0, 1.0, 5):
            ph = ph0 + u * cp.swing
            a = self._planned_foot(diag[0], ph, height, v_xy, wz)[:2]
            b = self._planned_foot(diag[1], ph, height, v_xy, wz)[:2]
            sw = legs.neutral_foot(swing_leg, height)[:2]
            t = (b - a) / np.linalg.norm(b - a)
            n = np.array([-t[1], t[0]])
            if np.dot(sw - a, n) > 0:
                n = -n
            dist = np.dot(self.cog - a, n)
            need.append((self.cp.margin - dist, n))
        k = int(np.argmax([x for x, _ in need]))
        return need[k][1] * need[k][0]

    def body_shift(self, phase, height, v_xy=(0.0, 0.0), wz=0.0):
        cp = self.cp
        k = int(np.floor(phase * 4.0)) % 4             # current slot
        s_cur = self._shift_for(CRAWL_ORDER[k], height, v_xy, wz)
        s_next = self._shift_for(CRAWL_ORDER[(k + 1) % 4], height, v_xy, wz)
        u = phase * 4.0 - np.floor(phase * 4.0)        # 0..1 within the slot
        sw = cp.swing * 4.0                            # swing share of a slot
        if u < sw:                                     # current leg in the air: hold
            return s_cur
        a = (u - sw) / (1.0 - sw)                      # shift towards next leg's support
        a = 10 * a ** 3 - 15 * a ** 4 + 6 * a ** 5
        return s_cur + (s_next - s_cur) * a

    def targets(self, t, v_xy=(0.0, 0.0), wz=0.0, height=None, amp=1.0):
        cp = self.cp
        h = cp.height if height is None else height
        phase = (t / cp.period) % 1.0
        shift = self.body_shift(phase, h, np.asarray(v_xy) * amp, wz * amp)
        out = {}
        for leg in P.LEGS:
            p0 = legs.neutral_foot(leg, h)
            vrel = -(np.array([v_xy[0], v_xy[1], 0.0]) * amp + np.cross([0, 0, wz * amp], [p0[0], p0[1], 0.0]))
            d = vrel * cp.period * (1.0 - cp.swing)
            ph = (phase - self._lift_phase(leg)) % 1.0
            if ph < cp.swing:                          # swing: p0 + d/2 -> p0 - d/2
                s = ph / cp.swing
                p = p0 + d * (0.5 - _swing_xy(s))
                p = p + np.array([0, 0, cp.step_height * _swing_z(s)])
                c = False
            else:
                s = (ph - cp.swing) / (1.0 - cp.swing)
                p = p0 + d * (s - 0.5)
                c = True
            p = p - np.array([shift[0], shift[1], 0.0])
            out[leg] = (p, c)
        return out

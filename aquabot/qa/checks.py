"""QA checks. Each check returns a Result; checks are attached to product-tree nodes in run.py."""
from __future__ import annotations

import itertools
import json
import os
import subprocess
import tempfile
from dataclasses import dataclass, field, asdict
from functools import lru_cache
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from .. import params as P
from .. import robot as R

ROOT = Path(__file__).resolve().parents[2]
REQ = P.REQ


@dataclass
class Result:
    id: str
    title: str
    passed: bool
    value: Any = None
    limit: Any = None
    detail: str = ""
    req: str = ""
    fixer: Optional[str] = None

    def as_dict(self):
        d = asdict(self)
        for k in ("value", "limit"):
            v = d[k]
            if isinstance(v, (np.floating, np.integer)):
                d[k] = v.item()
        return d


def R_(id, title, passed, value=None, limit=None, detail="", req="", fixer=None):
    return Result(id, title, bool(passed), _clean(value), _clean(limit), detail, req, fixer)


def _clean(v):
    if isinstance(v, (np.floating,)):
        return round(float(v), 6)
    if isinstance(v, float):
        return round(v, 6)
    if isinstance(v, np.ndarray):
        return np.round(v, 4).tolist()
    return v


# ======================================================================= PARTS
def part_checks(key: str) -> List[Result]:
    from ..cad.catalog import all_parts
    from ..massprops import part_mass_props
    p = all_parts()[key]
    out = []
    shapes = p.shape.vals()
    valid = all(s.isValid() for s in shapes)
    out.append(R_(f"P-GEO-01:{key}", "valid single B-rep solid", valid and len(shapes) == 1, len(shapes), 1))
    vol = p.solid().Volume()
    # STEP file on disk is current and round-trips
    step = ROOT / "cad" / "step" / f"{key}.step"
    if step.exists():
        import cadquery as cq
        v2 = cq.importers.importStep(str(step)).val().Volume()
        err = abs(v2 - vol) / vol
        out.append(R_(f"P-GEO-02:{key}", "STEP round-trip volume error", err < 1e-3, err, 1e-3, f"{step.name}"))
    else:
        out.append(R_(f"P-GEO-02:{key}", "STEP round-trip volume error", False, None, 1e-3, "STEP missing"))
    if p.vendor:
        ok = bool(p.mass_g and p.mass_g > 0 and p.descr)
        out.append(R_(f"V-DAT-01:{key}", "vendor data (mass, description, source)", ok, p.mass_g, ">0"))
        return out
    stl = ROOT / "cad" / "stl" / f"{key}.stl"
    if stl.exists():
        import trimesh
        m = trimesh.load(stl, force="mesh")
        err = abs(m.volume - vol) / vol
        out.append(R_(f"P-GEO-03:{key}", "STL watertight + volume error", m.is_watertight and err < 0.015,
                      round(err, 5), 0.015, f"watertight={m.is_watertight}, faces={len(m.faces)}"))
    else:
        out.append(R_(f"P-GEO-03:{key}", "STL watertight + volume error", False, None, 0.015, "STL missing"))
    bb = p.solid().BoundingBox()
    dims = sorted([bb.xlen, bb.ylen, bb.zlen])
    bed = sorted(P.PRINT_BED)
    fits = all(d <= b for d, b in zip(dims, bed))
    out.append(R_(f"P-MFG-01:{key}", "fits print bed (some orientation)", fits, [round(d, 1) for d in dims], bed))
    t = wall_thickness(key)
    ok = t["p01"] >= 0.8 and t["frac_below_1p2"] <= 0.06
    out.append(R_(f"P-MFG-02:{key}", "wall thickness (ray-cast, p1 ≥ 0.8 mm, <6 % below 1.2 mm)", ok,
                  dict(p01=t["p01"], p05=t["p05"], median=t["median"], frac_below_1p2=t["frac_below_1p2"]),
                  dict(p01=0.8, frac_below_1p2=0.06)))
    out.append(R_(f"P-MFG-03:{key}", "material + print instructions present", bool(p.print_note and p.material in P.MATERIALS),
                  p.material))
    m, _, _ = part_mass_props(key)
    out.append(R_(f"P-MAS-01:{key}", "estimated mass in plausible range", 0.5 < m < 250, round(m, 1), "0.5–250 g"))
    return out


@lru_cache(maxsize=None)
def wall_thickness(key: str, n: int = 900) -> dict:
    import trimesh
    from ..qa.geometry import _mesh
    m = _mesh(key)
    rng = np.random.default_rng(7)
    pts, fi = trimesh.sample.sample_surface(m, n, seed=7)
    nrm = m.face_normals[fi]
    origins = pts - nrm * 0.02
    dirs = -nrm
    locs, ray_idx, _ = m.ray.intersects_location(origins, dirs, multiple_hits=False)
    d = np.full(n, np.inf)
    for loc, ri in zip(locs, ray_idx):
        d[ri] = min(d[ri], np.linalg.norm(loc - origins[ri]))
    d = d[np.isfinite(d)]
    if len(d) == 0:
        return dict(p01=0, p05=0, median=0, frac_below_1p2=1.0)
    return dict(p01=round(float(np.percentile(d, 1)), 3), p05=round(float(np.percentile(d, 5)), 3),
                median=round(float(np.median(d)), 3), frac_below_1p2=round(float(np.mean(d < 1.2)), 4))


# ============================================================== ASSEMBLY LEVEL
def nominal_q(arm=None):
    from ..kinematics import legs as LK
    from ..sim.runtime import STOW
    q = {}
    for leg in P.LEGS:
        qq = LK.ik(leg, LK.neutral_foot(leg))
        q[f"{leg}_hip_abd"], q[f"{leg}_hip_pitch"], q[f"{leg}_knee"] = qq
    a = STOW if arm is None else arm
    q.update(arm_yaw=a[0], arm_shoulder=a[1], arm_elbow=a[2], arm_wrist=a[3], grip_a=8.0)
    return q


@lru_cache(maxsize=1)
def full_interference():
    from .geometry import interference
    inst = [(n, k, M) for (n, k, M, l) in R.world_instances(nominal_q())]
    return interference(inst, thresh=REQ["interference_max_mm3"])


def interference_check(node_id: str, link_filter: Callable[[str], bool]) -> Result:
    links = {n: l for (n, k, M, l) in R.world_instances({})}
    hits = [h for h in full_interference() if link_filter(links[h["a"]]) or link_filter(links[h["b"]])]
    return R_(f"A-INT-01:{node_id}", "exact B-rep interference at nominal pose (OCC boolean)", not hits,
              len(hits), 0, "; ".join(f"{h['a']}×{h['b']}={h['vol']} mm³" for h in hits[:6]),
              req="interference_max_mm3")


@lru_cache(maxsize=1)
def _holes():
    from .geometry import hole_mating
    return hole_mating(R.world_instances(nominal_q()))


def fastener_check(node_id: str, link_filter) -> Result:
    links = {n: l for (n, k, M, l) in R.world_instances({})}
    um, matched = _holes()
    bad = [u for u in um if link_filter(links[u["inst"]])]
    return R_(f"A-FAS-01:{node_id}", "every fastener hole has a coaxial mating hole", not bad, len(bad), 0,
              f"{matched} holes matched overall" + ("; " + str(bad[:3]) if bad else ""))


def leg_sweep(leg: str) -> List[Result]:
    """(a) leg-internal links over the full joint-limit box; (b) leg vs body/other legs over the
    operating envelope (crawl targets, turns, crouch, body-pose IK demo)."""
    from .geometry import Sweeper
    from ..kinematics import legs as LK
    from ..kinematics.gait import Crawl
    out = []
    sw = Sweeper(lambda n, l: l.startswith(leg))
    lo, hi = LK.limits(leg)
    bad = set()
    grid = [np.linspace(lo[i], hi[i], n) for i, n in enumerate((4, 9, 9))]
    for q1, q2, q3 in itertools.product(*grid):
        sw.set_pose({f"{leg}_hip_abd": q1, f"{leg}_hip_pitch": q2, f"{leg}_knee": q3})
        for c in sw.collisions():
            bad.add((c, tuple(np.degrees([q1, q2, q3]).round(0))))
    out.append(R_(f"A-KIN-01:{leg}", "no self-collision inside joint limits (4×9×9 grid)", not bad, len(bad), 0,
                  str(sorted(bad)[:3]) if bad else "324 poses clear"))
    # operating envelope
    other = {"FL": "FR", "FR": "FL", "RL": "RR", "RR": "RL"}[leg]
    sw2 = Sweeper(lambda n, l: l == "base" or l.startswith(leg) or l.startswith(other))
    envelope = []
    c = Crawl()
    for v, wz in ((35, 0), (0, 0.15), (25, 0.08), (0, -0.15), (-25, 0)):
        for t in np.linspace(0, c.cp.period, 25, endpoint=False):
            envelope.append(c.targets(t, (v, 0), wz, P.STAND_HEIGHT, 1.0)[leg][0])
    for h in (P.WATER_STANCE["height"], P.STAND_HEIGHT):
        envelope.append(LK.neutral_foot(leg, h))

    hits, margin_min = set(), 1e9
    for pft in envelope:
        q = LK.ik(leg, pft, check_limits=False, clamp=True)
        lo_, hi_ = LK.limits(leg)
        margin_min = min(margin_min, np.degrees(min((q - lo_).min(), (hi_ - q).min())))
        sw2.set_pose({f"{leg}_hip_abd": q[0], f"{leg}_hip_pitch": q[1], f"{leg}_knee": q[2]})
        for cc in sw2.collisions():
            hits.add(cc)
    out.append(R_(f"A-KIN-02:{leg}", "operating envelope collision-free vs body & neighbour leg", not hits,
                  len(hits), 0, f"{len(envelope)} poses; " + (str(sorted(hits)[:3]) if hits else "clear")))
    out.append(R_(f"A-KIN-03:{leg}", "joint-limit margin over gait envelope ≥ 3°", margin_min >= 3.0,
                  round(margin_min, 2), 3.0))
    # pick-up crouch (re-stepped feet, body rolled & shifted towards the object)
    from ..kinematics.body import rpy_matrix
    PS = P.PICK_STANCE
    Rb = rpy_matrix(np.radians(PS["roll_deg"]), 0, 0)
    pw = LK.neutral_foot(leg, P.STAND_HEIGHT); pw[2] = P.FOOT_R
    pw[0] += PS["foot_dx_front"] if leg[0] == "F" else PS["foot_dx_rear"]
    pb = Rb.T @ (pw - np.array([0.0, PS["shift_y"], PS["height"]]))
    q = LK.ik(leg, pb, check_limits=False, clamp=True)
    lo_, hi_ = LK.limits(leg)
    mc = np.degrees(min((q - lo_).min(), (hi_ - q).min()))
    sw2.set_pose({f"{leg}_hip_abd": q[0], f"{leg}_hip_pitch": q[1], f"{leg}_knee": q[2]},
                 base_T=None)
    col = sw2.collisions()
    out.append(R_(f"A-KIN-04:{leg}", "pick crouch posture inside limits (≥ 1°) and collision-free", mc >= 1.0 and not col,
                  round(mc, 2), 1.0, str(col[:2]) if col else ""))
    return out


ADJ_ARM = {("arm_turntable", "arm_upper"), ("arm_upper", "arm_fore"), ("arm_fore", "arm_wrist"),
           ("arm_wrist", "jaw_a"), ("arm_wrist", "jaw_b"), ("jaw_a", "jaw_b"), ("arm_wrist", "pinion")}


def arm_sweep() -> List[Result]:
    from .geometry import Sweeper
    from ..kinematics import arm as AK
    out = []
    sw = Sweeper(lambda n, l: l.startswith(("arm_", "jaw_", "pinion")))
    lo, hi = AK.limits()
    bad = set()
    for qs, qe, qw, g in itertools.product(np.linspace(lo[1], hi[1], 7), np.linspace(lo[2], hi[2], 8),
                                           np.linspace(lo[3], hi[3], 7), (0.0, P.GRIP_STROKE)):
        sw.set_pose(dict(arm_shoulder=qs, arm_elbow=qe, arm_wrist=qw, grip_a=g))
        for a, b in sw.collisions():
            la, lb = sw.meta[a][1], sw.meta[b][1]
            if (la, lb) in ADJ_ARM or (lb, la) in ADJ_ARM:
                bad.add((a, b, tuple(np.degrees([qs, qe, qw]).round(0))))
    out.append(R_("A-KIN-01:arm", "adjacent arm links collision-free inside joint limits (7×8×7×2)", not bad,
                  len(bad), 0, str(sorted(bad)[:3]) if bad else "784 poses clear"))
    # used poses: stow, watering aims, pick waypoints vs everything
    sw2 = Sweeper(lambda n, l: True)
    poses = used_arm_poses()
    hits = set()
    for name, q in poses.items():
        sw2.set_pose(nominal_q(q))
        for cc in sw2.collisions():
            hits.add((name,) + cc)
    out.append(R_("A-KIN-02:arm", "mission arm poses collision-free vs whole robot", not hits, len(hits), 0,
                  f"{len(poses)} poses; " + (str(sorted(hits)[:3]) if hits else "clear")))
    return out


def used_arm_poses():
    from ..sim.runtime import STOW
    from ..kinematics import arm as AK
    poses = {"stow": STOW, "water_mid": np.radians([90.0, 60.0, -110.0, -40.0]),
             "pick_mid": np.radians([90.0, 55.0, -100.0, -40.0])}
    # watering aim (body frame, at the watering stance) for plants 235 mm to the left, 3 offsets
    WS = P.WATER_STANCE
    for dx in (-30.0, 0.0, 30.0):
        tgt = np.array([P.ARM_YAW_XY[0] + dx, 235.0, -WS["height"] + 82.0])
        tip, pitch = AK.aim_nozzle(tgt, np.array([P.ARM_YAW_XY[0], 0, 0]) - tgt, np.radians(WS["pitch_deg"]), WS["drop"])
        try:
            poses[f"water_aim_{dx:+.0f}"] = AK.ik(tip, pitch, tool=AK.NOZZLE, q_ref=np.radians([90, 40, -100, -20]))
        except AK.Unreachable:
            pass
    return poses


def gripper_checks() -> List[Result]:
    from ..cad import parts_arm as PA
    out = []
    m = P.GRIP_MODULE
    rp = PA.R_PITCH
    ok = abs(rp - m * P.GRIP_PINION_TEETH / 2) < 1e-9 and PA.GR["rack"][0] >= PA.GR["pinion"][0]
    stroke_deg = np.degrees(P.GRIP_STROKE / rp)
    out.append(R_("A-GRP-01:gripper", "rack/pinion geometry consistent; stroke within servo range", ok and stroke_deg <= P.MG90S.range_deg,
                  round(stroke_deg, 1), P.MG90S.range_deg, f"pitch radius {rp} mm, opening {2*P.GRIP_STROKE+PA.GR['closed_gap']:.0f} mm max"))
    F = P.MG90S.stall_torque / (rp / 1000.0)                   # N per jaw pair
    need = 2.0 * (P.PAYLOAD_G / 1000.0 * 9.81) / (2 * 0.6)      # SF 2 with μ=0.6
    out.append(R_("A-GRP-02:gripper", "grip force ≥ 2× payload slip force (μ=0.6)", F >= need, round(F, 2), round(need, 2),
                  f"MG90S stall {P.MG90S.stall_torque} N·m / r {rp} mm"))
    return out


# =============================================================== SYSTEM LEVEL
def mass_checks() -> List[Result]:
    from ..massprops import total_mass
    wet = total_mass(1.0)
    return [R_("S-MAS-01", "total mass with full tank", wet <= REQ["mass_max_g"], round(wet, 1), REQ["mass_max_g"],
               f"dry {total_mass(0.0):.0f} g", req="mass_max_g")]


def support_polygon(feet_xy):
    from scipy.spatial import ConvexHull
    pts = np.asarray(feet_xy)
    hull = ConvexHull(pts)
    return pts[hull.vertices]


def margin_in_polygon(p, poly):
    m = 1e9
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        t = (b - a) / np.linalg.norm(b - a)
        nrm = np.array([-t[1], t[0]])
        c = poly.mean(axis=0)
        if np.dot(c - a, nrm) < 0:
            nrm = -nrm
        m = min(m, np.dot(p - a, nrm))
    return m


def stability_checks() -> List[Result]:
    from ..massprops import cog
    from ..kinematics import legs as LK
    from ..kinematics.gait import Crawl
    out = []
    cases = {"stand, full tank, arm stowed": (None, 1.0, 0.0), "stand, empty tank": (None, 0.0, 0.0)}
    for k, q in used_arm_poses().items():
        cases[f"arm {k}, full tank, 100 g payload"] = (q, 1.0, P.PAYLOAD_G)
    worst = 1e9; detail = []
    for name, (aq, fill, pay) in cases.items():
        q = nominal_q(aq)
        m, c = cog(q, fill, pay)
        feet = [R.foot_point(l, q)[:2] for l in P.LEGS]
        mg = margin_in_polygon(c[:2], support_polygon(feet))
        worst = min(worst, mg)
        detail.append(f"{name}: {mg:.0f} mm")
    out.append(R_("S-STB-01", "static stability margin, 4-foot stance (all mission configs)", worst >= REQ["stability_margin_min"],
                  round(worst, 1), REQ["stability_margin_min"], "; ".join(detail), req="stability_margin_min"))
    # crawl gait: planned CoG margin at every instant with 3 feet down
    c = Crawl()
    m, cg = cog(nominal_q(), 1.0)
    c.cog = cg[:2]
    worst = 1e9
    for v, wz in ((P.CRAWL["v_cmd"] * 1000, 0.0), (0.0, 0.12), (25.0, 0.08)):
        for t in np.linspace(0, c.cp.period, 240, endpoint=False):
            tg = c.targets(t, (v, 0), wz, P.STAND_HEIGHT, 1.0)
            st = [p[:2] for (p, cc) in tg.values() if cc]
            if len(st) < 3:
                worst = -1e9; continue
            worst = min(worst, margin_in_polygon(cg[:2], support_polygon(st)))
    out.append(R_("S-STB-02", "crawl gait: planned CoG margin inside support (3+ feet) ≥ 12 mm", worst >= 12.0,
                  round(worst, 1), 12.0, "sampled 720 instants incl. turning", fixer="crawl_margin"))
    return out


def torque_checks() -> List[Result]:
    """Static torque safety factor: 3-leg support with the CoG at the planned crawl margin, and the
    arm at full horizontal reach with payload."""
    from ..massprops import cog, total_mass
    from ..kinematics import legs as LK, arm as AK
    out = []
    W = total_mass(1.0) / 1000.0 * 9.81
    q = nominal_q()
    m, cg = cog(q, 1.0)
    worst, wj = 0.0, ""
    for lift in P.LEGS:
        st = [l for l in P.LEGS if l != lift]
        F = [R.foot_point(l, q) for l in st]
        A = np.array([[1, 1, 1], [F[0][0], F[1][0], F[2][0]], [F[0][1], F[1][1], F[2][1]]])
        # CoG placed `margin` inside the triangle (planned) — use the shift the crawl applies
        from ..kinematics.gait import Crawl
        cr = Crawl(); cr.cog = cg[:2]
        s = cr._shift_for(lift, P.STAND_HEIGHT, (P.CRAWL["v_cmd"] * 1000, 0), 0.0)
        cxy = cg[:2] + s
        fz = np.linalg.solve(A, np.array([W, W * cxy[0], W * cxy[1]]))
        for l, f in zip(st, fz):
            qq = np.array([q[f"{l}_hip_abd"], q[f"{l}_hip_pitch"], q[f"{l}_knee"]])
            tau = np.abs(LK.static_torques(l, qq, [0, 0, f]))
            if tau.max() > worst:
                worst, wj = tau.max(), f"{l} lifted={lift} τ={np.round(tau,3).tolist()} N·m (Fz={f:.1f} N)"
    sf = P.DS3218.stall_torque / worst
    out.append(R_("S-TRQ-01", "leg static torque safety factor (3-foot crawl support)", sf >= REQ["torque_sf_static_min"],
                  round(sf, 2), REQ["torque_sf_static_min"], wj, req="torque_sf_static_min"))
    # arm: horizontal full reach with payload
    from ..massprops import link_inertials
    li = link_inertials()
    g = 9.81
    qa = np.array([0.0, 0.0, 0.0, 0.0])
    lw = R.link_world(nominal_q(qa))
    def moment_about(joint_link, links, pay=True):
        piv = lw[joint_link][:3, 3]
        M = 0.0
        for ln in links:
            mm, c, _ = li[ln]
            cw = (lw[ln] @ np.array([*c, 1.0]))[:3]
            M += mm / 1000 * g * (cw[0] - piv[0]) / 1000
        if pay:
            M += P.PAYLOAD_G / 1000 * g * (R.tcp_point(nominal_q(qa))[0] - piv[0]) / 1000
        return abs(M)
    dist = ["arm_upper", "arm_fore", "arm_wrist", "jaw_a", "jaw_b", "pinion"]
    tau = {"shoulder": moment_about("arm_upper", dist), "elbow": moment_about("arm_fore", dist[1:]),
           "wrist": moment_about("arm_wrist", dist[2:])}
    stall = {"shoulder": P.DS3218.stall_torque, "elbow": P.DS3218.stall_torque, "wrist": P.MG90S.stall_torque}
    sfs = {k: stall[k] / v for k, v in tau.items()}
    k_min = min(sfs, key=sfs.get)
    out.append(R_("S-TRQ-02", "arm static torque SF at full horizontal reach + 100 g payload", sfs[k_min] >= REQ["torque_sf_static_min"],
                  round(sfs[k_min], 2), REQ["torque_sf_static_min"],
                  ", ".join(f"{k}: {tau[k]:.3f} N·m (SF {sfs[k]:.1f})" for k in tau), req="torque_sf_static_min"))
    return out


def ik_checks() -> List[Result]:
    from ..kinematics import legs as LK, arm as AK
    rng = np.random.default_rng(42)
    out = []
    e_rt, e_tree = 0.0, 0.0
    for leg in P.LEGS:
        lo, hi = LK.limits(leg)
        for _ in range(2500):
            q = rng.uniform(lo, hi)
            p = LK.fk(leg, q)
            e_rt = max(e_rt, np.linalg.norm(LK.fk(leg, LK.ik(leg, p, check_limits=False)) - p))
        for _ in range(60):
            q = rng.uniform(lo, hi)
            qd = {f"{leg}_hip_abd": q[0], f"{leg}_hip_pitch": q[1], f"{leg}_knee": q[2]}
            e_tree = max(e_tree, np.linalg.norm(R.foot_point(leg, qd) - LK.fk(leg, q)))
    out.append(R_("S-KIN-01", "leg IK round-trip error (10 000 random poses)", e_rt <= REQ["ik_err_max"], e_rt, REQ["ik_err_max"],
                  req="ik_err_max"))
    out.append(R_("S-KIN-02", "analytic leg FK == CAD link-tree FK", e_tree <= 1e-6, e_tree, 1e-6))
    lo, hi = AK.limits()
    e_rt, e_tree, fails = 0.0, 0.0, 0
    for tool in (AK.TCP, AK.NOZZLE):
        for _ in range(3000):
            q = rng.uniform(lo, hi)
            p, _ = AK.fk(q, tool)
            try:
                q2 = AK.ik(p, AK.pitch_outward(q, tool), tool=tool, check_limits=False)
                e_rt = max(e_rt, np.linalg.norm(AK.fk(q2, tool)[0] - p))
            except AK.Unreachable:
                fails += 1
    for _ in range(100):
        q = rng.uniform(lo, hi)
        qd = dict(arm_yaw=q[0], arm_shoulder=q[1], arm_elbow=q[2], arm_wrist=q[3])
        e_tree = max(e_tree, np.linalg.norm(R.tcp_point(qd) - AK.fk(q)[0]),
                     np.linalg.norm(R.nozzle_tip(qd)[0] - AK.fk(q, AK.NOZZLE)[0]))
    out.append(R_("S-KIN-03", "arm IK round-trip error (6 000 poses, TCP + nozzle tools)", e_rt <= REQ["ik_err_max"] and fails == 0,
                  e_rt, REQ["ik_err_max"], f"unsolved={fails}", req="ik_err_max"))
    out.append(R_("S-KIN-04", "analytic arm FK == CAD link-tree FK (TCP and nozzle)", e_tree <= 1e-6, e_tree, 1e-6))
    return out


def firmware_checks() -> List[Result]:
    from .. import firmware_config
    from ..kinematics import legs as LK, arm as AK
    from ..kinematics.gait import Crawl
    out = []
    cfg = (ROOT / "firmware" / "aquabot" / "config.h")
    fresh = cfg.exists() and cfg.read_text() == firmware_config.generate()
    out.append(R_("S-FW-01", "firmware config.h matches params.py (regenerated)", fresh, fresh, True))
    tmp = Path(tempfile.mkdtemp())
    fw = ROOT / "firmware"
    b1 = subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-DHOST_BUILD", f"-I{fw/'test'/'stubs'}",
                         f"-I{fw/'aquabot'}", str(fw / "test" / "host_build.cpp"), str(fw / "aquabot" / "kinematics.cpp"),
                         str(fw / "aquabot" / "gait.cpp"), str(fw / "aquabot" / "hal.cpp"), "-o", str(tmp / "fw")],
                        capture_output=True, text=True)
    ok = b1.returncode == 0 and subprocess.run([str(tmp / "fw")], capture_output=True).returncode == 0
    out.append(R_("S-FW-02", "full sketch compiles on host with -Wall -Wextra -Werror and runs 300 ticks", ok, b1.returncode, 0,
                  b1.stderr[:300]))
    b2 = subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Werror", f"-I{fw/'test'/'stubs'}", str(fw / "test" / "host_ik.cpp"),
                         str(fw / "aquabot" / "kinematics.cpp"), str(fw / "aquabot" / "gait.cpp"), "-o", str(tmp / "ik")],
                        capture_output=True, text=True)
    if b2.returncode != 0:
        out.append(R_("S-FW-03", "firmware IK/gait parity with Python", False, None, REQ["fw_parity_max"], b2.stderr[:300]))
        return out
    rng = np.random.default_rng(3)
    lines, expect = [], []
    for i in range(400):
        leg = P.LEGS[i % 4]
        lo, hi = LK.limits(leg)
        q = rng.uniform(lo, hi)
        p = LK.fk(leg, q)
        lines.append(f"L {i%4} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}")
        expect.append(("L", np.degrees(LK.ik(leg, p, check_limits=False))))
    lo, hi = AK.limits()
    n = 0
    while n < 300:
        q = rng.uniform(lo, hi)
        p, ph = AK.fk(q, AK.NOZZLE)
        if AK.pitch_outward(q, AK.NOZZLE) != ph or q[2] > 0:
            continue
        lines.append(f"A {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {np.degrees(ph):.8f} {AK.NOZZLE[0]} {AK.NOZZLE[1]}")
        expect.append(("A", np.degrees(q)))
        n += 1
    cr = Crawl()
    for t in np.linspace(0, cr.cp.period, 60, endpoint=False):
        lines.append(f"G {t:.6f} 35 0.05 {P.STAND_HEIGHT}")
        tg = cr.targets(t, (35, 0), 0.05, P.STAND_HEIGHT, 1.0)
        expect.append(("G", np.concatenate([np.r_[tg[l][0], float(tg[l][1])] for l in P.LEGS])))
    res = subprocess.run([str(tmp / "ik")], input="\n".join(lines) + "\n", capture_output=True, text=True)
    rows = [np.array([float(x) for x in r.split()]) for r in res.stdout.strip().splitlines()]
    e_ang, e_g = 0.0, 0.0
    for (kind, ex), row in zip(expect, rows):
        if kind == "L":
            e_ang = max(e_ang, np.abs(((row[:3] - ex) + 180) % 360 - 180).max())
        elif kind == "A":
            e_ang = max(e_ang, np.abs(((row[:4] - ex) + 180) % 360 - 180).max())
        else:
            pos = [i for i in range(16) if i % 4 != 3]
            e_g = max(e_g, np.abs(row[pos] - ex[pos]).max())
    ok = len(rows) == len(expect) and e_ang <= REQ["fw_parity_max"] * 30 and e_g < 0.05
    out.append(R_("S-FW-03", "firmware (float32 C++) IK & crawl planner parity with Python", ok,
                  dict(max_angle_err_deg=round(e_ang, 6), max_gait_err_mm=round(e_g, 5)),
                  dict(max_angle_err_deg=REQ["fw_parity_max"] * 30, max_gait_err_mm=0.05),
                  f"{len(rows)} vectors (float32 vs float64)", req="fw_parity_max"))
    return out


def water_checks() -> List[Result]:
    import cadquery as cq
    from ..cad import parts_body as PB
    from ..kinematics import arm as AK
    T = P.TANK
    out = []
    inner = cq.Workplane("XY").box(T["x1"] - T["x0"] - 2 * T["wall"], 2 * (T["y"] - T["wall"]), T["z1"] - T["z0"] - T["floor"]) \
        .translate(((T["x0"] + T["x1"]) / 2, 0, (T["z1"] + T["z0"] + T["floor"]) / 2))
    tank = PB.tank_body().shape
    cavity = inner.cut(tank)
    vol_ml = cavity.val().Volume() / 1000.0
    out.append(R_("S-WAT-01", "usable tank volume (CAD cavity minus bosses & baffle)", vol_ml >= REQ["tank_ml_min"],
                  round(vol_ml, 1), REQ["tank_ml_min"], req="tank_ml_min"))
    v0 = AK.water_jet_speed()
    out.append(R_("S-WAT-02", "jet exit speed sensible for ballistic aiming (0.5–3 m/s)", 0.5 <= v0 <= 3.0, round(v0, 3), "0.5–3",
                  f"{P.PUMP['flow_ml_min']} ml/min through Ø{P.NOZZLE_ORIFICE} mm"))
    return out


def power_checks(sim_metrics: dict | None) -> List[Result]:
    """Runtime from the simulated servo supply current (I ∝ τ per servo, incl. holding load)."""
    out = []
    E_Wh = P.BATTERY["v_nom"] * P.BATTERY["mah"] / 1000.0 * 0.8        # 80 % usable
    walk = (sim_metrics or {}).get("walk", {})
    I_servo = walk.get("mean_servo_current_A", 6.0)                    # conservative default without sim
    servo_W = I_servo * 6.0 / P.UBEC["eff"]                            # 6 V rail through the UBECs
    pump_W = P.PUMP["voltage"] * P.PUMP["current"] * 0.25              # 25 % duty during missions
    P_total = servo_W + P.ELECTRONICS_W / 0.85 + pump_W
    runtime = E_Wh / P_total * 60.0
    out.append(R_("S-PWR-01", "estimated mission runtime on one charge (walking continuously)", runtime >= REQ["runtime_min"],
                  round(runtime, 1), REQ["runtime_min"],
                  f"servo rail {I_servo:.2f} A mean (sim, I∝τ) -> P≈{P_total:.1f} W from {E_Wh:.1f} Wh usable",
                  req="runtime_min"))
    peak = walk.get("peak_servo_current_A", 12 * P.DS3218.stall_current * 0.35)
    out.append(R_("S-PWR-02", "servo rail peak current within 2 × 10 A UBEC rating", peak <= 20.0, round(peak, 1), 20.0,
                  "simulated peak (sum of all servos)" if walk else "estimate: 35 % of legs at stall"))
    return out


# --------------------------------------------------------------- simulation
SIM_CACHE: Dict[str, dict] = {}


def sim_metrics(name: str, fresh=False) -> dict:
    from ..sim import scenarios as S
    if fresh or name not in SIM_CACHE:
        SIM_CACHE[name] = {"walk": S.walk, "watering": S.watering, "pick": S.pick_place, "ik": S.ik_showcase}[name](video=False)
    return SIM_CACHE[name]


def sim_checks_walk(w: dict) -> List[Result]:
    out = []
    out.append(R_("S-SIM-01", "crawl speed with full tank", w["speed_m_s"] >= REQ["walk_speed_min"], round(w["speed_m_s"], 4),
                  REQ["walk_speed_min"], req="walk_speed_min"))
    out.append(R_("S-SIM-02", "body tilt during walk + turn", w["max_tilt_deg"] <= REQ["tilt_max_deg"], round(w["max_tilt_deg"], 2),
                  REQ["tilt_max_deg"], req="tilt_max_deg", fixer="crawl_slower"))
    out.append(R_("S-SIM-03", "servo torque, 50 ms filtered peak (fraction of stall)", w["peak_torque_frac_50ms"] <= REQ["sim_peak_torque_frac"],
                  round(w["peak_torque_frac_50ms"], 3), REQ["sim_peak_torque_frac"], f"joint {w['peak_joint_50ms']}; raw peak {w['peak_torque_frac']:.2f}",
                  req="sim_peak_torque_frac", fixer="crawl_slower"))
    out.append(R_("S-SIM-04", "servo torque, 99th percentile (fraction of stall)", w["p99_torque_frac"] <= REQ["sim_p99_torque_frac"],
                  round(w["p99_torque_frac"], 3), REQ["sim_p99_torque_frac"], req="sim_p99_torque_frac", fixer="crawl_slower"))
    out.append(R_("S-SIM-05", "turn in place achieves ≥ 70 % of commanded yaw, no IK clamping", w["turn_deg"] >= 0.7 * np.degrees(0.12 * 7.2) * 0.8 and w["ik_clamps"] == 0,
                  round(w["turn_deg"], 1), round(0.7 * np.degrees(0.12 * 7.2) * 0.8, 1), f"IK clamps {w['ik_clamps']}"))
    return out


def robustness_checks() -> List[Result]:
    """Gait must not sit next to a stability cliff: perturb the cycle time ±10 % for both
    straight walking and turning in place; every run must stay upright without joint clamping."""
    from ..sim.runtime import AquaSim
    runs = []
    base, turn = P.CRAWL["period"], P.CRAWL["turn_period"]
    for kind, per in (("walk", base * 0.9), ("walk", base * 1.1), ("turn", turn * 0.9), ("turn", turn * 1.1)):
        saved = dict(P.CRAWL)
        P.CRAWL["period" if kind == "walk" else "turn_period"] = per
        try:
            s = AquaSim(drops=False)
            s.init_pose(); s.stand(0.8)
            if kind == "walk":
                s.crawl_walk(4 * per, v=(P.CRAWL["v_cmd"], 0.0))
            else:
                s.crawl_walk(3 * per, v=(0.0, 0.0), wz=0.12)
            m = s.metrics(0.8)
            clamp_ms = int(getattr(s, "ik_clamps", 0)) * s.dt * 1000      # upper bound (per-leg events)
            runs.append((kind, round(per, 2), round(m["max_tilt_deg"], 1), round(clamp_ms)))
        finally:
            P.CRAWL.clear(); P.CRAWL.update(saved)
    ok = all(t <= REQ["tilt_max_deg"] and c <= 50 for (_, _, t, c) in runs)
    return [R_("S-SIM-11", "gait robustness: cycle time ±10 % (walk & turn) upright, joint-limit contact ≤ 50 ms", ok,
               max(r[2] for r in runs), REQ["tilt_max_deg"],
               "; ".join(f"{k} T={p}s tilt {t}° limit-contact {c} ms" for k, p, t, c in runs))]


def sim_checks() -> List[Result]:
    out = sim_checks_walk(sim_metrics("walk"))
    wt = sim_metrics("watering")
    out.append(R_("S-SIM-06", "watering: fraction of emitted water landing in target pots", wt["delivery_fraction"] >= REQ["water_delivery_min"],
                  round(wt["delivery_fraction"], 4), REQ["water_delivery_min"], f"per plant {np.round(wt['per_plant_ml'],2).tolist()} ml",
                  req="water_delivery_min"))
    ok = abs(wt["water_balance_err_ml"]) < 0.01 and min(wt["per_plant_ml"]) >= 9.0 and not wt["aim_failures"] and wt["ik_clamps"] == 0
    out.append(R_("S-SIM-07", "watering: mass balance exact, every plant dosed ≥ 90 %, no aim failures", ok,
                  round(wt["water_balance_err_ml"], 5), 0.01, f"tank left {wt['tank_left_ml']:.1f} ml"))
    pk = sim_metrics("pick")
    ok = (pk["lift_mm"] >= REQ["grasp_lift_min"] and pk["place_err_mm"] <= REQ["place_err_max"] and pk["in_tray"]
          and pk["unreachable_waypoints"] == 0 and pk.get("ik_clamps", 0) == 0)
    out.append(R_("S-SIM-08", "pick & place: lift ≥ 30 mm, placed ≤ 25 mm, no unreachable waypoints, no joint clamping", ok,
                  dict(lift_mm=round(pk["lift_mm"], 1), place_err_mm=round(pk["place_err_mm"], 1), unreachable=pk["unreachable_waypoints"],
                       ik_clamps=pk.get("ik_clamps", 0)),
                  dict(lift_mm=REQ["grasp_lift_min"], place_err_mm=REQ["place_err_max"]), req="grasp_lift_min"))
    out.append(R_("S-SIM-09", "pick & place: servo 50 ms peak ≤ limit, tilt ≤ limit",
                  pk["peak_torque_frac_50ms"] <= REQ["sim_peak_torque_frac"] and pk["max_tilt_deg"] <= REQ["tilt_max_deg"],
                  dict(peak=round(pk["peak_torque_frac_50ms"], 3), tilt=round(pk["max_tilt_deg"], 2)),
                  dict(peak=REQ["sim_peak_torque_frac"], tilt=REQ["tilt_max_deg"])))
    ik = sim_metrics("ik")
    out.append(R_("S-SIM-10", "IK showcase: arm Cartesian tracking error (analytic)", ik["arm_ik_max_err_mm"] <= REQ["ik_err_max"] * 10,
                  ik["arm_ik_max_err_mm"], REQ["ik_err_max"] * 10))
    return out


def model_checks() -> List[Result]:
    import mujoco
    from ..sim import mjcf
    from .. import urdf
    from ..massprops import total_mass
    out = []
    path = ROOT / "sim" / "models" / "aquabot.xml"
    try:
        m = mujoco.MjModel.from_xml_path(str(path))
        mass = float(sum(m.body_mass[1:]))
        ok = abs(mass * 1000 - total_mass(1.0)) / total_mass(1.0) < 0.005
        out.append(R_("S-MOD-01", "MJCF compiles; mass == CAD mass (±0.5 %)", ok, round(mass * 1000, 1), round(total_mass(1.0), 1),
                      f"nq={m.nq} nu={m.nu} nbody={m.nbody}"))
    except Exception as ex:
        out.append(R_("S-MOD-01", "MJCF compiles; mass == CAD mass", False, None, None, str(ex)[:200]))
    up = ROOT / "sim" / "models" / "urdf" / "aquabot.urdf"
    rng = np.random.default_rng(5)
    err = 0.0
    names = R.joint_names()
    tree = R.build_tree()
    for _ in range(50):
        q = {}
        for l in tree.values():
            if l.joint and not l.joint.mimic:
                q[l.joint.name] = rng.uniform(l.joint.lower, l.joint.upper)
        lw = R.link_world(q)
        qu = {k: (v / 1000.0 if k.startswith("grip_") else v) for k, v in q.items()}
        uw = urdf.fk_from_urdf(up, qu)
        for k in lw:
            err = max(err, np.linalg.norm(lw[k][:3, 3] / 1000.0 - uw[k][:3, 3]) * 1000,
                      np.abs(lw[k][:3, :3] - uw[k][:3, :3]).max() * 100)
    out.append(R_("S-MOD-02", "URDF forward kinematics == link tree (50 random poses)", err < 1e-3, err, 1e-3,
                  f"{len(names)} actuated joints"))
    return out


def bom_checks() -> List[Result]:
    from ..bom import build_rows, instance_counts
    from ..cad.catalog import all_parts
    rows = build_rows()
    cnt = instance_counts()
    out = []
    in_bom = {r["item"]: r["qty"] for r in rows if r["category"] in ("Printed", "Purchased")}
    missing = [k for k in cnt if in_bom.get(k) != cnt[k]]
    out.append(R_("S-BOM-01", "every part instance appears in the BOM with the right quantity", not missing, len(missing), 0, str(missing[:5])))
    files = []
    for r in rows:
        if r["category"] == "Printed":
            for f in r["files"].split("; "):
                if not (ROOT / f).exists():
                    files.append(f)
        if r["category"] == "Purchased":
            f = r["files"].split(" ")[0]
            if not (ROOT / f).exists():
                files.append(f)
    out.append(R_("S-BOM-02", "every BOM part has its STEP (and STL for printed parts)", not files, len(files), 0, str(files[:5])))
    orphans = [p.stem for p in (ROOT / "cad" / "stl").glob("*.stl") if p.stem not in all_parts()]
    orphans += [p.stem for p in (ROOT / "cad" / "step").glob("*.step") if p.stem not in all_parts()]
    out.append(R_("S-BOM-03", "no orphan CAD files", not orphans, len(orphans), 0, str(orphans[:5])))
    md = (ROOT / "bom" / "BOM.md")
    out.append(R_("S-BOM-04", "BOM.md / BOM.csv generated", md.exists() and (ROOT / "bom" / "BOM.csv").exists(), md.exists(), True))
    return out


def doc_checks() -> List[Result]:
    import re
    out = []
    bad = []
    for md in [ROOT / "README.md"] + list((ROOT / "docs").glob("*.md")):
        if not md.exists():
            bad.append(str(md.name)); continue
        for link in re.findall(r"\]\(([^)#\s]+)", md.read_text()):
            if link.startswith(("http", "mailto")):
                continue
            if not (md.parent / link).resolve().exists():
                bad.append(f"{md.name}->{link}")
    out.append(R_("S-DOC-01", "all relative links in README/docs resolve", not bad, len(bad), 0, str(bad[:6])))
    vids = sorted(p.name for p in (ROOT / "sim" / "videos").glob("*.mp4"))
    out.append(R_("S-DOC-02", "simulation videos present (5)", len(vids) >= 5, len(vids), 5, ", ".join(vids)))
    return out

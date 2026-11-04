"""Arm, wrist and gripper parts."""
from __future__ import annotations

import numpy as np
import cadquery as cq

from .. import params as P
from .geom import Hole, Part, box, bolt_circle, cut_all, cyl, cyl_x, cyl_y, cyl_z, tube, union_all, \
    slot_xz_profile_y
from .parts_leg import fork_top, ds_housing, link_column
from . import parts_body as PB

S = P.DS3218
M = P.MG90S
F = P.FORK
B = P.IDLER_BEARING
INS = P.INSERT_M3["d_hole"]
CLR3 = 3.4

# Turntable link origin (body frame): yaw axis at horn top
TT_ORIGIN = (PB.YAW["x"], PB.YAW["y"], PB.YAW_HORN_TOP)
SHOULDER = (PB.YAW["x"], 0.0, P.ARM_SHOULDER_Z)

# MG90S wrist placement inside the forearm (link frame)
MG_CASE_TOP_Y = F["horn_top"] - M.horn_top      # 16.75: wrist horn top flush with DS3218 horns


def _shift(part: Part, o) -> Part:
    ox, oy, oz = o
    part.shape = part.shape.translate((-ox, -oy, -oz))
    for h in part.holes:
        h.p = (h.p[0] - ox, h.p[1] - oy, h.p[2] - oz)
    return part


# --------------------------------------------------------------- turntable
def arm_turntable() -> Part:
    x0, y0 = PB.YAW["x"], PB.YAW["y"]
    zh = PB.YAW_HORN_TOP
    zp0, zp1 = PB.BEAR_Z1, PB.BEAR_Z1 + 5.0           # plate 94..99
    xs, _, zs = SHOULDER
    hub = cyl_z(12.5, x0, y0, zh, zp0)
    plate = union_all([cyl_z(32.0, x0, y0, zp0, zp1), box(40.0, 70.0, -10.0, 38.0, zp0, zp1),
                       box(16.0, 50.0, -8.0, 6.0, zp0, zp1)])
    # QA iter 7: trim the front so the upper-arm crossbar clears when reaching down
    plate = plate.intersect(box(x0 - 60.0, x0 + 25.0, -60.0, 60.0, zp0 - 1, zp1 + 1))
    skirt = tube(24.0, P.YAW_BEARING["D"] / 2, (x0, y0, PB.BEAR_Z0), (x0, y0, zp0))
    # shoulder servo ear plate (y range from the fork layout, arm Y' = -y)
    ey0, ey1 = -F["ear_plate"][1], -F["ear_plate"][0]          # -3.75 .. 1.25
    c = 0.3
    # shoulder servo: X_s -> -x (bulk back), Y_s -> -z
    wx0, wx1 = xs - (S.case_l - S.shaft_off) - c, xs + S.shaft_off + c
    wz0, wz1 = zs - S.case_w / 2 - c, zs + S.case_w / 2 + c
    fx0 = xs - (S.case_center_x + S.ear_span / 2) - 1.75     # 16
    fx1 = xs - (S.case_center_x - S.ear_span / 2) + 1.75     # 74
    ear = slot_xz_profile_y([(fx0, zp1), (xs - 10.0, zp1), (xs - 10.0, zs - 14.0), (fx1, zs - 14.0),
                             (fx1, zs + 14.0), (fx0, zs + 14.0)], ey0, ey1)
    ear = ear.cut(box(wx0, wx1, ey0 - 1, ey1 + 1, wz0, wz1))
    # idler upright (arm inner side = +y)
    iy0, iy1 = -F["idler"][1], -F["idler"][0]                  # 31.75 .. 37.75
    idler = box(xs - 12.0, xs + 12.0, iy0, iy1, zp1, zs).union(cyl_y(12.0, xs, zs, iy0, iy1))
    body = union_all([hub, plate, skirt, ear, idler])
    cuts, holes = [], []
    cuts.append(tube(18.5, 12.5, (x0, y0, zp0 - 0.01), (x0, y0, zp0 + 0.5)))   # rubbing relief
    for k, (dx, dy) in enumerate(bolt_circle(S.horn_pcd, 4, 45.0)):
        cuts.append(cyl_z(CLR3 / 2, x0 + dx, y0 + dy, zh - 1, zp1 + 1))
        cuts.append(cyl_z(3.25, x0 + dx, y0 + dy, zp1 - 3.5, zp1 + 1))
        holes.append(Hole(f"yawhorn{k}", (x0 + dx, y0 + dy, zh), (0, 0, 1.0), CLR3, fastener="M3x10 SHCS"))
    cc = S.case_center_x
    for X in (cc - S.hole_pitch_l / 2, cc + S.hole_pitch_l / 2):
        for Y in (-S.hole_pitch_w / 2, S.hole_pitch_w / 2):
            x, z = xs - X, zs - Y
            cuts.append(cyl_y(INS / 2, x, z, ey0 - 1, ey0 + 6.0))
            holes.append(Hole(f"shear_{X:+.1f}_{Y:+.0f}", (x, ey0, z), (0, 1.0, 0), INS, kind="insert",
                              extra="M3 heat-set insert (M3x5.7)"))
    cuts.append(cyl_y(B["D"] / 2, xs, zs, iy0 - 0.5, iy0 + B["B"]))
    cuts.append(cyl_y(2.9, xs, zs, iy0 - 1, iy1 + 1))
    holes.append(Hole("idler_brg", (xs, iy0, zs), (0, 1.0, 0), B["D"], kind="bearing"))
    body = cut_all(body, cuts)
    part = Part("arm_turntable", body, "PETG-G", holes=holes, subsystem="arm",
                descr="Turntable on yaw horn + 61806 bearing; shoulder servo plate + shoulder idler",
                print_note="Print plate-down, 5 walls, 40 % gyroid, supports under idler boss only")
    return _shift(part, TT_ORIGIN)


# --------------------------------------------------------------- upper arm
def upper_arm() -> Part:
    top, h1 = fork_top()
    hous, h2, hz1 = ds_housing(P.L_ARM1)
    col = link_column(-P.WEB_R0, hz1 - 0.01)
    body = union_all([top, hous, col])
    body = body.cut(box(-7.5, 7.5, -30, 20, hz1 + 4.0, -P.WEB_R1 - 4.0))
    return Part("upper_arm", body, "PETG-G", holes=h1 + h2, subsystem="arm",
                descr="Upper arm: fork on shoulder servo, houses elbow servo + elbow idler",
                print_note="Print lying on the outer tine face, 4 walls, 35 % gyroid")


# ---------------------------------------------------------------- forearm
def mg_housing(L: float):
    c = 0.3
    yct = MG_CASE_TOP_Y
    ye = yct + M.ear_z0                 # flange bottom face (seat)
    yb = yct - M.case_h + 0.25
    xo = 9.0
    z0, z1 = -L - M.shaft_off - c, -L + (M.case_l - M.shaft_off) + c
    hz0 = -L - (M.ear_span / 2 - M.case_center_x) - 3.8
    hz1 = -L + (M.ear_span / 2 + M.case_center_x) + 3.5
    main = box(-xo, xo, yb, ye, hz0, hz1).cut(box(-(M.case_w / 2 + c), M.case_w / 2 + c, yb - 1, ye + 1, z0, z1))
    sides = [box(sx * (M.case_w / 2 + c + 0.1), sx * xo, ye, yct - 1.0, hz0, hz1) for sx in (-1, 1)]
    body = union_all([main] + sides)
    holes, cuts = [], []
    cc = M.case_center_x
    for X in (cc - M.hole_pitch_l / 2, cc + M.hole_pitch_l / 2):
        z = -L + X
        cuts.append(cyl_y(0.9, 0.0, z, ye - 6.0, ye + 1))
        holes.append(Hole(f"wear_{X:+.1f}", (0.0, ye, z), (0, 1.0, 0), 1.8, kind="thread"))
    return cut_all(body, cuts), holes, hz1, (yb, yct - 1.0)


def forearm() -> Part:
    top, h1 = fork_top()
    hous, h2, hz1, (yb, yt) = mg_housing(P.L_ARM2)
    col = box(-10.0, 10.0, yb, yt, hz1 - 0.01, -P.WEB_R0)
    body = union_all([top, hous, col])
    body = body.cut(box(-5.0, 5.0, -30, 20, hz1 + 4.0, -P.WEB_R1 - 4.0))
    return Part("forearm", body, "PETG-G", holes=h1 + h2, subsystem="arm",
                descr="Forearm: fork on elbow servo, houses the MG90S wrist servo",
                print_note="Print lying on the outer tine face, 4 walls, 35 % gyroid")


# ---------------------------------------------------------------- gripper
GR = dict(
    plate=(-41.0, -36.0),        # mount plate z'
    servo_top=-48.0,             # gripper servo case top z' (shaft points -Z')
    pinion=(-62.0, -56.0),
    rack=(-61.5, -56.5),
    carriage=(-70.0, -62.0),
    finger=(-83.0, -62.0),
    rod_x=8.0, rod_z=-66.0,
    endplate=(33.0, 37.0),
    width=16.0,
    nozzle_x=-20.0,
    closed_gap=1.0,              # finger gap when fully closed (mm)
)
R_PITCH = P.GRIP_MODULE * P.GRIP_PINION_TEETH / 2.0


def gear_profile(z: int, m: float, pa_deg: float = 20.0, n_flank: int = 6):
    """2-D involute spur gear outline (list of (x, y))."""
    pa = np.radians(pa_deg)
    rp = m * z / 2
    rb = rp * np.cos(pa)
    ra = rp + m
    rf = rp - 1.25 * m
    inv = lambda a: np.tan(a) - a
    a_tip = np.arccos(rb / ra)
    half_tooth = np.pi / (2 * z) + inv(pa)              # angular half thickness at base circle
    pts = []
    for k in range(z):
        c = 2 * np.pi * k / z
        ts = np.linspace(max(0.0, np.arccos(min(1.0, rb / max(rf, rb)))), a_tip, n_flank)
        flank = []
        for a in ts:
            r = rb / np.cos(a)
            th = half_tooth - inv(a)
            flank.append((r, th))
        right = [(c - th, r) for (r, th) in flank]
        left = [(c + th, r) for (r, th) in reversed(flank)]
        root0 = (c - half_tooth - 0.35 * np.pi / z, rf)
        seq = [root0, (c - half_tooth, rf)] + right + left + [(c + half_tooth, rf)]
        for (a, r) in seq:
            pts.append((r * np.cos(a), r * np.sin(a)))
    # de-duplicate consecutive near points
    out = [pts[0]]
    for p in pts[1:]:
        if np.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-3:
            out.append(p)
    return out


def pinion() -> Part:
    z0, z1 = GR["pinion"]
    prof = gear_profile(P.GRIP_PINION_TEETH, P.GRIP_MODULE)
    g = cq.Workplane("XY", origin=(0, 0, z0)).polyline(prof).close().extrude(z1 - z0)
    holes = []
    for k, (x, y) in enumerate(bolt_circle(M.horn_pcd, 4, 45.0)):
        g = g.cut(cyl_z(1.1, x, y, z0 - 1, z1 + 1))
        holes.append(Hole(f"horn{k}", (x, y, z1), (0, 0, 1.0), 2.2, fastener="M2x8 self-tapping"))
    g = g.cut(cyl_z(1.5, 0, 0, z0 - 1, z1 + 1))
    return Part("gripper_pinion", g, "PETG", holes=holes, subsystem="arm",
                descr=f"Pinion m{P.GRIP_MODULE:g} z{P.GRIP_PINION_TEETH}, bolted to MG90S horn",
                print_note="Print flat, 0.12 mm layers, 100 % infill")


def gripper_base() -> Part:
    yo0 = F["horn_top"]
    yo1 = yo0 + 4.0
    zp0, zp1 = GR["plate"]
    e0, e1 = GR["endplate"]
    w = GR["width"]
    hornplate = cyl_y(12.0, 0, 0, yo0, yo1).union(box(-10.0, 10.0, yo0, yo1, zp1, 0.0))
    mplate = box(-24.0, w, -e1, e1, zp0, zp1)
    c = 0.3
    zt = GR["servo_top"]
    # gripper servo: X_s -> +Y', Y_s -> +X', Z_s -> -Z'
    mplate = mplate.cut(box(-(M.case_w / 2 + c), M.case_w / 2 + c, -M.shaft_off - c,
                            M.case_l - M.shaft_off + c, zp0 - 1, zp1 + 1))
    ends = [box(-w, w, sy * e0, sy * e1, GR["carriage"][0] - 2.0, zp1) for sy in (-1, 1)]
    clip = box(-24.0, -16.0, -6.0, 6.0, -54.0, zp0 + 0.01)
    body = union_all([hornplate, mplate, clip] + ends)
    cuts, holes = [], []
    for k, (x, z) in enumerate(bolt_circle(M.horn_pcd, 4, 45.0)):
        cuts.append(cyl_y(1.1, x, z, yo0 - 1, yo1 + 1))
        holes.append(Hole(f"whorn{k}", (x, yo0, z), (0, 1.0, 0), 2.2, fastener="M2x6 self-tapping"))
    cc = M.case_center_x
    for X in (cc - M.hole_pitch_l / 2, cc + M.hole_pitch_l / 2):
        cuts.append(cyl_z(0.9, 0.0, X, zp0 - 1, zp1 - 0.8))
        holes.append(Hole(f"gear_{X:+.1f}", (0.0, X, zp0), (0, 0, 1.0), 1.8, kind="thread"))
    for sx in (-1, 1):
        cuts.append(cyl_y(1.5, sx * GR["rod_x"], GR["rod_z"], -e1 - 1, e1 + 1))
        for sy in (-1, 1):
            holes.append(Hole(f"rod_{sx:+d}{sy:+d}", (sx * GR["rod_x"], sy * e0, GR["rod_z"]), (0, 1.0, 0), 3.0,
                              kind="bearing"))
    cuts.append(cyl_z(2.05, GR["nozzle_x"], 0.0, -60.0, zp1 + 1))
    body = cut_all(body, cuts)
    return Part("gripper_base", body, "PETG-G", holes=holes, subsystem="arm",
                descr="Gripper base: on wrist horn, carries gripper servo, guide rods, nozzle clip",
                print_note="Print with the mount plate on the bed, 4 walls, 40 % infill, supports for end plates")


def rack_teeth(x_pitch: float, y0: float, y1: float, z0: float, z1: float, phase: float = 0.0):
    """Straight rack (module m) with teeth pointing -X towards a pinion, pitch line at x_pitch."""
    m = P.GRIP_MODULE
    p = np.pi * m
    ha, hf = m, 1.25 * m
    tan = np.tan(np.radians(20.0))
    half_root = p / 4 + hf * tan
    half_tip = p / 4 - ha * tan
    teeth = []
    yc = y0 + phase + p / 2
    while yc + half_root <= y1 + 1e-9:
        poly = [(x_pitch + hf + 0.05, yc - half_root), (x_pitch - ha, yc - half_tip),
                (x_pitch - ha, yc + half_tip), (x_pitch + hf + 0.05, yc + half_root)]
        teeth.append(cq.Workplane("XY", origin=(0, 0, z0)).polyline(poly).close().extrude(z1 - z0))
        yc += p
    return union_all(teeth)


def gripper_jaw() -> Part:
    """Jaw A (moves along -Y' when opening). Jaw B = this part rotated 180° about Z'."""
    w = GR["width"]
    c0, c1 = GR["carriage"]
    g0 = GR["closed_gap"] / 2
    carriage = box(-w, w, -12.0 - g0, -g0, c0, c1)
    xr = R_PITCH
    hf = 1.25 * P.GRIP_MODULE
    rack_body = box(xr + hf, 15.0, -12.0 - g0, 32.0 - g0, GR["rack"][0], GR["rack"][1])
    # phase so that a tooth gap is centred on Y'=0 when closed (pinion tooth at 0° engages it)
    p = np.pi * P.GRIP_MODULE
    teeth = rack_teeth(xr, -12.0 - g0, 32.0 - g0, GR["rack"][0], GR["rack"][1], phase=(12.0 + g0 - p) % p)
    finger = box(-12.0, 12.0, -4.0 - g0, -g0, GR["finger"][0], c1)
    body = union_all([carriage, rack_body, teeth, finger])
    for sx in (-1, 1):
        body = body.cut(cyl_y(1.6, sx * GR["rod_x"], GR["rod_z"], -14, 1))
    holes = [Hole(f"rod{sx:+d}", (sx * GR["rod_x"], 0.0, GR["rod_z"]), (0, 1.0, 0), 3.2, kind="bearing", mate=False)
             for sx in (-1, 1)]
    return Part("gripper_jaw", body, "PETG", holes=holes, subsystem="arm",
                descr="Gripper jaw with integral rack (two identical jaws, mirrored by rotation)",
                print_note="Print on the carriage side, 0.12 mm layers, 5 walls; add 1 mm TPU pad tape")


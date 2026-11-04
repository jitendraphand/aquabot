"""Leg parts + the generic DS3218 fork-link generator (re-used by the arm).

Link frame convention ("L" = left-hand variant):
  origin on the top pivot axis in the link mid-plane, +Y along the top servo
  shaft (outboard), link extends along -Z, +X is the free (bending) side of
  the bottom joint.  "R" variants are exact mirror images (Y -> -Y).
"""
from __future__ import annotations

import numpy as np

from .. import params as P
from .geom import Hole, Part, box, bolt_circle, cut_all, cyl, cyl_x, cyl_y, cyl_z, sphere, union_all, \
    slot_xz_profile_y

S = P.DS3218
F = P.FORK
INS = P.INSERT_M3["d_hole"]
CLR3 = 3.4
B = P.IDLER_BEARING


def mirror_part(p: Part, name: str) -> Part:
    shape = p.shape.mirror("XZ")
    holes = [Hole(h.name, (h.p[0], -h.p[1], h.p[2]), (h.d[0], -h.d[1], h.d[2]), h.dia, h.kind,
                  h.fastener, h.extra, h.mate) for h in p.holes]
    return Part(name, shape, p.material, p.vendor, p.mass_g, holes, p.descr.replace("left", "right"),
                p.print_note, p.price_usd, p.supplier, p.rgba, mirror_of=p.name, subsystem=p.subsystem)


# --------------------------------------------------------------- fork top
def fork_top(horn_spec: P.ServoSpec = S):
    """Outer tine (on horn) + inner tine (idler bolt) + crossbar web."""
    yo0, yo1 = F["outer_tine"]
    yi0, yi1 = F["inner_tine"]
    w, R = P.TINE_HALF_W, P.TINE_R
    outer = cyl_y(R, 0, 0, yo0, yo1).union(box(-w, w, yo0, yo1, -P.WEB_R1, 0))
    inner = cyl_y(R, 0, 0, yi0, yi1).union(box(-w, w, yi0, yi1, -P.WEB_R1, 0))
    web = box(-w, w, yi0, yo1, -P.WEB_R1, -P.WEB_R0)
    body = union_all([outer, inner, web])
    cuts, holes = [], []
    for i, (x, z) in enumerate(bolt_circle(horn_spec.horn_pcd, 4, 45.0)):
        cuts.append(cyl_y(CLR3 / 2, x, z, yo0 - 1, yo1 + 1))
        holes.append(Hole(f"horn{i}", (x, yo0, z), (0, 1.0, 0), CLR3, fastener="M3x10 SHCS"))
    cuts.append(cyl_y(3.5, 0, 0, yo0 - 1, yo1 + 1))           # horn centre-screw access
    cuts.append(cyl_y(2.75, 0, 0, yi0 - 1, yi1 + 1))          # M5 pivot bolt
    cuts.append(cyl_y(4.75, 0, 0, yi1 - 3.5, yi1 + 1))        # low-head counterbore (inner face)
    holes.append(Hole("pivot", (0, yi0, 0), (0, 1.0, 0), 5.5, kind="clearance",
                      fastener=P.IDLER_BOLT))
    return cut_all(body, cuts), holes


# ---------------------------------------------------- DS3218 end housing
def ds_housing(L: float, with_idler: bool = True):
    """Housing for a DS3218 whose shaft is the bottom pivot at z = -L.

    Servo bulk points up the link (+Z), flange seats on the ear plate.
    """
    c = 0.3
    x_case = S.case_w / 2 + c          # 10.3
    xo = 14.5
    z0 = -L - S.shaft_off - c          # cavity
    z1 = -L + (S.case_l - S.shaft_off) + c
    hz0 = -L - (S.ear_span / 2 - S.case_center_x) - 3.75   # -L - 21
    hz1 = -L + (S.ear_span / 2 + S.case_center_x) + 3.75   # -L + 41
    ye0 = F["ear_plate"][1]           # 3.75 (flange seat)
    yb = F["case_bot"] + 0.25         # -23.0
    main = box(-xo, xo, yb, ye0, hz0, hz1).cut(box(-x_case, x_case, yb - 1, ye0 + 1, z0, z1))
    sides = [box(sx * (x_case + 0.1), sx * xo, ye0, F["case_top"] - 1.25, hz0, hz1) for sx in (-1, 1)]
    body = union_all([main] + sides)
    cuts, holes = [], []
    cc = S.case_center_x
    for X in (cc - S.hole_pitch_l / 2, cc + S.hole_pitch_l / 2):
        for Y in (-S.hole_pitch_w / 2, S.hole_pitch_w / 2):
            x, z = Y, -L + X           # servo Y -> +X, servo X -> +Z
            cuts.append(cyl_y(INS / 2, x, z, ye0 - 6.0, ye0 + 1))
            holes.append(Hole(f"ear_{X:+.1f}_{Y:+.0f}", (x, ye0, z), (0, 1.0, 0), INS, kind="insert",
                              extra="M3 heat-set insert (M3x5.7)"))
    if with_idler:
        yI0, yI1 = P.FORK["idler"]
        bridge = box(-xo, -4.0, yI0, F["case_bot"] - 0.35, -L + 17.0, hz1).union(
            box(-xo, -(x_case + 0.2), F["case_bot"] - 0.5, yb + 0.5, -L + 17.0, hz1))
        idler = box(-xo, 10.0, yI0, yI1, -L - 10.0, hz1).union(cyl_y(11.0, 0, -L, yI0, yI1))
        body = union_all([body, bridge, idler])
        cuts.append(cyl_y(B["D"] / 2, 0, -L, yI1 - B["B"], yI1 + 0.5))    # bearing seat
        cuts.append(cyl_y(2.9, 0, -L, yI0 - 1, yI1 + 1))
        holes.append(Hole("idler_brg", (0, yI1, -L), (0, 1.0, 0), B["D"], kind="bearing"))
    body = cut_all(body, cuts)
    return body, holes, hz1


def link_column(z_top: float, z_bot: float, xo: float = 12.0, y0: float = -23.0, y1: float = 16.0):
    return box(-xo, xo, y0, y1, z_bot, z_top)


# ----------------------------------------------------------------- femur
def femur_L() -> Part:
    top, h1 = fork_top()
    hous, h2, hz1 = ds_housing(P.L_FEMUR)
    col = link_column(-P.WEB_R0, hz1 - 0.01)
    body = union_all([top, hous, col])
    # lightening window through the column (Y direction)
    body = body.cut(box(-6.0, 6.0, -30, 20, hz1 + 2.0, -P.WEB_R1 - 2.0)) if hz1 + 2.0 < -P.WEB_R1 - 2.0 else body
    return Part("femur_L", body, "PETG", holes=h1 + h2, subsystem="leg",
                descr="Femur (left): fork on hip-pitch servo, houses knee servo, carries knee idler",
                print_note="Print lying on the outer tine face, 4 walls, 35 % gyroid, tree supports under idler")


# ----------------------------------------------------------------- tibia
def tibia_L() -> Part:
    top, holes = fork_top()
    zb = -P.L_TIBIA
    shin = slot_xz_profile_y([(-P.TINE_HALF_W, -P.WEB_R0), (P.TINE_HALF_W, -P.WEB_R0),
                              (7.0, zb + 12.0), (-7.0, zb + 12.0)], -7.0, 7.0)
    spigot = cyl_z(6.0, 0, 0, zb - 1.0, zb + 14.0)
    body = union_all([top, shin, spigot])
    body = body.cut(cyl_x(1.7, 0, zb + 5.0, -8, 8))
    holes.append(Hole("foot_pin", (-6.0, 0, zb + 5.0), (1.0, 0, 0), 3.4, fastener="M3x16 SHCS + M3 nyloc"))
    return Part("tibia_L", body, "PETG", holes=holes, subsystem="leg",
                descr="Tibia (left): fork on knee servo, tapered shin, foot spigot",
                print_note="Print lying on the outer tine face, 4 walls, 35 % gyroid")


def foot_pad() -> Part:
    zb = -P.L_TIBIA
    body = sphere(P.FOOT_R, (0, 0, zb))
    body = body.cut(box(-20, 20, -20, 20, zb + 9.0, zb + 20))      # QA iter 8: no knife edge at the bore
    body = body.cut(cyl_z(6.15, 0, 0, zb - 1.5, zb + 20))
    body = body.cut(cyl_x(1.7, 0, zb + 5.0, -20, 20))
    holes = [Hole("pin", (-6.0, 0, zb + 5.0), (1.0, 0, 0), 3.4)]
    return Part("foot_pad", body, "TPU95A", holes=holes, subsystem="leg",
                descr="Ø26 TPU foot (sphere contact, slips over the tibia spigot)",
                print_note="TPU 95A, print bore-up, 3 walls, 25 % infill, slow (25 mm/s)")


# ------------------------------------------------------- shoulder bracket
def shoulder_bracket_L() -> Part:
    """Hip link for the front-left leg, in the J1 joint frame.

    Origin: J1 horn top face centre. Axes body-aligned (FL). J2 pivot at
    (J2_DX, J2_DY, J2_DZ).
    """
    jx, jy, jz = P.J2_DX, P.J2_DY, P.J2_DZ
    t = P.BRACKET_T
    e0, e1 = jy + F["ear_plate"][0], jy + F["ear_plate"][1]   # ear plate y
    i0, i1 = jy + F["idler"][0], jy + F["idler"][1]           # idler arm y
    c = 0.3
    # J2 servo: servo X -> +z, servo Y -> +x, servo Z -> +y
    wx0, wx1 = jx - S.case_w / 2 - c, jx + S.case_w / 2 + c
    wz0, wz1 = jz - S.shaft_off - c, jz + (S.case_l - S.shaft_off) + c
    ztop = jz + S.ear_span / 2 + S.case_center_x + 3.75       # ~ +9
    zbot = jz - (S.ear_span / 2 - S.case_center_x) - 3.75     # ~ -53
    x_front = jx + 14.0
    back = box(0, t, -14.0, e1, -16.0, 16.0)
    ear = slot_xz_profile_y([(t, ztop), (x_front, ztop), (x_front, zbot), (jx - 14.0, zbot),
                             (jx - 14.0, jz + 5.0), (t, jz + 5.0)], e0, e1)
    ear = ear.cut(box(wx0, wx1, e0 - 1, e1 + 1, wz0, wz1))
    idler = box(t, jx + 10.0, i0, i1, jz - 10.0, 16.0).union(cyl_y(10.0, jx, jz, i0, i1))
    topp = box(t, x_front, i0, e1, ztop + 2.0, 16.0)
    gus = box(t, wx0 - 0.5, i0, e1, -16.0, -10.0)
    body = union_all([back, ear, idler, topp, gus])
    cuts, holes = [], []
    for k, (y, z) in enumerate(bolt_circle(S.horn_pcd, 4, 45.0)):
        cuts.append(cyl_x(INS / 2, y, z, -1, t + 1))
        holes.append(Hole(f"j1horn{k}", (0, y, z), (1.0, 0, 0), INS, kind="insert",
                          extra="M3 heat-set insert (M3x5.7)",
                          fastener="M3x10 countersunk (horn holes drilled Ø3.2)"))
    cuts.append(cyl_x(4.0, 0, 0, -1, t + 1))
    cc = S.case_center_x
    for X in (cc - S.hole_pitch_l / 2, cc + S.hole_pitch_l / 2):
        for Y in (-S.hole_pitch_w / 2, S.hole_pitch_w / 2):
            x, z = jx + Y, jz + X
            cuts.append(cyl_y(INS / 2, x, z, e0 - 1, e1 + 1))
            holes.append(Hole(f"j2ear_{X:+.1f}_{Y:+.0f}", (x, e1, z), (0, 1.0, 0), INS, kind="insert",
                              extra="M3 heat-set insert (M3x5.7)"))
    cuts.append(cyl_y(B["D"] / 2, jx, jz, i1 - B["B"], i1 + 0.5))
    cuts.append(cyl_y(2.9, jx, jz, i0 - 1, i1 + 1))
    holes.append(Hole("idler_brg", (jx, i1, jz), (0, 1.0, 0), B["D"], kind="bearing"))
    body = cut_all(body, cuts)
    return Part("shoulder_bracket_L", body, "PETG-G", holes=holes, subsystem="leg",
                descr="Shoulder bracket (FL / RR): on J1 horn, holds hip-pitch servo + femur idler",
                print_note="Print standing on the back plate, 4 walls, 40 % gyroid")


def build_leg_parts() -> dict:
    fl = femur_L()
    tl = tibia_L()
    bl = shoulder_bracket_L()
    return {
        "femur_L": fl, "femur_R": mirror_part(fl, "femur_R"),
        "tibia_L": tl, "tibia_R": mirror_part(tl, "tibia_R"),
        "shoulder_bracket_L": bl, "shoulder_bracket_R": mirror_part(bl, "shoulder_bracket_R"),
        "foot_pad": foot_pad(),
    }

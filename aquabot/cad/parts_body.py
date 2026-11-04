"""Printed body / water-system / electronics-housing parts (body frame coords)."""
from __future__ import annotations

import numpy as np

from .. import params as P
from .geom import (Hole, Part, box, cut_all, cyl, cyl_x, cyl_y, cyl_z, tube,
                   union_all, bolt_circle)

S = P.DS3218
INS = P.INSERT_M3["d_hole"]
INS_DEPTH = P.INSERT_M3["depth"]
CLR3 = 3.4

# Tub <-> bulkhead joint locations (y, z) and tub <-> deck locations (x, y)
BULK_JOINTS = [(-39.5, -42.5), (39.5, -42.5), (0.0, -42.5), (-39.5, 30.0), (39.5, 30.0)]  # QA: -42.5 avoids tangency with floor
DECK_JOINTS = [(x, y) for x in (-80.0, 0.0, 80.0) for y in (-39.5, 39.5)]


def j1_ear_holes_yz():
    """J1 flange holes in the bulkhead plane (y, z)."""
    cc = S.case_center_x  # flange centre along the case (servo X), servo X -> -z
    zs = [-(cc - S.hole_pitch_l / 2), -(cc + S.hole_pitch_l / 2)]
    out = []
    for sy in (-1, 1):
        for dy in (-S.hole_pitch_w / 2, S.hole_pitch_w / 2):
            for z in zs:
                out.append((sy * P.Y_J1 + dy, z))
    return out


def j1_window(sy):
    c = 0.3
    x0 = -S.shaft_off
    x1 = S.case_l - S.shaft_off
    # servo X -> -z ; servo Y -> ±y
    return (sy * P.Y_J1 - S.case_w / 2 - c, sy * P.Y_J1 + S.case_w / 2 + c, -x1 - c, -x0 + c)


# --------------------------------------------------------------------- tub
def chassis_tub() -> Part:
    X, W = P.TUB_X, P.BODY_W
    outer = box(-X, X, -W, W, P.TUB_Z0, P.TUB_Z1)
    inner = box(-X - 1, X + 1, -W + P.WALL_T, W - P.WALL_T, P.TUB_Z0 + P.FLOOR_T, P.TUB_Z1 + 1)
    body = outer.cut(inner)
    adds, cuts, holes = [], [], []
    # bulkhead bosses (horizontal) at both ends
    for ex in (-1, 1):
        for (y, z) in BULK_JOINTS:
            x_end = ex * X
            adds.append(cyl_x(P.BOSS_D / 2, y, z, ex * (X - 12.0), x_end))
            cuts.append(cyl_x(INS / 2, y, z, ex * (X - INS_DEPTH - 1.0), x_end + ex * 0.5))
            holes.append(Hole(f"bulk_{'F' if ex > 0 else 'R'}_{y:+.0f}_{z:+.0f}", (x_end, y, z),
                              (1.0, 0, 0), INS, kind="insert", extra="M3 heat-set insert (M3x5.7)"))
    # deck bosses (vertical)
    for (x, y) in DECK_JOINTS:
        adds.append(cyl_z(P.BOSS_D / 2, x, y, 18.0, P.TUB_Z1))
        # gusset down to the wall so the boss is printable without support
        adds.append(box(x - 3.0, x + 3.0, np.sign(y) * (abs(y) - 1.0), np.sign(y) * (W - P.WALL_T + 0.5),
                        8.0, P.TUB_Z1))
        cuts.append(cyl_z(INS / 2, x, y, P.TUB_Z1 - INS_DEPTH - 1.0, P.TUB_Z1 + 0.5))
        holes.append(Hole(f"deck_{x:+.0f}_{y:+.0f}", (x, y, P.TUB_Z1), (0, 0, 1.0), INS,
                          kind="insert", extra="M3 heat-set insert (M3x5.7)"))
    body = union_all([body] + adds)
    # cable pass-throughs (leg servo leads) and floor drains
    for ex in (-1, 1):
        for sy in (-1, 1):
            cuts.append(cyl_y(6.0, ex * 55.0, 24.0, sy * (W + 1), sy * (W - P.WALL_T - 1)))
    for x in (-40.0, 40.0):
        cuts.append(cyl_z(3.0, x, 0.0, P.TUB_Z0 - 1, P.TUB_Z0 + P.FLOOR_T + 1))
    # QA iter 8 (mass budget): lightening windows in the side walls, clear of bosses and tank lid
    for sy in (-1, 1):
        for x0, x1 in ((-58.0, -8.0), (8.0, 46.0)):
            cuts.append(box(x0, x1, sy * (W + 1), sy * (W - P.WALL_T - 1), -38.0, -6.0))
    body = cut_all(body, cuts)
    return Part("chassis_tub", body, "PETG", holes=holes, subsystem="body",
                descr="Main body tub: floor + side walls, insert bosses for bulkheads and deck",
                print_note="Print upright (open top up), 4 walls, 30 % gyroid, no supports")


# --------------------------------------------------------------- bulkheads
def bulkhead(front: bool) -> Part:
    X0, X1 = P.TUB_X, P.TUB_X + P.BULKHEAD_T
    W = P.BODY_W
    body = box(X0, X1, -W, W, P.BULK_Z0, P.BULK_Z1)
    cuts, holes = [], []
    for sy in (-1, 1):
        y0, y1, z0, z1 = j1_window(sy)
        cuts.append(box(X0 - 1, X1 + 1, y0, y1, z0, z1))
    for (y, z) in j1_ear_holes_yz():
        cuts.append(cyl_x(INS / 2, y, z, X0 - 1, X1 + 1))
        holes.append(Hole(f"j1ear_{y:+.0f}_{z:+.1f}", (X1, y, z), (1.0, 0, 0), INS, kind="insert",
                          extra="M3 heat-set insert (M3x5.7)"))
    for (y, z) in BULK_JOINTS:
        cuts.append(cyl_x(CLR3 / 2, y, z, X0 - 1, X1 + 1))
        cuts.append(cyl_x(3.1, y, z, X1 - 2.0, X1 + 1))  # counterbore for SHCS head
        holes.append(Hole(f"tub_{y:+.0f}_{z:+.0f}", (X0, y, z), (1.0, 0, 0), CLR3, kind="clearance",
                          fastener="M3x12 SHCS"))
    # leg servo cable grommets
    for sy in (-1, 1):
        cuts.append(cyl_x(4.5, sy * 9.0, -40.0, X0 - 1, X1 + 1))
    if front:
        cuts.append(box(X0 - 1, X1 + 1, -6.0, 6.0, -18.0, -8.0))     # ToF window
        for y in (-10.0, 10.0):
            cuts.append(cyl_x(1.1, y, -13.0, X0 - 1, X1 + 1))        # M2 self-tap
    else:
        cuts.append(box(X0 - 1, X1 + 1, -6.5, 6.5, -22.0, -3.0))      # rocker switch 13x19
        cuts.append(box(X0 - 1, X1 + 1, -8.2, 8.2, 19.8, 28.2))      # XT60 panel cut-out
        for y in (-10.5, 10.5):
            cuts.append(cyl_x(1.3, y, 24.0, X0 - 1, X1 + 1))
    body = cut_all(body, cuts)
    name = "bulkhead_front" if front else "bulkhead_rear"
    return Part(name, body, "PETG", holes=holes, subsystem="body",
                descr=("Front" if front else "Rear") + " bulkhead: carries two J1 hip-abduction servos"
                      + (" + ToF sensor window" if front else " + power switch & XT60 charge port"),
                print_note="Print flat, 5 walls around insert holes, 40 % gyroid")


# -------------------------------------------------------------------- deck
PEDESTAL_SCREWS = [(40.5, -39.5), (69.5, -39.5), (40.5, 19.5), (69.5, 19.5)]   # = PED_POSTS
TRAY_SCREWS = [(-24.0, -6.0), (28.0, -6.0), (-24.0, 2.0), (28.0, 2.0)]
TUBE_HOLE = (-38.0, 0.0)


def top_deck() -> Part:
    X, W = P.TUB_X, P.BODY_W
    z0, z1 = P.TUB_Z1, P.DECK_Z1
    body = box(-X, X, -W, W, z0, z1)
    cuts, holes = [], []
    for (x, y) in DECK_JOINTS:
        cuts.append(cyl_z(CLR3 / 2, x, y, z0 - 1, z1 + 1))
        holes.append(Hole(f"tub_{x:+.0f}_{y:+.0f}", (x, y, z0), (0, 0, 1.0), CLR3, fastener="M3x10 SHCS"))
    fx, fy = P.FILL_NECK["x"], P.FILL_NECK["y"]
    cuts.append(cyl_z(P.FILL_NECK["od"] / 2 + 1.0, fx, fy, z0 - 1, z1 + 1))
    for (x, y) in PEDESTAL_SCREWS:
        cuts.append(cyl_z(INS / 2, x, y, z0 - 1, z1 + 1))
        holes.append(Hole(f"ped_{x:+.0f}_{y:+.0f}", (x, y, z1), (0, 0, 1.0), INS, kind="insert",
                          extra="M3 heat-set insert (M3x4)"))
    for (x, y) in TRAY_SCREWS:
        cuts.append(cyl_z(INS / 2, x, y, z0 - 1, z1 + 1))
        holes.append(Hole(f"tray_{x:+.0f}_{y:+.0f}", (x, y, z1), (0, 0, 1.0), INS, kind="insert",
                          extra="M3 heat-set insert (M3x4)"))
    cuts.append(box(-20.0, 20.0, -16.0, -10.0, z0 - 1, z1 + 1))          # wiring slot under tray
    cuts.append(box(42.0, 68.0, -12.0, -4.0, z0 - 1, z1 + 1))            # yaw servo lead
    cuts.append(cyl_z(4.0, TUBE_HOLE[0], TUBE_HOLE[1], z0 - 1, z1 + 1))   # water tube
    # lightening pockets at the rear (underside, keeps a 2 mm skin)
    cuts.append(box(-86.0, -66.0, -30.0, 10.0, z0 - 1, z1 - 2.0))
    body = cut_all(body, cuts)
    return Part("top_deck", body, "PETG", holes=holes, subsystem="body",
                descr="Top deck: closes the tub, carries arm pedestal, electronics tray, fill neck",
                print_note="Print flat, 4 walls, 30 % gyroid")


# ------------------------------------------------------------ water system
def tank_body() -> Part:
    T = P.TANK
    x0, x1, y, z0, z1, w, f = T["x0"], T["x1"], T["y"], T["z0"], T["z1"], T["wall"], T["floor"]
    outer = box(x0, x1, -y, y, z0, z1)
    inner = box(x0 + w, x1 - w, -y + w, y - w, z0 + f, z1 + 1)
    body = outer.cut(inner)
    adds, cuts, holes = [], [], []
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = (x1 - w - 3.0) if sx > 0 else (x0 + w + 3.0)
            cy = sy * (y - w - 3.0)
            adds.append(cyl_z(4.0, cx, cy, z1 - 14.0, z1))
            # quarter-blend into the corner
            adds.append(box(cx, cx + sx * 4.5, cy - 0.0, cy + sy * 4.5, z1 - 14.0, z1))
            cuts.append(cyl_z(INS / 2, cx, cy, z1 - INS_DEPTH - 1, z1 + 1))
            holes.append(Hole(f"lid_{sx:+d}{sy:+d}", (cx, cy, z1), (0, 0, 1.0), INS, kind="insert",
                              extra="M3 heat-set insert (M3x5.7)"))
    # anti-slosh baffle with flow ports (transverse, at x = 0)
    baffle = box(-1.0, 1.0, -y + w, y - w, z0 + f, z1 - 8.0)
    for yy in (-24.0, 0.0, 24.0):
        baffle = baffle.cut(cyl_x(6.0, yy, z0 + f + 6.0, -2, 2))
    adds.append(baffle)
    body = cut_all(union_all([body] + adds), cuts)
    return Part("tank_body", body, "PETG-B", holes=holes, subsystem="water",
                descr="400 ml water reservoir with anti-slosh baffle (sits on tub floor)",
                print_note="PETG, 100 % solid walls (or 6 walls), 0.2 mm layers, then seal inside "
                           "with food-safe epoxy; leak-test before install")


def tank_lid() -> Part:
    T = P.TANK
    x0, x1, y, w = T["x0"], T["x1"], T["y"], T["wall"]
    z0 = T["z1"]
    z1 = z0 + P.LID_T
    body = box(x0, x1, -y, y, z0, z1)
    fx, fy = P.FILL_NECK["x"], P.FILL_NECK["y"]
    neck = tube(P.FILL_NECK["od"] / 2, P.FILL_NECK["id"] / 2, (fx, fy, z0), (fx, fy, P.FILL_NECK["top"]))
    dx, dy = P.DIP_PORT["x"], P.DIP_PORT["y"]
    barb_up = cyl_z(1.6, dx, dy, z1, z1 + 8.0).union(cyl_z(1.9, dx, dy, z1 + 5.5, z1 + 6.5))
    barb_dn = cyl_z(1.6, dx, dy, z0 - 8.0, z0).union(cyl_z(1.9, dx, dy, z0 - 6.5, z0 - 5.5))
    boss = cyl_z(3.5, dx, dy, z1, z1 + 1.5)
    body = union_all([body, neck, barb_up, barb_dn, boss])
    cuts, holes = [], []
    cuts.append(cyl_z(P.FILL_NECK["id"] / 2, fx, fy, z0 - 1, P.FILL_NECK["top"] + 1))
    cuts.append(cyl_z(0.75, dx, dy, z0 - 9, z1 + 9))
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = (x1 - w - 3.0) if sx > 0 else (x0 + w + 3.0)
            cy = sy * (y - w - 3.0)
            cuts.append(cyl_z(CLR3 / 2, cx, cy, z0 - 1, z1 + 1))
            holes.append(Hole(f"tank_{sx:+d}{sy:+d}", (cx, cy, z0), (0, 0, 1.0), CLR3,
                              fastener="M3x10 SHCS + Ø7 nylon washer"))
    body = cut_all(body, cuts)
    return Part("tank_lid", body, "PETG-B", holes=holes, subsystem="water",
                descr="Tank lid with fill neck (Ø20) and dip-tube barb for the peristaltic pump",
                print_note="Print flat, neck up, 6 walls; gasket: 1 mm silicone sheet")


def fill_cap() -> Part:
    fx, fy, top = P.FILL_NECK["x"], P.FILL_NECK["y"], P.FILL_NECK["top"]
    plug = cyl_z(P.FILL_NECK["id"] / 2 - 0.2, fx, fy, top - 10.0, top)
    groove = tube(P.FILL_NECK["id"] / 2 + 1, P.FILL_NECK["id"] / 2 - 1.6, (fx, fy, top - 6.5), (fx, fy, top - 4.5))
    head = cyl_z(15.0, fx, fy, top, top + 6.0)
    grip = box(fx - 2.0, fx + 2.0, fy - 12.0, fy + 12.0, top + 6.0, top + 12.0)
    body = union_all([plug.cut(groove), head, grip])
    body = body.cut(cyl_z(0.6, fx + 6.0, fy, top - 11, top + 13))   # vent (pump draws air)
    return Part("fill_cap", body, "PETG-B", subsystem="water",
                descr="Fill cap: push-fit plug with 16x2 O-ring and Ø1.2 air vent",
                print_note="Print head-down, 100 % infill")


# --------------------------------------------------------- electronics bay
TRAY = dict(x0=-31.0, x1=32.8, y=43.6, z0=P.DECK_Z1, t=3.0)
COVER = dict(x0=-31.0 - 2.4, x1=35.0, y=P.BODY_W, z1=66.0, wall=2.0)
POSTS = [(-27.0, -40.0), (29.0, -40.0), (-27.0, 40.0), (29.0, 40.0)]
PCA_POS = (-29.5, -36.0)          # PCB corner (x, y)
ESP_POS = (-24.0, 7.0)
MOSFET_POS = (0.0, -10.2)


def electronics_tray() -> Part:
    T = TRAY
    z0, z1 = T["z0"], T["z0"] + T["t"]
    body = box(T["x0"], T["x1"], -T["y"], T["y"], z0, z1)
    adds, cuts, holes = [], [], []
    for (x, y) in POSTS:
        adds.append(cyl_z(3.5, x, y, z1, COVER["z1"] - COVER["wall"]))
        cuts.append(cyl_z(INS / 2, x, y, COVER["z1"] - COVER["wall"] - INS_DEPTH - 1, COVER["z1"]))
        holes.append(Hole(f"cov_{x:+.0f}_{y:+.0f}", (x, y, COVER["z1"] - COVER["wall"]), (0, 0, 1.0), INS,
                          kind="insert", extra="M3 heat-set insert (M3x5.7)"))
    # PCA9685 standoffs (M2.5, 56 x 19 pattern)
    px, py = PCA_POS
    for dx in (3.0, 59.0):
        for dy in (3.2, 22.2):
            adds.append(cyl_z(2.6, px + dx, py + dy, z1, z1 + 3.0))
            cuts.append(cyl_z(1.1, px + dx, py + dy, z1 - 1, z1 + 4))
    # ESP32 cradle rails
    ex, ey = ESP_POS
    adds.append(box(ex + 4.0, ex + 50.0, ey - 2.0, ey, z1, z1 + 5.0))
    adds.append(box(ex + 4.0, ex + 50.0, ey + 28.0, ey + 30.0, z1, z1 + 5.0))
    body = union_all([body] + adds)
    for (x, y) in TRAY_SCREWS:
        cuts.append(cyl_z(CLR3 / 2, x, y, z0 - 1, z1 + 1))
        holes.append(Hole(f"deck_{x:+.0f}_{y:+.0f}", (x, y, z0), (0, 0, 1.0), CLR3, fastener="M3x8 SHCS"))
    cuts.append(box(-20.0, 20.0, -16.0, -10.0, z0 - 1, z1 + 1))
    body = cut_all(body, cuts)
    return Part("electronics_tray", body, "PETG", holes=holes, subsystem="electronics",
                descr="Electronics tray: PCA9685 standoffs, ESP32 cradle, cover posts",
                print_note="Print flat, 3 walls, 20 % infill")


def electronics_cover() -> Part:
    C = COVER
    z0 = P.DECK_Z1
    outer = box(C["x0"], C["x1"], -C["y"], C["y"], z0, C["z1"])
    inner = box(C["x0"] + C["wall"], C["x1"] - C["wall"], -C["y"] + C["wall"], C["y"] - C["wall"],
                z0 - 1, C["z1"] - C["wall"])
    body = outer.cut(inner)
    cuts, holes = [], []
    for (x, y) in POSTS:
        cuts.append(cyl_z(CLR3 / 2, x, y, C["z1"] - C["wall"] - 1, C["z1"] + 1))
        holes.append(Hole(f"post_{x:+.0f}_{y:+.0f}", (x, y, C["z1"] - C["wall"]), (0, 0, 1.0), CLR3,
                          fastener="M3x8 button head"))
    for i in range(6):                                            # vent slots
        y = -30.0 + i * 12.0
        cuts.append(box(-10.0, 22.0, y, y + 4.0, C["z1"] - C["wall"] - 1, C["z1"] + 1))
    cuts.append(box(C["x0"] - 1, C["x0"] + C["wall"] + 1, 14.0, 26.0, z0 + 6.0, z0 + 13.0))  # USB
    cuts.append(box(C["x0"] - 1, C["x0"] + C["wall"] + 1, -8.0, 8.0, z0 - 1, z0 + 6.0))      # harness
    body = cut_all(body, cuts)
    return Part("electronics_cover", body, "PETG-G", holes=holes, subsystem="electronics",
                descr="Splash cover over ESP32 + PCA9685 (vent slots, USB access)",
                print_note="Print upside-down (top on bed), 3 walls")


# ----------------------------------------------------------- arm pedestal
YAW = dict(x=P.ARM_YAW_XY[0], y=P.ARM_YAW_XY[1])
YAW_CASE_BOT = P.DECK_Z1 + 1.5
YAW_CASE_TOP = YAW_CASE_BOT + S.case_h            # 82.0
YAW_HORN_TOP = YAW_CASE_TOP + S.horn_top           # 89.5
PED_PLATE = (YAW_CASE_TOP + 0.5, YAW_HORN_TOP - 3.5)   # bearing plate z range
PED_POSTS = [(40.5, -39.5), (69.5, -39.5), (40.5, 19.5), (69.5, 19.5)]
BEAR_Z0 = PED_PLATE[1] + 1.0
BEAR_Z1 = BEAR_Z0 + P.YAW_BEARING["B"]


def yaw_servo_box():
    """Yaw servo footprint in body coords. Servo X -> -y, servo Y -> +x."""
    x0, x1 = -S.shaft_off, S.case_l - S.shaft_off
    return (YAW["x"] - S.case_w / 2, YAW["x"] + S.case_w / 2, YAW["y"] - x1, YAW["y"] - x0)


def arm_pedestal() -> Part:
    cx0, cx1, cy0, cy1 = yaw_servo_box()
    c = 0.3
    wall = 3.0
    ex0, ex1 = YAW["y"] - (S.case_center_x + S.ear_span / 2), YAW["y"] - (S.case_center_x - S.ear_span / 2)
    ix0, ix1 = cx0 - c, cx1 + c
    iy0, iy1 = ex0 - c, ex1 + c
    z0, z1 = P.DECK_Z1, PED_PLATE[0]
    flange = box(36.5, 73.5, -44.0, 24.0, z0, z0 + 3.0).cut(box(ix0, ix1, iy0, iy1, z0 - 1, z0 + 4))
    walls = box(ix0 - wall, ix1 + wall, iy0 - wall, iy1 + wall, z0, z1).cut(box(ix0, ix1, iy0, iy1, z0 - 1, z1 + 1))
    shelf_z1 = YAW_CASE_TOP + S.ear_z0
    shelf = box(ix0, ix1, iy0, iy1, shelf_z1 - 5.0, shelf_z1).cut(
        box(cx0 - c, cx1 + c, cy0 - c, cy1 + c, shelf_z1 - 6, shelf_z1 + 1))
    posts = [cyl_z(4.0, x, y, z0, z1) for (x, y) in PED_POSTS]
    body = union_all([flange, walls, shelf] + posts)
    cuts, holes = [], []
    for (x, y) in PED_POSTS:
        cuts.append(cyl_z(CLR3 / 2, x, y, z0 - 1, z1 + 1))     # M3x50 through-bolt
    # yaw servo flange holes (inserts in the shelf). servo X -> -y, servo Y -> +x
    cc = S.case_center_x
    for X in (cc - S.hole_pitch_l / 2, cc + S.hole_pitch_l / 2):
        for Y in (-S.hole_pitch_w / 2, S.hole_pitch_w / 2):
            x, y = YAW["x"] + Y, YAW["y"] - X
            cuts.append(cyl_z(INS / 2, x, y, shelf_z1 - 6, shelf_z1 + 1))
            holes.append(Hole(f"yawear_{X:+.1f}_{Y:+.0f}", (x, y, shelf_z1), (0, 0, 1.0), INS, kind="insert",
                              extra="M3 heat-set insert (M3x5.7)"))
    cuts.append(box(ix0 - wall - 1, ix0 + 1, -12.0, -4.0, z0 + 3.0, z0 + 12.0))  # servo lead exit
    body = cut_all(body, cuts)
    return Part("arm_pedestal", body, "PETG-G", holes=holes, subsystem="arm",
                descr="Arm pedestal: houses the yaw servo (flange on internal shelf); clamped by 4 M3x50 through-bolts",
                print_note="Print upright, 4 walls, 30 % gyroid, no supports (shelf bridges 21 mm)")


def arm_bearing_plate() -> Part:
    z0, z1 = PED_PLATE
    cx0, cx1, cy0, cy1 = yaw_servo_box()
    plate = box(36.5, 73.5, -44.0, 24.0, z0, z1)
    boss = tube(P.YAW_BEARING["d"] / 2, 13.5, (YAW["x"], YAW["y"], z1), (YAW["x"], YAW["y"], BEAR_Z1))
    ring = tube(16.5, 13.5, (YAW["x"], YAW["y"], z1), (YAW["x"], YAW["y"], BEAR_Z0))
    body = union_all([plate, boss, ring])
    cuts, holes = [], []
    cuts.append(cyl_z(13.5, YAW["x"], YAW["y"], z0 - 1, z1 + 0.01))
    for (x, y) in PED_POSTS:
        cuts.append(cyl_z(CLR3 / 2, x, y, z0 - 1, z1 + 1))
        holes.append(Hole(f"ped_{x:+.0f}_{y:+.0f}", (x, y, z0), (0, 0, 1.0), CLR3,
                          fastener="M3x50 SHCS (through pedestal post into deck insert)"))
    body = cut_all(body, cuts)
    return Part("arm_bearing_plate", body, "PETG-G", holes=holes, subsystem="arm",
                descr="Bearing plate: carries the 61806 thin-section yaw bearing on a Ø30 boss",
                print_note="Print flat, boss up, 5 walls (bearing seat), 40 % infill")

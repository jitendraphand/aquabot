"""Simplified envelope models of purchased components.

These are *envelopes* for fit / interference / mass checks, not detailed
models. Every dimension comes from params.py and is datasheet-based.
"""
from __future__ import annotations

import numpy as np

from .. import params as P
from .geom import Hole, Part, box, cyl, cyl_x, cyl_y, cyl_z, tube, union_all, bolt_circle

BLACK = (0.08, 0.08, 0.09, 1.0)
DARKBLUE = (0.10, 0.14, 0.30, 1.0)
ALU = (0.75, 0.76, 0.78, 1.0)
STEEL = (0.55, 0.56, 0.58, 1.0)
PCB_G = (0.05, 0.40, 0.18, 1.0)
PCB_B = (0.05, 0.20, 0.55, 1.0)
LIPO = (0.85, 0.65, 0.10, 1.0)
BRASS = (0.80, 0.62, 0.25, 1.0)


def servo(spec: P.ServoSpec, name: str) -> Part:
    s = spec
    x0, x1 = -s.shaft_off, s.case_l - s.shaft_off
    cc = s.case_center_x
    case = box(x0, x1, -s.case_w / 2, s.case_w / 2, -s.case_h, 0.0)
    ears = box(cc - s.ear_span / 2, cc + s.ear_span / 2, -s.case_w / 2, s.case_w / 2, s.ear_z0, s.ear_z1)
    boss = cyl_z(s.boss_d / 2, 0, 0, 0.0, s.boss_h)
    body = union_all([case, ears, boss])
    holes = []
    ys = [-s.hole_pitch_w / 2, s.hole_pitch_w / 2] if s.hole_pitch_w > 0 else [0.0]
    for xi in (cc - s.hole_pitch_l / 2, cc + s.hole_pitch_l / 2):
        for yi in ys:
            body = body.cut(cyl_z(s.hole_d / 2, xi, yi, s.ear_z0 - 1, s.ear_z1 + 1))
            holes.append(Hole(f"ear_{xi:.1f}_{yi:.1f}", (xi, yi, s.ear_z0), (0, 0, 1), s.hole_d,
                              kind="ear", fastener=s.mount_screw))
    return Part(name, body, material="vendor", vendor=True, mass_g=s.mass, holes=holes,
                descr=f"{s.name} servo, {s.stall_torque:.2f} N·m stall @6 V",
                price_usd=s.price_usd, supplier="AliExpress / Amazon (DSSERVO)", rgba=BLACK,
                subsystem="actuation")


def horn(spec: P.ServoSpec, name: str) -> Part:
    s = spec
    hub = cyl_z(s.horn_hub_d / 2, 0, 0, s.boss_h, s.horn_top - s.horn_t)
    disc = cyl_z(s.horn_d / 2, 0, 0, s.horn_top - s.horn_t, s.horn_top)
    body = hub.union(disc)
    holes = []
    for i, (x, y) in enumerate(bolt_circle(s.horn_pcd, 4, 45.0)):
        body = body.cut(cyl_z(s.horn_hole_d / 2 * 0.85, x, y, s.horn_top - s.horn_t - 1, s.horn_top + 1))
        holes.append(Hole(f"pcd{i}", (x, y, s.horn_top), (0, 0, 1), s.horn_hole_d, kind="thread"))
    body = body.cut(cyl_z(1.6, 0, 0, s.boss_h - 1, s.horn_top + 1))
    return Part(name, body, material="vendor", vendor=True, mass_g=s.horn_mass, holes=holes,
                descr=("25T aluminium disc horn Ø25" if s is P.DS3218 else "MG90S round horn Ø20"),
                price_usd=(1.5 if s is P.DS3218 else 0.0),
                supplier="included w/ servo" if s is P.MG90S else "AliExpress (25T alu horn)",
                rgba=ALU, subsystem="actuation")


def bearing(b: dict, name: str) -> Part:
    """Ball bearing, axis +Z, face at z=0..B."""
    ring = tube(b["D"] / 2, b["d"] / 2, (0, 0, 0), (0, 0, b["B"]))
    return Part(name, ring, material="vendor", vendor=True, mass_g=b["mass"],
                descr=f"{b['name']} ball bearing {b['d']:.0f}x{b['D']:.0f}x{b['B']:.0f}",
                price_usd=b["price_usd"], supplier="AliExpress / bearing shop", rgba=STEEL,
                subsystem="mechanical")


def battery() -> Part:
    L, W, H = P.BATTERY["dims"]
    b = box(0, L, -W / 2, W / 2, 0, H)
    return Part("battery_2s_2200", b, material="vendor", vendor=True, mass_g=P.BATTERY["mass"],
                descr=P.BATTERY["name"], price_usd=P.BATTERY["price_usd"], supplier="hobby shop",
                rgba=LIPO, subsystem="power")


def board(name: str, spec: dict, rgba=PCB_G, comp_h: float | None = None) -> Part:
    L, W, H = spec["dims"]
    pcb = box(0, L, 0, W, 0, 1.6)
    if comp_h is None:
        comp_h = H - 1.6
    comps = box(L * 0.1, L * 0.9, W * 0.15, W * 0.85, 1.6, 1.6 + comp_h)
    return Part(name, pcb.union(comps), material="vendor", vendor=True, mass_g=spec["mass"],
                descr=spec["name"], price_usd=spec["price_usd"], supplier="AliExpress / Amazon",
                rgba=rgba, subsystem="electronics")


def pump() -> Part:
    # Head 32 (x) x 28 (y) x 32 (z) at origin corner; motor Ø24 along -y.
    head = box(0, 32, 0, 28, 0, 30)
    motor = cyl_y(12.0, 16.0, 15.0, -28.0, 0.0)
    barb_in = cyl_x(2.0, 9.0, 24.0, 32.0, 40.0)
    barb_out = cyl_x(2.0, 19.0, 24.0, 32.0, 40.0)
    feet = box(-4, 36, 4, 24, 0, 2)
    body = union_all([head, motor, barb_in, barb_out, feet])
    holes = []  # mounted with VHB foam tape on the tank lid
    return Part("pump_peristaltic", body, material="vendor", vendor=True, mass_g=P.PUMP["mass"],
                descr=P.PUMP["name"] + f", {P.PUMP['flow_ml_min']:.0f} ml/min",
                price_usd=P.PUMP["price_usd"], supplier="AliExpress (Kamoer-style NKP)",
                rgba=(0.85, 0.85, 0.88, 1.0), holes=holes, subsystem="water")


def ubec() -> Part:
    L, W, H = P.UBEC["dims"]
    return Part("ubec_6v_10a", box(0, L, 0, W, 0, H), material="vendor", vendor=True,
                mass_g=P.UBEC["mass"], descr=P.UBEC["name"], price_usd=P.UBEC["price_usd"],
                supplier="hobby shop", rgba=(0.15, 0.15, 0.15, 1.0), subsystem="power")


def rocker_switch() -> Part:
    flange = box(0, 2, -7.5, 7.5, -10.5, 10.5)
    body = box(-18, 0, -6.5, 6.5, -9.5, 9.5)
    return Part("rocker_switch", flange.union(body), material="vendor", vendor=True, mass_g=4.0,
                descr="KCD1 rocker switch 10 A (13x19 mm cut-out)", price_usd=0.5,
                supplier="AliExpress", rgba=BLACK, subsystem="power")


def xt60_panel() -> Part:
    plate = box(0, 3, -12.5, 12.5, -8, 8)
    body = box(-14, 0, -8, 8, -4.2, 4.2)
    return Part("xt60_panel_mount", plate.union(body), material="vendor", vendor=True, mass_g=6.0,
                descr="XT60 panel-mount female (charge port)", price_usd=1.5,
                supplier="hobby shop", rgba=(0.9, 0.8, 0.1, 1.0), subsystem="power")


def steel_rod(L: float, d: float, name: str) -> Part:
    r = cyl_y(d / 2, 0, 0, -L / 2, L / 2)
    return Part(name, r, material="vendor", vendor=True, mass_g=7.85e-3 * np.pi * (d / 2) ** 2 * L,
                descr=f"Ø{d:.0f} x {L:.0f} mm ground steel rod (cut from 3 mm silver steel)",
                price_usd=0.3, supplier="hardware store", rgba=STEEL, subsystem="arm")


def nozzle() -> Part:
    # Axis along -Z (tool direction), tip at z=0.
    body = cyl_z(2.0, 0, 0, 0.0, 14.0).union(cyl_z(1.4, 0, 0, 14.0, 22.0))
    body = body.cut(cyl_z(P.NOZZLE_ORIFICE / 2, 0, 0, -1, 23))
    return Part("nozzle_brass", body, material="vendor", vendor=True, mass_g=2.0,
                descr=f"Brass dispensing tip, Ø{P.NOZZLE_ORIFICE} mm orifice, barbed for 2x4 mm tube",
                price_usd=0.8, supplier="AliExpress (blunt dispensing tip)", rgba=BRASS, subsystem="water")


def tof() -> Part:
    return board("tof_vl53l0x", P.TOF, PCB_B, comp_h=1.4)

"""Mass properties from CAD: printed-part mass model, link inertials, CoG."""
from __future__ import annotations

from functools import lru_cache
from typing import Dict, Tuple

import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from . import params as P
from .cad.catalog import all_parts
from . import robot as R


@lru_cache(maxsize=None)
def _gprops(key: str):
    sh = all_parts()[key].solid().wrapped
    vp = GProp_GProps()
    BRepGProp.VolumeProperties_s(sh, vp)
    sp = GProp_GProps()
    BRepGProp.SurfaceProperties_s(sh, sp)
    V = vp.Mass()                                  # mm^3
    A = sp.Mass()                                  # mm^2
    c = vp.CentreOfMass()
    I = vp.MatrixOfInertia()                       # about CoM, density 1 (mm^5)
    Im = np.array([[I.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return V, A, np.array([c.X(), c.Y(), c.Z()]), Im


def fill_fraction(key: str) -> float:
    p = all_parts()[key]
    V, A, _, _ = _gprops(key)
    if p.material == "PETG-B":                     # water-tight parts printed solid
        return 1.0
    infill = 0.25 if p.material.startswith("TPU") else P.PRINT_INFILL
    shell = min(V, A * P.PRINT_SHELL / 2.0)        # A counts both faces of thin walls
    return float(min(1.0, (shell + infill * (V - shell)) / V))


@lru_cache(maxsize=None)
def part_mass_props(key: str) -> Tuple[float, np.ndarray, np.ndarray]:
    """(mass g, CoM mm (part frame), inertia g·mm² about CoM (part axes))."""
    p = all_parts()[key]
    V, A, c, Iv = _gprops(key)
    if p.vendor:
        m = float(p.mass_g)
    else:
        rho = P.MATERIALS[p.material]["density"] * 1e-3      # g/mm^3
        m = rho * V * fill_fraction(key)
    I = Iv * (m / V)
    return m, c, I


def instance_props(key: str, M: np.ndarray):
    m, c, I = part_mass_props(key)
    Rm = M[:3, :3]
    cw = (M @ np.array([*c, 1.0]))[:3]
    return m, cw, Rm @ I @ Rm.T


def combine(items):
    """items: [(m, c, I_about_c)] in one frame -> (m, c, I_about_c)."""
    m = sum(i[0] for i in items)
    if m <= 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    c = sum(i[0] * i[1] for i in items) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in items:
        d = ci - c
        I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, c, I


@lru_cache(maxsize=1)
def link_inertials() -> Dict[str, Tuple[float, np.ndarray, np.ndarray]]:
    """Per link (g, mm, g·mm²) in the link frame; includes estimated fasteners & wiring."""
    out = {}
    for name, l in R.build_tree().items():
        items = [instance_props(it.part, it.T) for it in l.instances]
        m, c, I = combine(items)
        out[name] = (m, c, I)
    # lump fasteners / wiring / tubing on the base (estimated, see BOM)
    m, c, I = out["base"]
    extra = fastener_wiring_mass()
    out["base"] = combine([(m, c, I), (extra, np.array([0.0, 0.0, 10.0]), np.eye(3) * extra * 40.0 ** 2)])
    return out


def fastener_wiring_mass() -> float:
    from .bom import fastener_counts
    fc = fastener_counts()
    per = {"M3": 1.2, "M5": 4.5, "M2": 0.3, "insert": 0.4, "nut": 0.8}
    m = 0.0
    for k, n in fc.items():
        if "M5" in k: m += per["M5"] * n
        elif "insert" in k: m += per["insert"] * n
        elif "M2" in k: m += per["M2"] * n
        else: m += per["M3"] * n
    m += 85.0       # servo extension leads, power wiring, silicone tube, velcro
    return m


def water_mass(fill: float = 1.0) -> Tuple[float, np.ndarray]:
    T = P.TANK
    cx = (T["x0"] + T["x1"]) / 2
    z0 = T["z0"] + T["floor"]
    h = (T["z1"] - z0) * fill
    return P.TANK_TARGET_ML * fill, np.array([cx, 0.0, z0 + h / 2])


def total_mass(water_fill: float = 1.0) -> float:
    return sum(v[0] for v in link_inertials().values()) + water_mass(water_fill)[0]


def cog(q=None, water_fill=1.0, payload_g=0.0) -> Tuple[float, np.ndarray]:
    lw = R.link_world(q or {})
    items = []
    for name, (m, c, I) in link_inertials().items():
        items.append((m, (lw[name] @ np.array([*c, 1.0]))[:3], I))
    mw, cw = water_mass(water_fill)
    if mw > 0:
        items.append((mw, cw, np.zeros((3, 3))))
    if payload_g > 0:
        items.append((payload_g, R.tcp_point(q or {}), np.zeros((3, 3))))
    m, c, _ = combine(items)
    return m, c

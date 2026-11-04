"""Small, robust CadQuery helpers + part metadata containers."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cadquery as cq
import numpy as np

Vec = Tuple[float, float, float]


# ---------------------------------------------------------------- primitives
def box(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    if x1 < x0: x0, x1 = x1, x0
    if y1 < y0: y0, y1 = y1, y0
    if z1 < z0: z0, z1 = z1, z0
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0)
            .translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))


def cyl(r: float, p0: Sequence[float], p1: Sequence[float]) -> cq.Workplane:
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    d = p1 - p0
    h = float(np.linalg.norm(d))
    s = cq.Solid.makeCylinder(r, h, cq.Vector(*p0), cq.Vector(*(d / h)))
    return cq.Workplane(obj=s)


def tube(r_out: float, r_in: float, p0, p1) -> cq.Workplane:
    return cyl(r_out, p0, p1).cut(cyl(r_in, p0, p1))


def cyl_y(r, x, z, y0, y1):
    return cyl(r, (x, y0, z), (x, y1, z))


def cyl_x(r, y, z, x0, x1):
    return cyl(r, (x0, y, z), (x1, y, z))


def cyl_z(r, x, y, z0, z1):
    return cyl(r, (x, y, z0), (x, y, z1))


def sphere(r, c):
    return cq.Workplane(obj=cq.Solid.makeSphere(r, cq.Vector(*c), angleDegrees1=-90, angleDegrees2=90))


def union_all(items: List[cq.Workplane]) -> cq.Workplane:
    out = items[0]
    for it in items[1:]:
        out = out.union(it)
    return out


def cut_all(base: cq.Workplane, items: List[cq.Workplane]) -> cq.Workplane:
    for it in items:
        base = base.cut(it)
    return base


def slot_xz_profile_y(pts_xz: Sequence[Tuple[float, float]], y0: float, y1: float) -> cq.Workplane:
    """Extrude a polygon given in the XZ plane between y0 and y1."""
    wp = cq.Workplane("XZ", origin=(0, y0, 0)).polyline(list(pts_xz)).close().extrude(-(y1 - y0))
    return wp


def poly_xy_extrude_z(pts_xy, z0, z1) -> cq.Workplane:
    return cq.Workplane("XY", origin=(0, 0, z0)).polyline(list(pts_xy)).close().extrude(z1 - z0)


def poly_yz_extrude_x(pts_yz, x0, x1) -> cq.Workplane:
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(list(pts_yz)).close().extrude(x1 - x0)


# ---------------------------------------------------------------- metadata
@dataclass
class Hole:
    """A hole or fastener interface, expressed in the part's local frame."""
    name: str
    p: Vec                    # a point on the axis (at the mating face)
    d: Vec                    # unit axis direction
    dia: float
    kind: str = "clearance"   # clearance | insert | thread | bearing | horn | ear
    fastener: Optional[str] = None   # counted in the BOM (one side of a joint only)
    extra: Optional[str] = None      # e.g. "M3 heat-set insert", counted in BOM
    mate: bool = True                # QA requires a coaxial partner hole


@dataclass
class Part:
    name: str
    shape: cq.Workplane
    material: str = "PETG"
    vendor: bool = False
    mass_g: Optional[float] = None       # vendor mass (printed parts are computed)
    holes: List[Hole] = field(default_factory=list)
    descr: str = ""
    print_note: str = ""
    price_usd: float = 0.0
    supplier: str = ""
    rgba: Optional[Tuple[float, float, float, float]] = None
    mirror_of: Optional[str] = None
    subsystem: str = ""

    def solid(self) -> cq.Shape:
        return self.shape.val() if len(self.shape.vals()) == 1 else self.shape.combine().val()


def make_holes_y(name, pts_xz, y, dia, direction=+1, **kw) -> List[Hole]:
    return [Hole(f"{name}{i}", (x, y, z), (0.0, float(direction), 0.0), dia, **kw)
            for i, (x, z) in enumerate(pts_xz)]


def bolt_circle(pcd: float, n: int = 4, phase_deg: float = 45.0):
    a = np.radians(phase_deg) + np.arange(n) * 2 * np.pi / n
    return [(pcd / 2 * np.cos(t), pcd / 2 * np.sin(t)) for t in a]

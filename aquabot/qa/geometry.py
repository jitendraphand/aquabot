"""Geometric QA primitives: exact OCC interference, mesh sweeps, hole mating."""
from __future__ import annotations

import itertools
from functools import lru_cache
from typing import Dict, Iterable, List, Sequence, Tuple

import cadquery as cq
import numpy as np
import trimesh
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from ..cad.catalog import all_parts
from ..cad.export import tessellate
from .. import robot as R

# Pairs that are *meant* to touch (zero-clearance mating or gear mesh).
ALLOWED_SUBSTR = [
    ("horn_", "servo_"),          # horn hub seats on its servo output boss
    ("gripper_pinion", "gripper_jaw"),   # gear mesh (checked separately by backlash)
    ("gripper_pinion", "horn_gripper"),
    ("arm_turntable", "bearing_yaw"),    # yaw bearing outer race carries the turntable
]


def allowed(a: str, b: str) -> bool:
    for x, y in ALLOWED_SUBSTR:
        if (x in a and y in b) or (x in b and y in a):
            # horn/servo pairs must belong to the same joint
            if x == "horn_" and y == "servo_":
                ja = a.replace("horn_", "").replace("servo_", "")
                jb = b.replace("horn_", "").replace("servo_", "")
                if ja == jb:
                    return True
                continue
            return True
    return False


def _trsf(M: np.ndarray):
    from OCP.gp import gp_Trsf
    U, _, Vt = np.linalg.svd(M[:3, :3])
    Rm = U @ Vt
    t = gp_Trsf()
    t.SetValues(Rm[0, 0], Rm[0, 1], Rm[0, 2], M[0, 3],
                Rm[1, 0], Rm[1, 1], Rm[1, 2], M[1, 3],
                Rm[2, 0], Rm[2, 1], Rm[2, 2], M[2, 3])
    return t


@lru_cache(maxsize=None)
def _solid(key: str):
    return all_parts()[key].solid()


def world_shape(key: str, M: np.ndarray):
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    sh = BRepBuilderAPI_Transform(_solid(key).wrapped, _trsf(M), True).Shape()
    return cq.Shape.cast(sh)


def volume(shape) -> float:
    pr = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped if hasattr(shape, "wrapped") else shape, pr)
    return pr.Mass()


def bbox_overlap(a, b, tol=0.0) -> bool:
    A, B = a.BoundingBox(), b.BoundingBox()
    return not (A.xmax < B.xmin - tol or B.xmax < A.xmin - tol or A.ymax < B.ymin - tol or
                B.ymax < A.ymin - tol or A.zmax < B.zmin - tol or B.zmax < A.zmin - tol)


def interference(instances: Sequence[Tuple[str, str, np.ndarray]], thresh: float = 0.5,
                 only_pairs: Iterable[Tuple[str, str]] | None = None) -> List[dict]:
    """Exact pairwise interference volume (mm^3) between placed solids."""
    shapes = {n: world_shape(k, M) for (n, k, M) in instances}
    names = list(shapes)
    pairs = only_pairs if only_pairs is not None else itertools.combinations(names, 2)
    hits = []
    for a, b in pairs:
        if allowed(a, b):
            continue
        sa, sb = shapes[a], shapes[b]
        if not bbox_overlap(sa, sb):
            continue
        op = BRepAlgoAPI_Common(sa.wrapped, sb.wrapped)
        op.Build()
        if not op.IsDone():
            hits.append(dict(a=a, b=b, vol=float("nan"), note="boolean failed"))
            continue
        v = volume(op.Shape())
        if v > thresh:
            hits.append(dict(a=a, b=b, vol=round(v, 3)))
    return hits


# ------------------------------------------------------------- mesh sweeps
@lru_cache(maxsize=None)
def _mesh(key: str) -> trimesh.Trimesh:
    return tessellate(all_parts()[key], 0.15, 0.3)


class Sweeper:
    """FCL-based collision checks of moving links over joint grids."""

    def __init__(self, instance_filter=None):
        import fcl  # noqa: F401
        self.cm = trimesh.collision.CollisionManager()
        self.meta = {}
        inst = R.world_instances({})
        for (n, k, M, link) in inst:
            if instance_filter and not instance_filter(n, link):
                continue
            self.cm.add_object(n, _mesh(k), transform=self._m(M))
            self.meta[n] = (k, link)

    @staticmethod
    def _m(M):
        X = M.copy()
        return X

    def set_pose(self, q: Dict[str, float], base_T=None):
        lw = R.link_world(q, base_T)
        tree = R.build_tree()
        for lname, l in tree.items():
            for it in l.instances:
                if it.name in self.meta:
                    self.cm.set_transform(it.name, lw[lname] @ it.T)

    def collisions(self, skip_same_link=True) -> List[Tuple[str, str]]:
        hit, names = self.cm.in_collision_internal(return_names=True)
        out = []
        if not hit:
            return out
        for a, b in names:
            if allowed(a, b):
                continue
            if skip_same_link and self.meta[a][1] == self.meta[b][1]:
                continue
            out.append(tuple(sorted((a, b))))
        return sorted(set(out))


# ----------------------------------------------------------- hole mating
def world_holes(instances) -> List[dict]:
    parts = all_parts()
    out = []
    for (n, k, M, *_) in instances:
        for h in parts[k].holes:
            p = (M @ np.array([*h.p, 1.0]))[:3]
            d = M[:3, :3] @ np.asarray(h.d, float)
            out.append(dict(inst=n, part=k, hole=h, p=p, d=d / np.linalg.norm(d)))
    return out


def hole_mating(instances, ang_tol_deg=0.5, off_tol=0.15) -> Tuple[List[dict], int]:
    """Every hole with mate=True must have a coaxial partner hole in another part."""
    H = world_holes(instances)
    unmatched, matched = [], 0
    cos_tol = np.cos(np.radians(ang_tol_deg))
    P = np.array([h["p"] for h in H])
    for i, h in enumerate(H):
        if not h["hole"].mate:
            continue
        ok = False
        dists = np.linalg.norm(P - h["p"], axis=1)
        for j in np.argsort(dists)[:40]:
            if j == i or H[j]["inst"] == h["inst"]:
                continue
            g = H[j]
            if abs(np.dot(h["d"], g["d"])) < cos_tol:
                continue
            v = g["p"] - h["p"]
            perp = v - np.dot(v, h["d"]) * h["d"]
            if np.linalg.norm(perp) <= off_tol and abs(np.dot(v, h["d"])) < 60.0:
                ok = True
                break
        if ok:
            matched += 1
        else:
            unmatched.append(dict(inst=h["inst"], hole=h["hole"].name, p=np.round(h["p"], 2).tolist()))
    return unmatched, matched

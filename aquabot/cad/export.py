"""Exports: STEP, STL, tessellated meshes and SVG projections."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Tuple

import cadquery as cq
import numpy as np
import trimesh

from .geom import Part

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build"
MESH_DIR = BUILD / "mesh"


def tessellate(part: Part, tol: float = 0.08, ang: float = 0.25) -> trimesh.Trimesh:
    shape = part.solid()
    verts, tris = shape.tessellate(tol, ang)
    v = np.array([(p.x, p.y, p.z) for p in verts], float)
    f = np.array(tris, int)
    m = trimesh.Trimesh(v, f, process=True)
    return m


def sim_mesh_path(key: str, part: Part, tol: float = 0.25) -> Path:
    """Coarse STL for simulation/rendering, cached in build/mesh."""
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    p = MESH_DIR / f"{key}.stl"
    if not p.exists():
        m = tessellate(part, tol, 0.35)
        m.export(p)
    return p


def export_step(part: Part, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(part.shape, str(path), exportType="STEP")


def export_stl(part: Part, path: Path, tol: float = 0.02, ang: float = 0.1):
    """High-resolution STL; degenerate triangles (e.g. at sphere poles) are removed so every
    exported mesh is a closed 2-manifold (checked by QA P-GEO-03)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(part.shape, str(path), exportType="STL", tolerance=tol, angularTolerance=ang)
    m = trimesh.load(path, force="mesh")
    if not m.is_watertight:
        m.update_faces(m.nondegenerate_faces())
        m.remove_unreferenced_vertices()
        m.merge_vertices()
        m.export(path)


def export_svg(part: Part, path: Path, proj: Tuple[float, float, float] = (1, -1, 0.7), w=600, h=450):
    path.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(part.shape, str(path), exportType="SVG",
                        opt={"width": w, "height": h, "marginLeft": 20, "marginTop": 20,
                             "showAxes": False, "projectionDir": proj, "strokeWidth": 0.25,
                             "strokeColor": (30, 30, 30), "hiddenColor": (170, 170, 170),
                             "showHidden": True})

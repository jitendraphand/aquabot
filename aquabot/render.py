"""Static CAD renders through MuJoCo's offscreen renderer (EGL)."""
from __future__ import annotations

import os
os.environ.setdefault("MUJOCO_GL", "egl")

from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import numpy as np

from .cad.catalog import all_parts
from .cad.export import sim_mesh_path
from .robot import rot_to_quat

SCENE_HEADER = """
<visual>
  <global offwidth="{W}" offheight="{H}" fovy="30"/>
  <quality shadowsize="4096" offsamples="8"/>
  <headlight ambient="0.35 0.35 0.35" diffuse="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
  <map znear="0.002" zfar="20"/>
</visual>
<asset>
  <texture name="sky" type="skybox" builtin="gradient" rgb1="0.97 0.98 1.0" rgb2="0.80 0.86 0.93" width="64" height="64"/>
  <texture name="grid" type="2d" builtin="checker" rgb1="0.90 0.91 0.92" rgb2="0.84 0.85 0.87" width="300" height="300"/>
  <material name="grid" texture="grid" texrepeat="12 12" reflectance="0.05"/>
  {meshes}
</asset>
"""


def _mesh_assets(keys: Iterable[str]) -> str:
    parts = all_parts()
    out = []
    for k in sorted(set(keys)):
        p = sim_mesh_path(k, parts[k])
        out.append(f'<mesh name="m_{k}" file="{p}" scale="0.001 0.001 0.001"/>')
    return "\n  ".join(out)


def geom_xml(part_key: str, M: np.ndarray, rgba=None, name=None, alpha=1.0) -> str:
    parts = all_parts()
    c = rgba or parts[part_key].rgba
    pos = M[:3, 3] / 1000.0
    q = rot_to_quat(M[:3, :3])
    nm = f' name="{name}"' if name else ""
    return (f'<geom{nm} type="mesh" mesh="m_{part_key}" pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}" '
            f'quat="{q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f}" rgba="{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {c[3]*alpha:.3f}" '
            f'contype="0" conaffinity="0"/>')


def render_static(instances: Sequence[Tuple[str, str, np.ndarray]], out: Path, views: List[dict],
                  W: int = 1600, H: int = 1000, floor_z: float | None = None, extra: str = "") -> List[Path]:
    import mujoco
    from PIL import Image
    keys = [k for (_, k, _) in instances]
    geoms = "\n    ".join(geom_xml(k, M) for (_, k, M) in instances)
    floor = ""
    if floor_z is not None:
        floor = f'<geom type="plane" size="3 3 0.1" pos="0 0 {floor_z/1000.0:.4f}" material="grid"/>'
    xml = f"""<mujoco model="aquabot_render">
{SCENE_HEADER.format(W=W, H=H, meshes=_mesh_assets(keys))}
<worldbody>
    <light pos="0.6 -0.8 1.6" dir="-0.4 0.5 -1" diffuse="0.55 0.55 0.55" castshadow="true"/>
    <light pos="-0.8 0.6 1.2" dir="0.5 -0.4 -1" diffuse="0.25 0.25 0.28" castshadow="false"/>
    {floor}
    {geoms}
    {extra}
</worldbody>
</mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    r = mujoco.Renderer(model, H, W)
    paths = []
    out.parent.mkdir(parents=True, exist_ok=True)
    for i, v in enumerate(views):
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = np.asarray(v.get("lookat", (0, 0, 0)), float) / 1000.0
        cam.distance = v.get("distance", 700) / 1000.0
        cam.azimuth = v.get("azimuth", 135)
        cam.elevation = v.get("elevation", -25)
        r.update_scene(data, cam)
        img = r.render()
        p = out.with_name(out.stem + (f"_{v['name']}" if "name" in v else f"_{i}") + ".png")
        Image.fromarray(img).save(p)
        paths.append(p)
    r.close()
    return paths

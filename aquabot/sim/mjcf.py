"""Generate the MuJoCo model (MJCF) from the link tree + CAD mass properties."""
from __future__ import annotations

import shutil
from pathlib import Path
from typing import Dict, List

import numpy as np

from .. import params as P
from .. import robot as R
from ..cad.catalog import all_parts
from ..cad.export import sim_mesh_path
from ..massprops import link_inertials, water_mass
from ..cad import parts_arm as PA

ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "sim" / "models"
MESH_OUT = MODEL_DIR / "meshes"

KP_DS, KV_DS = 32.0, 0.55
KP_MG, KV_MG = 4.0, 0.06
ARMATURE_DS, ARMATURE_MG = 0.0045, 0.0006


def f(v, n=6):
    return " ".join(f"{x:.{n}f}" for x in np.ravel(v))


def _inertial(m_g, c_mm, I_gmm2) -> str:
    m = max(m_g, 1e-3) / 1000.0
    c = np.asarray(c_mm) / 1000.0
    I = np.asarray(I_gmm2) * 1e-9
    # enforce symmetric positive definite
    I = 0.5 * (I + I.T)
    w, V = np.linalg.eigh(I)
    w = np.maximum(w, 1e-8)
    I = V @ np.diag(w) @ V.T
    return (f'<inertial pos="{f(c)}" mass="{m:.6f}" '
            f'fullinertia="{I[0,0]:.4e} {I[1,1]:.4e} {I[2,2]:.4e} {I[0,1]:.4e} {I[0,2]:.4e} {I[1,2]:.4e}"/>')


def _vis_geoms(link: R.Link, parts) -> List[str]:
    out = []
    for it in link.instances:
        M = it.T
        q = R.rot_to_quat(M[:3, :3])
        c = parts[it.part].rgba
        out.append(f'<geom class="vis" mesh="{it.part}" pos="{f(M[:3,3]/1000.0)}" quat="{f(q,7)}" '
                   f'rgba="{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {c[3]:.3f}"/>')
    return out


def _col_geoms(name: str) -> List[str]:
    g = []
    if name == "base":
        g.append('<geom class="col" type="box" size="0.095 0.046 0.045" pos="0 0 -0.005"/>')
        g.append('<geom class="col" type="box" size="0.035 0.046 0.013" pos="0.0 0 0.053"/>')
    elif name.endswith("_tibia"):
        g.append(f'<geom class="foot" name="{name[:2]}_foot" type="sphere" size="{P.FOOT_R/1000:.4f}" '
                 f'pos="0 0 {-P.L_TIBIA/1000:.4f}"/>')
        g.append('<geom class="col" type="capsule" size="0.009" fromto="0 0 -0.032 0 0 -0.082"/>')
    elif name.endswith("_femur"):
        g.append(f'<geom class="col" type="capsule" size="0.015" fromto="0 0 -0.030 0 0 {-(P.L_FEMUR-12)/1000:.4f}"/>')
    elif name in ("jaw_a", "jaw_b"):
        g0 = PA.GR["closed_gap"] / 2
        z0, z1 = PA.GR["finger"]
        g.append(f'<geom class="finger" name="{name}_pad" type="box" size="0.011 0.0020 {(z1-z0)/2000:.4f}" '
                 f'pos="0 {-(g0+2.0)/1000:.4f} {(z0+z1)/2000:.4f}"/>')
    return g


def build_robot_xml(base_pos=(0, 0, 0.0), base_quat=(1, 0, 0, 0), parts=None) -> Dict[str, str]:
    parts = parts or all_parts()
    tree = R.build_tree()
    inert = link_inertials()
    children: Dict[str, List[str]] = {}
    for n, l in tree.items():
        if l.parent:
            children.setdefault(l.parent, []).append(n)

    def body(n: str, indent: str) -> List[str]:
        l = tree[n]
        if n == "base":
            head = f'{indent}<body name="base" pos="{f(base_pos)}" quat="{f(base_quat)}" childclass="aquabot">'
            lines = [head, f'{indent}  <freejoint name="root"/>']
        else:
            M = l.origin
            lines = [f'{indent}<body name="{n}" pos="{f(M[:3,3]/1000.0)}" quat="{f(R.rot_to_quat(M[:3,:3]),7)}">']
        m, c, I = inert[n]
        lines.append(f"{indent}  {_inertial(m, c, I)}")
        if l.joint:
            j = l.joint
            cls = "ds" if j.actuator == "DS3218" else "mg"
            if j.type == "revolute":
                lines.append(f'{indent}  <joint name="{j.name}" class="{cls}" type="hinge" axis="{f(j.axis,1)}" '
                             f'range="{j.lower:.5f} {j.upper:.5f}"/>')
            else:
                lines.append(f'{indent}  <joint name="{j.name}" class="slide" type="slide" axis="{f(j.axis,1)}" '
                             f'range="{j.lower/1000:.5f} {j.upper/1000:.5f}"/>')
        for g in _vis_geoms(l, parts):
            lines.append(f"{indent}  {g}")
        for g in _col_geoms(n):
            lines.append(f"{indent}  {g}")
        if n == "base":
            mw, cw = water_mass(1.0)
            lines.append(f'{indent}  <body name="water" pos="{f(cw/1000.0)}">')
            lines.append(f'{indent}    <inertial pos="0 0 0" mass="{mw/1000:.4f}" diaginertia="2e-4 4e-4 5e-4"/>')
            lines.append(f'{indent}  </body>')
            lines.append(f'{indent}  <site name="imu" pos="-0.0155 0.030 0.006" size="0.004"/>')
            lines.append(f'{indent}  <camera name="onboard" pos="0.097 0 -0.012" xyaxes="0 -1 0 0.2 0 1" fovy="70"/>')
        if n.endswith("_tibia"):
            lines.append(f'{indent}  <site name="{n[:2]}_foot_site" pos="0 0 {-P.L_TIBIA/1000:.4f}" size="0.014"/>')
        if n == "arm_wrist":
            lines.append(f'{indent}  <site name="tcp" pos="0 0 {-P.L_TCP/1000:.4f}" size="0.004" rgba="1 0 0 0.5"/>')
            lines.append(f'{indent}  <site name="nozzle_tip" pos="{PA.GR["nozzle_x"]/1000:.4f} 0 {-P.NOZZLE_OFF[0]/1000:.4f}" '
                         f'size="0.003" rgba="0 0.4 1 0.6"/>')
        for ch in children.get(n, []):
            lines += body(ch, indent + "  ")
        lines.append(f"{indent}</body>")
        return lines

    acts = []
    for n, l in tree.items():
        j = l.joint
        if not j or j.mimic:
            continue
        cls = ("ds" if j.actuator == "DS3218" else "mg") if j.type == "revolute" else "slide"
        acts.append(f'<motor name="{j.name}" class="{cls}" joint="{j.name}"/>')
    eq = [f'<joint joint1="grip_b" joint2="grip_a" polycoef="0 1 0 0 0" solref="0.004 1"/>',
          f'<joint joint1="grip_pinion" joint2="grip_a" polycoef="0 {-1000.0/PA.R_PITCH:.3f} 0 0 0" solref="0.004 1"/>']
    sensors = ['<accelerometer name="imu_acc" site="imu"/>', '<gyro name="imu_gyro" site="imu"/>',
               '<framequat name="imu_quat" objtype="site" objname="imu"/>']
    for leg in P.LEGS:
        sensors.append(f'<touch name="{leg}_contact" site="{leg}_foot_site"/>')
    for n, l in tree.items():
        if l.joint and not l.joint.mimic and l.joint.type == "revolute":
            sensors.append(f'<actuatorfrc name="{l.joint.name}_tau" actuator="{l.joint.name}"/>')
    return dict(body="\n".join(body("base", "    ")), actuators="\n    ".join(acts),
                equality="\n    ".join(eq), sensors="\n    ".join(sensors))


def defaults_xml() -> str:
    S, M = P.DS3218, P.MG90S
    return f"""
  <default>
    <default class="aquabot">
      <geom contype="0" conaffinity="0"/>
      <default class="vis"><geom type="mesh" group="1" contype="0" conaffinity="0" density="0"/></default>
      <default class="col"><geom group="3" contype="2" conaffinity="1" rgba="0.9 0.2 0.2 0.3" density="0"
                                  friction="0.6 0.01 0.001"/></default>
      <default class="foot"><geom group="3" contype="2" conaffinity="1" rgba="0.1 0.1 0.1 0.4" density="0"
                                   friction="1.1 0.02 0.002" condim="3" solref="0.006 1" solimp="0.92 0.97 0.001"/></default>
      <default class="finger"><geom group="3" contype="2" conaffinity="1" rgba="0.1 0.1 0.1 0.5" density="0"
                                     friction="1.6 0.02 0.002" condim="4" solref="0.004 1"/></default>
      <default class="ds"><joint damping="0.04" armature="{ARMATURE_DS}" frictionloss="0.03"/>
        <motor ctrlrange="{-S.stall_torque} {S.stall_torque}" ctrllimited="true"/></default>
      <default class="mg"><joint damping="0.008" armature="{ARMATURE_MG}" frictionloss="0.004"/>
        <motor ctrlrange="{-M.stall_torque} {M.stall_torque}" ctrllimited="true"/></default>
      <default class="slide"><joint damping="2.0" armature="0.02"/>
        <motor ctrlrange="-18 18" ctrllimited="true"/></default>
    </default>
  </default>"""


def assets_xml() -> str:
    parts = all_parts()
    used = sorted({it.part for l in R.build_tree().values() for it in l.instances})
    lines = []
    for k in used:
        lines.append(f'<mesh name="{k}" file="{k}.stl" scale="0.001 0.001 0.001"/>')
    return "\n    ".join(lines)


def export_meshes():
    MESH_OUT.mkdir(parents=True, exist_ok=True)
    parts = all_parts()
    used = sorted({it.part for l in R.build_tree().values() for it in l.instances})
    for k in used:
        src = sim_mesh_path(k, parts[k])
        shutil.copy(src, MESH_OUT / f"{k}.stl")


SCENE_TMPL = """<mujoco model="aquabot{suffix}">
  <compiler angle="radian" meshdir="{meshdir}" autolimits="true" inertiafromgeom="false"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="4">
    <flag multiccd="enable"/>
  </option>
  <size memory="64M"/>
  <visual>
    <global offwidth="1920" offheight="1080" fovy="40"/>
    <quality shadowsize="4096" offsamples="8"/>
    <headlight ambient="0.32 0.32 0.32" diffuse="0.5 0.5 0.5" specular="0.05 0.05 0.05"/>
    <map znear="0.005" zfar="30" haze="0.15"/>
    <rgba haze="0.82 0.88 0.95 1"/>
  </visual>
{defaults}
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.75 0.86 0.98" rgb2="0.98 0.99 1.0" width="128" height="128"/>
    <texture name="ground" type="2d" builtin="checker" rgb1="0.52 0.42 0.30" rgb2="0.48 0.38 0.27" width="512" height="512" mark="random" markrgb="0.42 0.55 0.28" random="0.04"/>
    <material name="ground" texture="ground" texrepeat="30 30" reflectance="0.0"/>
    <material name="terracotta" rgba="0.72 0.36 0.22 1"/>
    <material name="soil" rgba="0.30 0.21 0.13 1"/>
    <material name="soilwet" rgba="0.17 0.11 0.07 1"/>
    <material name="leaf" rgba="0.22 0.62 0.20 1"/>
    {assets}
  </asset>
  <worldbody>
    <light name="sun" pos="1.5 -2 3.5" dir="-0.35 0.5 -1" diffuse="0.75 0.75 0.72" castshadow="true"/>
    <light name="fill" pos="-2 2 2" dir="0.5 -0.5 -0.6" diffuse="0.25 0.25 0.3" castshadow="false"/>
    <geom name="floor" type="plane" size="20 20 0.1" material="ground" contype="1" conaffinity="2" friction="1.0 0.02 0.002"/>
    <camera name="side" pos="0.3 -1.25 0.42" xyaxes="1 0 0 0 0.32 0.95" fovy="40"/>
{robot}
{env}
  </worldbody>
  <equality>
    {equality}
  </equality>
  <actuator>
    {actuators}
  </actuator>
  <sensor>
    {sensors}
  </sensor>
</mujoco>
"""


def scene_xml(env: str = "", base_pos=(0, 0, P.STAND_HEIGHT / 1000.0), meshdir: str | None = None,
              suffix: str = "") -> str:
    rb = build_robot_xml(base_pos)
    return SCENE_TMPL.format(suffix=suffix, meshdir=meshdir or str(MESH_OUT), defaults=defaults_xml(),
                             assets=assets_xml(), robot=rb["body"], env=env, equality=rb["equality"],
                             actuators=rb["actuators"], sensors=rb["sensors"])


def write_model():
    """sim/models/aquabot.xml (robot on flat ground) + meshes; relative mesh dir for portability."""
    export_meshes()
    xml = scene_xml(meshdir="meshes")
    (MODEL_DIR / "aquabot.xml").write_text(xml)
    return MODEL_DIR / "aquabot.xml"

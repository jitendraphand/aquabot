"""Regenerate every derived artefact from aquabot/params.py.

  python -m scripts.build_all            # CAD exports, drawings, renders, BOM, MJCF, URDF, firmware config
  python -m scripts.build_all --no-render
"""
from __future__ import annotations

import os
os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
import json
import re
import shutil
import time
from pathlib import Path

import cadquery as cq
import numpy as np

from aquabot import params as P, robot as R
from aquabot.cad.catalog import all_parts
from aquabot.cad.export import export_step, export_stl, export_svg, BUILD
from aquabot.kinematics import legs as LK

ROOT = Path(__file__).resolve().parents[1]


def stand_q(arm=(0.0, 50.0, -115.0, -80.0)):
    q = {}
    for leg in P.LEGS:
        qq = LK.ik(leg, LK.neutral_foot(leg))
        q[f"{leg}_hip_abd"], q[f"{leg}_hip_pitch"], q[f"{leg}_knee"] = qq
    q.update(arm_yaw=np.radians(arm[0]), arm_shoulder=np.radians(arm[1]), arm_elbow=np.radians(arm[2]),
             arm_wrist=np.radians(arm[3]), grip_a=8.0)
    return q


def _loc(M):
    from aquabot.qa.geometry import _trsf
    return cq.Location(_trsf(M))


def export_parts():
    parts = all_parts()
    for k, p in parts.items():
        export_step(p, ROOT / "cad" / "step" / f"{k}.step")
        if not p.vendor:
            export_stl(p, ROOT / "cad" / "stl" / f"{k}.stl")
    return len(parts)


def export_assemblies():
    parts = all_parts()
    out = {}
    sets = {
        "aquabot_assembly": lambda n, l: True,
        "leg_FL_assembly": lambda n, l: l.startswith("FL"),
        "arm_assembly": lambda n, l: l.startswith(("arm_", "jaw_", "pinion")),
        "body_assembly": lambda n, l: l == "base",
    }
    for name, flt in sets.items():
        asm = cq.Assembly(name=name)
        for (n, k, M, link) in R.world_instances(stand_q()):
            if not flt(n, link):
                continue
            c = parts[k].rgba
            asm.add(parts[k].shape, name=n, loc=_loc(M), color=cq.Color(*c))
        path = ROOT / "cad" / "assembly" / f"{name}.step"
        path.parent.mkdir(parents=True, exist_ok=True)
        asm.save(str(path), exportType="STEP")
        out[name] = path.stat().st_size
    return out


def _svg_body(svg: str):
    m = re.search(r"<svg[^>]*>(.*)</svg>", svg, re.S)
    w = float(re.search(r'width="([\d.]+)"', svg).group(1))
    h = float(re.search(r'height="([\d.]+)"', svg).group(1))
    return m.group(1), w, h


def drawings():
    parts = all_parts()
    from aquabot.massprops import part_mass_props
    views = [("FRONT (−Y)", (0, -1, 0)), ("TOP (+Z)", (0, 0, 1)), ("RIGHT (+X)", (1, 0, 0)), ("ISO", (1, -1, 0.8))]
    tmp = BUILD / "svg"
    tmp.mkdir(parents=True, exist_ok=True)
    n = 0
    for k, p in parts.items():
        if p.vendor:
            continue
        bb = p.solid().BoundingBox()
        cells = []
        for i, (lbl, d) in enumerate(views):
            f = tmp / f"{k}_{i}.svg"
            export_svg(p, f, proj=d, w=560, h=380)
            body, w, h = _svg_body(f.read_text())
            cells.append((lbl, body, w, h))
        W, H = 1200, 920
        out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
               f'font-family="DejaVu Sans, Arial, sans-serif">',
               f'<rect width="{W}" height="{H}" fill="white"/>',
               f'<rect x="10" y="10" width="{W-20}" height="{H-20}" fill="none" stroke="#222" stroke-width="2"/>']
        for i, (lbl, body, w, h) in enumerate(cells):
            x0 = 20 + (i % 2) * 585
            y0 = 20 + (i // 2) * 385
            out.append(f'<rect x="{x0}" y="{y0}" width="575" height="375" fill="none" stroke="#bbb" stroke-width="1"/>')
            out.append(f'<g transform="translate({x0+8},{y0+4}) scale(0.98)">{body}</g>')
            out.append(f'<text x="{x0+10}" y="{y0+18}" font-size="13" font-weight="bold" fill="#345">{lbl}</text>')
        m, _, _ = part_mass_props(k)
        tb = [f"AquaBot v{P.VERSION} — {k}", p.descr, f"Material: {P.MATERIALS[p.material]['note']}",
              f"Envelope: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm   Est. mass: {m:.1f} g   Units: mm",
              f"Print: {p.print_note}", "Views auto-fitted (not to common scale). Dimensions are parametric "
              "(aquabot/params.py); the STEP file is the master geometry."]
        y = H - 112
        out.append(f'<rect x="10" y="{H-132}" width="{W-20}" height="122" fill="#f4f6f8" stroke="#222" stroke-width="1"/>')
        for j, line in enumerate(tb):
            out.append(f'<text x="24" y="{y + j*18}" font-size="{14 if j == 0 else 11.5}" '
                       f'font-weight="{"bold" if j == 0 else "normal"}" fill="#111">{_esc(line)}</text>')
        out.append("</svg>")
        (ROOT / "docs" / "drawings").mkdir(parents=True, exist_ok=True)
        (ROOT / "docs" / "drawings" / f"{k}.svg").write_text("\n".join(out))
        n += 1
    return n


def _esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def renders():
    from aquabot.render import render_static
    img = ROOT / "docs" / "images"
    q = stand_q()
    inst = [(n, k, M) for (n, k, M, l) in R.world_instances(q)]
    fz = -P.STAND_HEIGHT
    render_static(inst, img / "aquabot", [
        dict(name="hero", azimuth=140, elevation=-16, distance=1300, lookat=(0, 0, -15)),
        dict(name="side", azimuth=90, elevation=-2, distance=1300, lookat=(0, 0, -15)),
        dict(name="front", azimuth=180, elevation=-6, distance=1150, lookat=(0, 0, -15)),
        dict(name="top", azimuth=90, elevation=-88, distance=1000, lookat=(0, 0, 0)),
        dict(name="rear34", azimuth=-40, elevation=-20, distance=1250, lookat=(0, 0, -15)),
    ], W=1600, H=1000, floor_z=fz)
    qw = stand_q(arm=(90.0, 20.0, -95.0, -15.0))
    inst = [(n, k, M) for (n, k, M, l) in R.world_instances(qw)]
    render_static(inst, img / "aquabot_watering_pose", [dict(name="iso", azimuth=200, elevation=-18, distance=1100, lookat=(0, 90, -60))],
                  W=1600, H=1000, floor_z=fz)
    leg = [(n, k, M) for (n, k, M, l) in R.world_instances(q) if l.startswith("FL")]
    render_static(leg, img / "leg_FL", [dict(name="iso", azimuth=120, elevation=-15, distance=420, lookat=(140, 70, -110)),
                                         dict(name="side", azimuth=90, elevation=0, distance=420, lookat=(140, 70, -110))],
                  W=1200, H=1000)
    armi = [(n, k, M) for (n, k, M, l) in R.world_instances(stand_q(arm=(0.0, 30.0, -60.0, -30.0)))
            if l.startswith(("arm_", "jaw_", "pinion"))]
    render_static(armi, img / "arm", [dict(name="iso", azimuth=145, elevation=-18, distance=650, lookat=(170, 0, 110))],
                  W=1400, H=1000)
    grip = [(n, k, M) for (n, k, M, l) in R.world_instances(stand_q(arm=(0.0, 0.0, 0.0, -90.0)))
            if l in ("arm_wrist", "jaw_a", "jaw_b", "pinion")]
    render_static(grip, img / "gripper", [dict(name="iso", azimuth=150, elevation=-12, distance=300, lookat=(290, 0, 75))],
                  W=1200, H=1000)
    body = [(n, k, M) for (n, k, M, l) in R.world_instances(q)
            if l == "base" and k not in ("electronics_cover", "top_deck", "chassis_tub")]
    render_static(body, img / "body_internals", [dict(name="iso", azimuth=130, elevation=-35, distance=600, lookat=(0, 0, 0))],
                  W=1400, H=1000)


def gifs():
    import subprocess
    vid = ROOT / "sim" / "videos"
    img = ROOT / "docs" / "images"
    for src, dst, ss, t in (("01_crawl_walk_turn.mp4", "walk.gif", 2, 8), ("02_watering_mission.mp4", "watering.gif", 8, 9),
                            ("03_pick_and_place.mp4", "pick.gif", 4, 10), ("04_ik_showcase.mp4", "ik.gif", 1, 8)):
        if not (vid / src).exists():
            continue
        pal = BUILD / "pal.png"
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-ss", str(ss), "-t", str(t), "-i", str(vid / src),
                        "-vf", "fps=10,scale=480:-1:flags=lanczos,palettegen", str(pal)], check=True)
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-ss", str(ss), "-t", str(t), "-i", str(vid / src), "-i", str(pal),
                        "-lavfi", "fps=10,scale=480:-1:flags=lanczos[x];[x][1:v]paletteuse", str(img / dst)], check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    steps = a.only.split(",") if a.only else ["parts", "assemblies", "drawings", "bom", "mjcf", "urdf", "firmware", "renders", "gifs"]
    if a.no_render:
        steps = [s for s in steps if s not in ("renders", "gifs")]
    log = {}
    for s in steps:
        t = time.time()
        if s == "parts": log[s] = export_parts()
        elif s == "assemblies": log[s] = export_assemblies()
        elif s == "drawings": log[s] = drawings()
        elif s == "bom":
            from aquabot import bom
            log[s] = bom.write()
        elif s == "mjcf":
            from aquabot.sim import mjcf
            log[s] = str(mjcf.write_model())
        elif s == "urdf":
            from aquabot import urdf
            log[s] = str(urdf.write())
        elif s == "firmware":
            from aquabot import firmware_config
            log[s] = str(firmware_config.write())
        elif s == "renders": renders()
        elif s == "gifs": gifs()
        print(f"[build] {s:11s} {time.time()-t:6.1f}s  {log.get(s, '')}", flush=True)


if __name__ == "__main__":
    main()

"""Render all simulation videos to sim/videos/ and their metrics to sim/videos/metrics.json.

Usage: python -m scripts.render_videos [walk watering pick ik exploded]
"""
import json
import sys
import time
from pathlib import Path

from aquabot.sim import scenarios as S

ROOT = Path(__file__).resolve().parents[1]

JOBS = {
    "walk": lambda: S.walk(video=True),
    "watering": lambda: S.watering(video=True),
    "pick": lambda: S.pick_place(video=True),
    "ik": lambda: S.ik_showcase(video=True),
    "exploded": lambda: S.exploded_assembly(video=True),
}


def main(names):
    out = ROOT / "sim" / "videos" / "metrics.json"
    allm = json.loads(out.read_text()) if out.exists() else {}
    for n in names:
        t = time.time()
        m = JOBS[n]()
        m["render_wall_s"] = round(time.time() - t, 1)
        allm[n] = m
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(allm, indent=2, default=float))
        print(n, "done", m.get("render_wall_s"), flush=True)


if __name__ == "__main__":
    main(sys.argv[1:] or list(JOBS))

"""Demonstrate the QA control loop: start from an over-aggressive crawl and let the registered
fixers retune it until the simulation checks pass. Writes qa/control_loop_demo.md/json.
The project's tuned.json is backed up and restored, so the demo leaves the design unchanged."""
import json
import shutil
import time
from pathlib import Path

from aquabot import params as P
from aquabot.qa import checks as C, run as QR

ROOT = Path(__file__).resolve().parents[1]
TUNED = ROOT / "aquabot" / "tuned.json"


def main(start=dict(period=1.6, v_cmd=0.06), max_iter=10):
    backup = TUNED.read_text() if TUNED.exists() else None
    base = dict(P.CRAWL)
    log = []
    try:
        TUNED.write_text(json.dumps({"CRAWL": start}))
        P.CRAWL.update(start)
        for it in range(1, max_iter + 1):
            t = time.time()
            C.SIM_CACHE.clear()
            m = C.sim_metrics("walk")
            rs = C.sim_checks_walk(m)            # walk-only subset of the simulation checks (S-SIM-01..05)
            failing = [r for r in rs if not r.passed]
            entry = dict(iter=it, crawl=dict(period=P.CRAWL["period"], v_cmd=P.CRAWL["v_cmd"], margin=P.CRAWL["margin"]),
                         metrics={k: round(m[k], 4) for k in ("speed_m_s", "max_tilt_deg", "peak_torque_frac_50ms",
                                                              "p99_torque_frac", "turn_deg")},
                         failing=[f"{r.id} ({r.value} vs {r.limit})" for r in failing], seconds=round(time.time() - t, 1))
            fixers = sorted({r.fixer for r in failing if r.fixer})
            if not failing:
                log.append(entry)
                break
            entry["action"] = "; ".join(QR.FIXERS[f]() for f in fixers) if fixers else "no automatic fixer — escalate"
            log.append(entry)
            print(entry, flush=True)
            if not fixers:
                break
    finally:
        if backup is None:
            TUNED.unlink(missing_ok=True)
        else:
            TUNED.write_text(backup)
        P.CRAWL.clear(); P.CRAWL.update(base)
    out = ROOT / "qa"
    (out / "control_loop_demo.json").write_text(json.dumps(log, indent=2))
    L = ["# QA control-loop demonstration", "",
         "`python -m scripts.control_loop_demo` starts the crawl gait from a deliberately aggressive setting "
         f"(period {start['period']} s, commanded {start['v_cmd']} m/s) and lets the QA control loop correct it. "
         "Each iteration runs the MuJoCo walk-and-turn scenario, evaluates checks S-SIM-01…05, and applies the "
         "registered fixer of every failing check. The project's own tuned parameters are restored afterwards.", "",
         "| Iter | Crawl period (s) | v_cmd (m/s) | speed (m/s) | tilt (°) | 50 ms peak τ | p99 τ | Failing | Action |",
         "|---:|---:|---:|---:|---:|---:|---:|---|---|"]
    for e in log:
        mm = e["metrics"]
        L.append(f"| {e['iter']} | {e['crawl']['period']} | {e['crawl']['v_cmd']} | {mm['speed_m_s']} | {mm['max_tilt_deg']} | "
                 f"{mm['peak_torque_frac_50ms']} | {mm['p99_torque_frac']} | {'<br>'.join(e['failing']) or '✅ none'} | "
                 f"{e.get('action', '—')} |")
    (out / "control_loop_demo.md").write_text("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()

"""Recursive QA / QC runner.

The product is a tree (system -> subsystems -> assemblies -> parts). `evaluate()` walks it
depth-first: every child is verified before its parent's integration checks run, and a
node only passes when all of its children and its own checks pass (V-model, bottom-up).

Quality *control*: failing checks that carry a `fixer` name trigger a registered corrective
action (e.g. slow the crawl gait, widen the CoG margin). The fixer writes the tuned value
to aquabot/tuned.json, the affected subtree is re-verified, and the loop repeats until
everything passes or --max-iter is reached. Every iteration is logged to qa/qa_history.json.

    python -m aquabot.qa.run               # full run (CAD + kinematics + simulation)
    python -m aquabot.qa.run --fast        # skip simulation checks
    python -m aquabot.qa.run --fix         # enable the control loop
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

from .. import params as P
from . import checks as C

ROOT = Path(__file__).resolve().parents[2]
QA_DIR = ROOT / "qa"


@dataclass
class Node:
    id: str
    title: str
    checks: List[Callable[[], List[C.Result]]] = field(default_factory=list)
    children: List["Node"] = field(default_factory=list)
    results: List[C.Result] = field(default_factory=list)
    passed: Optional[bool] = None
    seconds: float = 0.0
    heavy: bool = False


def _parts(*keys):
    return [Node(f"part:{k}", k, [lambda k=k: C.part_checks(k)]) for k in keys]


def build_tree(fast: bool) -> Node:
    from ..cad.catalog import all_parts
    vend = sorted(k for k, p in all_parts().items() if p.vendor)
    is_leg = lambda leg: (lambda link: link.startswith(leg))
    legs = []
    for leg in P.LEGS:
        side = "L" if leg[1] == "L" else "R"
        legs.append(Node(f"asm:leg_{leg}", f"Leg {leg}",
                         [lambda leg=leg: [C.interference_check(leg, is_leg(leg))],
                          lambda leg=leg: [C.fastener_check(leg, is_leg(leg))],
                          lambda leg=leg: C.leg_sweep(leg)]))
    leg_parts = _parts("shoulder_bracket_L", "shoulder_bracket_R", "femur_L", "femur_R", "tibia_L", "tibia_R", "foot_pad")
    arm_link = lambda l: l.startswith(("arm_", "jaw_", "pinion"))
    sys_checks = [C.mass_checks, C.stability_checks, C.torque_checks]
    if not fast:   # runtime is computed from the simulated servo current -> needs the simulation subtree
        sys_checks.append(lambda: C.power_checks(C.SIM_CACHE))
    tree = Node("system", f"AquaBot v{P.VERSION}", sys_checks, [
        Node("sub:body", "Body & chassis",
             [lambda: [C.interference_check("body", lambda l: l == "base")], lambda: [C.fastener_check("body", lambda l: l == "base")]],
             _parts("chassis_tub", "bulkhead_front", "bulkhead_rear", "top_deck", "electronics_tray", "electronics_cover")),
        Node("sub:water", "Water system", [C.water_checks],
             _parts("tank_body", "tank_lid", "fill_cap") + _parts("pump_peristaltic", "nozzle_brass")),
        Node("sub:legs", "Legs (×4)", [], [Node("parts:leg", "Leg parts", [], leg_parts)] + legs),
        Node("sub:arm", "Arm & gripper",
             [lambda: [C.interference_check("arm", arm_link)], lambda: [C.fastener_check("arm", arm_link)], C.arm_sweep],
             [Node("asm:arm_base", "Pedestal + turntable", [], _parts("arm_pedestal", "arm_bearing_plate", "arm_turntable")),
              Node("asm:arm_links", "Upper arm + forearm", [], _parts("upper_arm", "forearm")),
              Node("asm:gripper", "Gripper", [C.gripper_checks], _parts("gripper_base", "gripper_jaw", "gripper_pinion"))]),
        Node("sub:purchased", "Purchased components", [], _parts(*[v for v in vend if v not in ("pump_peristaltic", "nozzle_brass")])),
        Node("sub:software", "Kinematics, firmware & models", [C.ik_checks, C.firmware_checks, C.model_checks]),
        Node("sub:deliverables", "BOM & documentation", [C.bom_checks, C.doc_checks]),
    ])
    if not fast:
        tree.children.append(Node("sub:simulation", "Physics simulation (MuJoCo)", [C.sim_checks, C.robustness_checks], heavy=True))
        # power uses sim power -> evaluated at system level after simulation subtree (children first)
    return tree


def evaluate(node: Node, depth=0, log=print) -> bool:
    t = time.time()
    ok = True
    for ch in node.children:                        # recursion: verify children first
        ok &= evaluate(ch, depth + 1, log)
    node.results = []
    for fn in node.checks:
        try:
            node.results += fn()
        except Exception as ex:                      # a crashing check is a failing check
            import traceback
            node.results.append(C.R_(f"ERR:{node.id}", f"check raised {type(ex).__name__}", False, None, None,
                                     traceback.format_exc()[-600:]))
    own = all(r.passed for r in node.results)
    node.passed = ok and own
    node.seconds = time.time() - t
    nfail = sum(not r.passed for r in node.results)
    log(f"{'  ' * depth}{'PASS' if node.passed else 'FAIL'}  {node.title:<34s} {len(node.results):3d} checks"
        f"{'' if not nfail else f', {nfail} failing'}  ({node.seconds:.1f}s)")
    return node.passed


def walk(node: Node):
    yield node
    for ch in node.children:
        yield from walk(ch)


# ------------------------------------------------------------- control loop
def _tuned_write(update: dict):
    path = ROOT / "aquabot" / "tuned.json"
    t = json.loads(path.read_text()) if path.exists() else {}
    for k, v in update.items():
        t.setdefault(k, {}).update(v)
    path.write_text(json.dumps(t, indent=2))
    P.CRAWL.update(t.get("CRAWL", {}))


def fix_crawl_slower():
    """Gentler gait: shorter strides (v·T × 0.92) with longer, softer swings (T × 1.08).
    (QA iter 10: the first version grew T faster than it cut v, which *lengthened* strides.)"""
    new = dict(period=round(P.CRAWL["period"] * 1.08, 3), v_cmd=round(P.CRAWL["v_cmd"] * 0.85, 4))
    _tuned_write({"CRAWL": new})
    C.SIM_CACHE.clear()
    return f"crawl period -> {new['period']} s, v_cmd -> {new['v_cmd']} m/s"


def fix_crawl_margin():
    new = dict(margin=round(P.CRAWL["margin"] + 3.0, 2))
    _tuned_write({"CRAWL": new})
    C.SIM_CACHE.clear()
    return f"crawl CoG margin -> {new['margin']} mm"


FIXERS = {"crawl_slower": fix_crawl_slower, "crawl_margin": fix_crawl_margin}


def git_sha():
    try:
        return subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    except Exception:
        return ""


def report(tree: Node, history: List[dict], wall: float):
    QA_DIR.mkdir(exist_ok=True)
    allres = [(n, r) for n in walk(tree) for r in n.results]
    npass = sum(r.passed for _, r in allres)
    data = dict(project=P.PROJECT, version=P.VERSION, date=dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
                git=git_sha(), passed=tree.passed, checks=len(allres), checks_passed=npass, wall_s=round(wall, 1),
                iterations=history,
                nodes=[dict(id=n.id, title=n.title, passed=n.passed, seconds=round(n.seconds, 2),
                            results=[r.as_dict() for r in n.results]) for n in walk(tree)])
    (QA_DIR / "qa_report.json").write_text(json.dumps(data, indent=2, default=str))
    hp = QA_DIR / "qa_history.json"
    hist = json.loads(hp.read_text()) if hp.exists() else []
    hist.append(dict(date=data["date"], git=data["git"], passed=data["passed"], checks=len(allres), checks_passed=npass,
                     iterations=len(history), fixes=[h.get("fix") for h in history if h.get("fix")]))
    hp.write_text(json.dumps(hist, indent=2))
    # markdown
    L = [f"# AquaBot QA / QC report", "",
         f"*Generated by `python -m aquabot.qa.run` on {data['date']} (git `{data['git'] or 'working tree'}`) in {wall/60:.1f} min.*", "",
         f"## Result: {'✅ ALL CHECKS PASS' if tree.passed else '❌ FAILURES PRESENT'} — {npass}/{len(allres)} checks", "",
         "Verification is hierarchical and recursive: every part is verified before the assembly that uses it, "
         "every assembly before its subsystem, and the system-level checks run last. A node passes only if all of "
         "its descendants pass.", "", "## Product tree", "", "```"]
    def tree_lines(n, d=0):
        mark = "✔" if n.passed else "✘"
        nf = sum(not r.passed for r in n.results)
        L.append(f"{'│  ' * d}{mark} {n.title}  [{len(n.results)} checks{', %d failing' % nf if nf else ''}]")
        for ch in n.children:
            tree_lines(ch, d + 1)
    tree_lines(tree)
    L += ["```", "", "## Control-loop iterations", ""]
    if history:
        L += ["| Iter | Failing checks | Corrective action |", "|---:|---|---|"]
        for h in history:
            L.append(f"| {h['iter']} | {', '.join(h['failing']) or '—'} | {h.get('fix') or '—'} |")
    L += ["", "## All checks", "", "| Node | ID | Check | Result | Value | Limit | Detail |", "|---|---|---|:---:|---|---|---|"]
    for n, r in allres:
        v = json.dumps(r.value, default=str) if not isinstance(r.value, str) else r.value
        lim = json.dumps(r.limit, default=str) if not isinstance(r.limit, str) else r.limit
        det = (r.detail or "").replace("|", "/").replace("\n", " ")[:160]
        L.append(f"| {n.title} | `{r.id}` | {r.title} | {'✅' if r.passed else '❌'} | {v} | {lim} | {det} |")
    (QA_DIR / "QA_REPORT.md").write_text("\n".join(L) + "\n")
    return data


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true", help="skip physics simulation checks")
    ap.add_argument("--fix", action="store_true", help="enable the corrective control loop")
    ap.add_argument("--max-iter", type=int, default=4)
    a = ap.parse_args(argv)
    t0 = time.time()
    history = []
    for it in range(1, a.max_iter + 1):
        print(f"\n=== QA iteration {it} ===")
        C.full_interference.cache_clear(); C._holes.cache_clear()
        tree = build_tree(a.fast)
        evaluate(tree)
        failing = [r for n in walk(tree) for r in n.results if not r.passed]
        entry = dict(iter=it, failing=[r.id for r in failing], passed=tree.passed)
        fixers = sorted({r.fixer for r in failing if r.fixer})
        if tree.passed or not a.fix or not fixers:
            history.append(entry)
            break
        msgs = [FIXERS[f]() for f in fixers]
        entry["fix"] = "; ".join(msgs)
        history.append(entry)
        print("  control action:", entry["fix"])
    data = report(tree, history, time.time() - t0)
    print(f"\nQA {'PASSED' if tree.passed else 'FAILED'}: {data['checks_passed']}/{data['checks']} checks, "
          f"{len(history)} iteration(s), {time.time()-t0:.0f}s -> qa/QA_REPORT.md")
    return 0 if tree.passed else 1


if __name__ == "__main__":
    sys.exit(main())

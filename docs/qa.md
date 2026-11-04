# Recursive quality assurance & control

`python -m aquabot.qa.run --fix` ([`aquabot/qa/run.py`](../aquabot/qa/run.py)) verifies the whole project and
writes [`qa/QA_REPORT.md`](../qa/QA_REPORT.md), `qa/qa_report.json` and appends to `qa/qa_history.json`.

## Recursive verification (V-model, bottom-up)

The product is a tree. `evaluate(node)` first evaluates every child, then the node's own integration checks;
a node passes only if all descendants and its own checks pass.

```
AquaBot (system: mass, stability, torque budget, power)
├─ Body & chassis      (interference, fasteners)      └─ parts: tub, bulkheads, deck, tray, cover
├─ Water system        (tank volume, jet speed)        └─ parts: tank, lid, cap, pump, nozzle
├─ Legs ×4             ├─ leg parts
│                      └─ Leg FL/FR/RL/RR (interference, fasteners, joint sweeps, envelope, crouch)
├─ Arm & gripper       (interference, fasteners, sweeps, mission poses)
│                      ├─ pedestal + turntable ├─ upper arm + forearm └─ gripper (rack/pinion, grip force)
├─ Purchased components (datasheet data, STEP)
├─ Kinematics, firmware & models (IK round trips, FK == CAD, C++ parity, MJCF mass, URDF FK)
├─ BOM & documentation (quantities, files, links, videos)
└─ Physics simulation   (walk, turn, watering, pick & place, IK showcase)
```

## Check catalogue (abridged)

| Level | Checks |
|---|---|
| Part | valid single B-rep solid · STEP round-trip volume < 0.1 % · STL watertight, volume < 1.5 % · fits 220×220×250 bed · ray-cast wall thickness · print notes · mass plausible |
| Assembly | exact OpenCascade boolean interference ≤ 0.5 mm³ · every fastener hole has a coaxial partner · FCL mesh sweeps over joint-limit grids · operating-envelope collisions · joint-limit margin ≥ 3° · rack/pinion geometry, grip force |
| System | gait robustness (cycle time ±10 %) · mass · 4-foot static stability margin over all mission configurations · crawl CoG margin at 720 instants · static torque SF (3-foot support, arm at full reach + payload) · IK round-trip 1e-6 mm · FK == CAD · firmware parity · tank volume from CAD cavity · runtime from simulated power · MJCF/URDF consistency · BOM ↔ CAD · docs links · simulation KPIs |

## Control loop

Checks that fail and carry a `fixer` trigger a corrective action, e.g. `crawl_slower` (cycle × 1.08,
speed × 0.85 → shorter, softer strides) or `crawl_margin` (+3 mm CoG margin). The tuned value is written to `aquabot/tuned.json`
(read by `params.py`), the simulation cache is invalidated, and the tree is re-evaluated — up to
`--max-iter` times. Design changes that cannot be automated (geometry, layout) are made by a human and the
loop is re-run; their history is in the [design log](design_log.md).

[`qa/control_loop_demo.md`](../qa/control_loop_demo.md) is a recorded run of the loop
(`python -m scripts.control_loop_demo`): starting from a deliberately aggressive crawl (1.6 s cycle,
6 cm/s) it retunes the gait until every walking check passes. Running that demo is how two real problems
were found — a fixer that made strides longer, and a turn-in-place instability next to the nominal cycle
time — see design-log iteration 10.

## CI

[`.github/workflows/qa.yml`](../.github/workflows/qa.yml) runs the fast QA (no simulation) and the firmware
host build on every push, and the full QA (with simulation) weekly and on demand (workflow_dispatch).

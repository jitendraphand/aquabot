"""Write the README metrics table and docs/requirements.md from qa/qa_report.json."""
import json
import re
from pathlib import Path

from aquabot import params as P

ROOT = Path(__file__).resolve().parents[1]

REQ_TEXT = {
    "mass_max_g": ("R1", "Total mass with a full tank", "≤ {} g", "S-MAS-01"),
    "tank_ml_min": ("R2", "Usable water volume", "≥ {} ml", "S-WAT-01"),
    "torque_sf_static_min": ("R3", "Static servo torque safety factor (legs and arm)", "≥ {}", "S-TRQ-01, S-TRQ-02"),
    "sim_peak_torque_frac": ("R4", "Simulated servo torque, 50 ms peak", "≤ {} × stall", "S-SIM-03"),
    "sim_p99_torque_frac": ("R5", "Simulated servo torque, 99th percentile", "≤ {} × stall", "S-SIM-04"),
    "stability_margin_min": ("R6", "Static stability margin, all mission configurations", "≥ {} mm", "S-STB-01"),
    "walk_speed_min": ("R7", "Walking speed with full tank", "≥ {} m/s", "S-SIM-01"),
    "tilt_max_deg": ("R8", "Body tilt while walking / turning", "≤ {}°", "S-SIM-02"),
    "runtime_min": ("R9", "Mission runtime per charge", "≥ {} min", "S-PWR-01"),
    "water_delivery_min": ("R10", "Water landing in the target pot", "≥ {} of emitted", "S-SIM-06"),
    "ik_err_max": ("R11", "IK round-trip error", "≤ {} mm", "S-KIN-01, S-KIN-03"),
    "fw_parity_max": ("R12", "Firmware IK parity (float32)", "≤ 30 × {}°", "S-FW-03"),
    "grasp_lift_min": ("R13", "Pick & place: lift height / placement error", "≥ {} mm / ≤ 25 mm", "S-SIM-08"),
    "interference_max_mm3": ("R14", "Part interference at nominal pose", "≤ {} mm³ per pair", "A-INT-01"),
}


def main():
    d = json.loads((ROOT / "qa" / "qa_report.json").read_text())
    res = {r["id"]: r for n in d["nodes"] for r in n["results"]}
    g = lambda k: res.get(k, {})
    bom = json.loads((ROOT / "bom" / "bom_summary.json").read_text())
    rows = [
        ("Size (L × W × H, standing, arm stowed)", "347 × 202 × 470 mm"),
        ("Mass (dry / 400 ml water)", f"{bom['robot_mass_dry_g']/1000:.2f} kg / {bom['robot_mass_wet_g']/1000:.2f} kg"),
        ("Degrees of freedom", "12 legs + 4 arm + 1 gripper"),
        ("Water", f"{g('S-WAT-01').get('value')} ml tank (CAD cavity), {P.PUMP['flow_ml_min']:.0f} ml/min peristaltic pump"),
        ("Walking (crawl, full tank)", f"{g('S-SIM-01').get('value')} m/s, max tilt {g('S-SIM-02').get('value')}°"),
        ("Servo load in simulation", f"p99 {g('S-SIM-04').get('value')} × stall, 50 ms peak {g('S-SIM-03').get('value')} × stall"),
        ("Static torque safety factor", f"legs {g('S-TRQ-01').get('value')}, arm {g('S-TRQ-02').get('value')}"),
        ("Stability margin (worst mission pose)", f"{g('S-STB-01').get('value')} mm"),
        ("Watering accuracy (sim)", f"{100*g('S-SIM-06').get('value', 0):.0f} % of water lands in the pot"),
        ("Estimated runtime (walking continuously)", f"{g('S-PWR-01').get('value')} min per 2S 2200 mAh charge"),
        ("Indicative cost", f"≈ ${bom['total_cost_usd']:.0f}"),
        ("QA status", f"{'✅' if d['passed'] else '❌'} {d['checks_passed']}/{d['checks']} checks passing "
                      f"([report](qa/QA_REPORT.md))"),
    ]
    table = "| | |\n|---|---|\n" + "\n".join(f"| {a} | {b} |" for a, b in rows)
    rd = (ROOT / "README.md").read_text()
    rd = re.sub(r"<!-- METRICS -->.*<!-- /METRICS -->", "<!-- METRICS -->\n" + table + "\n<!-- /METRICS -->", rd, flags=re.S)
    (ROOT / "README.md").write_text(rd)

    L = ["# Requirements & verification", "",
         f"Generated from `aquabot/params.py::REQ` and the latest QA run ({d['date']}). Every requirement is verified "
         "automatically; the method column names the QA check(s).", "",
         "| ID | Requirement | Target | Verified by | Result | Measured |", "|---|---|---|---|:---:|---|"]
    for k, (rid, txt, fmt, chk) in REQ_TEXT.items():
        ids = [c.strip() for c in chk.split(",")]
        rs = [r for i in ids for rid, r in res.items() if rid == i or rid.startswith(i + ":")]
        ok = bool(rs) and all(r["passed"] for r in rs)
        if len(rs) > 4:     # per-node checks (e.g. interference per subsystem): summarise
            meas = f"{sum(r['passed'] for r in rs)}/{len(rs)} nodes pass"
        else:
            meas = "; ".join(json.dumps(r["value"]) if not isinstance(r["value"], str) else r["value"] for r in rs)
        L.append(f"| {rid} | {txt} | {fmt.format(P.REQ[k])} | `{chk}` | {'✅' if ok else '❌'} | {meas} |")
    L += ["", "Functional requirements (demonstrated in simulation):", "",
          "* **F1** Walk along a crop row and turn in place — `S-SIM-01…05`, video 01.",
          "* **F2** Aim and dose water into a pot beside the robot — `S-SIM-06/07`, video 02.",
          "* **F3** Pick a 40 g object from the ground and place it — `S-SIM-08/09`, video 03.",
          "* **F4** Every part printable on a 220 × 220 mm bed with a 0.4 mm nozzle — `P-MFG-01/02`.",
          "* **F5** All geometry, BOM, models and firmware derived from one parameter file — `S-FW-01`, `S-MOD-01/02`, `S-BOM-01…03`."]
    (ROOT / "docs" / "requirements.md").write_text("\n".join(L) + "\n")
    print("README metrics + docs/requirements.md updated")


if __name__ == "__main__":
    main()

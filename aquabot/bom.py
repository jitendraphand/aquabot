"""Bill of materials generated from the link tree (+ hole/fastener interfaces)."""
from __future__ import annotations

import csv
import json
from collections import Counter, OrderedDict
from functools import lru_cache
from pathlib import Path

from . import params as P
from . import robot as R
from .cad.catalog import all_parts

ROOT = Path(__file__).resolve().parents[1]

# Items that are not modelled as geometry but are required to build the robot.
EXTRA_ITEMS = [
    # (category, item, qty, unit_price_usd, unit_mass_g, note)
    ("Electronics", "Servo extension lead 25 cm (JR/Futaba)", 8, 0.35, 5.0, "J2/J3/arm servos to PCA9685"),
    ("Electronics", "Dupont jumper set + 2.54 mm headers", 1, 3.0, 10.0, "I2C, PWM, sensor wiring"),
    ("Electronics", "Silicone wire 16 AWG, red+black, 1 m each", 1, 3.0, 20.0, "battery -> switch -> UBECs"),
    ("Electronics", "Blade fuse holder + 15 A fuse", 1, 1.5, 6.0, "battery positive lead"),
    ("Electronics", "Electrolytic capacitor 1000 µF / 16 V", 2, 0.3, 2.0, "one per servo rail at PCA9685 V+"),
    ("Electronics", "Voltage divider 100k/47k + 100 nF", 1, 0.1, 0.5, "battery voltage -> ESP32 ADC (GPIO35)"),
    ("Water", "Silicone tube 2 x 4 mm, 1 m", 1, 1.5, 12.0, "dip tube, pump, arm run to nozzle"),
    ("Water", "O-ring 16 x 2 mm NBR", 2, 0.1, 0.3, "fill cap (one spare)"),
    ("Water", "Silicone sheet 1 mm, 130 x 90 mm", 1, 1.0, 12.0, "tank lid gasket"),
    ("Water", "Food-safe epoxy coating (2-part, 10 ml)", 1, 4.0, 0.0, "seal inside of tank"),
    ("Water", "Tube clips, printed or 4 mm cable clips", 6, 0.05, 0.5, "route tube along arm"),
    ("Hardware", "VHB double-sided foam tape 19 mm, 0.5 m", 1, 2.0, 3.0, "pump, UBECs, buck, IMU, ESP32"),
    ("Hardware", "Hook-and-loop battery strap 200 mm", 1, 0.8, 4.0, "battery on tank lid"),
    ("Hardware", "M2.5 x 6 nylon standoff + screws", 4, 0.1, 0.4, "PCA9685 on tray"),
    ("Hardware", "M2 x 6 self-tapping screw", 2, 0.03, 0.2, "ToF board on front bulkhead"),
    ("Hardware", "M2.5 x 8 SHCS + nut", 2, 0.05, 0.5, "XT60 panel mount"),
    ("Hardware", "Zip ties 100 mm", 20, 0.02, 0.2, "cable management"),
    ("Consumable", "PETG filament (≈ 0.9 kg incl. supports & tests)", 1, 18.0, 0.0, "1 kg spool"),
    ("Consumable", "TPU 95A filament (≈ 40 g)", 1, 3.0, 0.0, "feet"),
]

PRICE = {  # indicative unit prices (USD) for fasteners
    "M3 heat-set insert (M3x5.7)": 0.04, "M3 heat-set insert (M3x4)": 0.04,
    "M3x10 SHCS": 0.05, "M3x8 SHCS": 0.05, "M3x12 SHCS": 0.05, "M3x10 countersunk (horn holes drilled Ø3.2)": 0.05,
    "M3x8 button head": 0.05, "M3x10 SHCS + Ø7 nylon washer": 0.07, "M3x16 SHCS + M3 nyloc": 0.08,
    "M3x50 SHCS (through pedestal post into deck insert)": 0.12, P.IDLER_BOLT: 0.15,
    "M2x6 self-tapping": 0.03, "M2x8 self-tapping": 0.03, P.IDLER_NUT: 0.06,
    "M3x6 horn centre screw (supplied with horn)": 0.0,
}


def instance_counts() -> Counter:
    c = Counter()
    for l in R.build_tree().values():
        for it in l.instances:
            c[it.part] += 1
    return c


@lru_cache(maxsize=1)
def fastener_counts() -> Counter:
    parts = all_parts()
    c = Counter()
    for l in R.build_tree().values():
        for it in l.instances:
            for h in parts[it.part].holes:
                if h.fastener:
                    c[h.fastener] += 1
                if h.extra:
                    c[h.extra] += 1
                if h.fastener == P.IDLER_BOLT:
                    c[P.IDLER_NUT] += 1
            if it.part.startswith("horn_ds3218"):
                c["M3x6 horn centre screw (supplied with horn)"] += 1
    return c


def files_for(key: str):
    return (f"cad/stl/{key}.stl", f"cad/step/{key}.step")


def build_rows():
    from .massprops import part_mass_props, fill_fraction
    parts = all_parts()
    cnt = instance_counts()
    rows = []
    idx = 1
    # printed
    for k in sorted(cnt):
        p = parts[k]
        if p.vendor:
            continue
        m, _, _ = part_mass_props(k)
        stl, step = files_for(k)
        rows.append(OrderedDict(id=f"P{idx:02d}", category="Printed", subsystem=p.subsystem, item=k,
                                description=p.descr, qty=cnt[k], material=P.MATERIALS[p.material]["note"],
                                unit_mass_g=round(m, 1), unit_price_usd=round(m * 0.022, 2),
                                source="3D print", files=f"{stl}; {step}", note=p.print_note))
        idx += 1
    # vendor (modelled)
    idx = 1
    for k in sorted(cnt):
        p = parts[k]
        if not p.vendor:
            continue
        rows.append(OrderedDict(id=f"V{idx:02d}", category="Purchased", subsystem=p.subsystem, item=k,
                                description=p.descr, qty=cnt[k], material="-",
                                unit_mass_g=round(p.mass_g, 1), unit_price_usd=p.price_usd,
                                source=p.supplier, files=f"cad/step/{k}.step (envelope)", note=""))
        idx += 1
    # fasteners
    idx = 1
    for k, n in sorted(fastener_counts().items()):
        rows.append(OrderedDict(id=f"F{idx:02d}", category="Fastener", subsystem="hardware", item=k,
                                description=k, qty=n, material="steel A2 / brass insert",
                                unit_mass_g=0.0, unit_price_usd=PRICE.get(k, 0.05), source="hardware kit",
                                files="", note="count derived from CAD hole interfaces"))
        idx += 1
    idx = 1
    for cat, item, q, pr, m, note in EXTRA_ITEMS:
        rows.append(OrderedDict(id=f"X{idx:02d}", category=cat, subsystem=cat.lower(), item=item,
                                description=item, qty=q, material="-", unit_mass_g=m, unit_price_usd=pr,
                                source="generic", files="", note=note))
        idx += 1
    return rows


def write(out_dir: Path | None = None) -> dict:
    from .massprops import total_mass
    out_dir = out_dir or ROOT / "bom"
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = build_rows()
    keys = list(rows[0].keys())
    with open(out_dir / "BOM.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys + ["ext_price_usd"])
        w.writeheader()
        for r in rows:
            r2 = dict(r); r2["ext_price_usd"] = round(r["qty"] * r["unit_price_usd"], 2)
            w.writerow(r2)
    total_cost = sum(r["qty"] * r["unit_price_usd"] for r in rows)
    printed_mass = sum(r["qty"] * r["unit_mass_g"] for r in rows if r["category"] == "Printed")
    summary = dict(lines=len(rows), total_cost_usd=round(total_cost, 2),
                   printed_parts=sum(1 for r in rows if r["category"] == "Printed"),
                   printed_instances=sum(r["qty"] for r in rows if r["category"] == "Printed"),
                   printed_mass_g=round(printed_mass, 1),
                   robot_mass_dry_g=round(total_mass(0.0), 1), robot_mass_wet_g=round(total_mass(1.0), 1))
    # markdown
    md = [f"# AquaBot v{P.VERSION} — Bill of Materials", "",
          "Generated from the CAD link tree by `aquabot/bom.py` — do not edit by hand.", "",
          f"* **Estimated cost:** ${summary['total_cost_usd']:.0f} (indicative 2026 USD, excl. shipping/tools)",
          f"* **Mass:** {summary['robot_mass_dry_g']/1000:.2f} kg dry, {summary['robot_mass_wet_g']/1000:.2f} kg with 400 ml water",
          f"* **Printed parts:** {summary['printed_parts']} unique / {summary['printed_instances']} prints, "
          f"≈ {summary['printed_mass_g']:.0f} g filament in parts", ""]
    for cat in ["Printed", "Purchased", "Fastener", "Electronics", "Water", "Hardware", "Consumable"]:
        sel = [r for r in rows if r["category"] == cat]
        if not sel:
            continue
        md += [f"## {cat}", "", "| ID | Item | Qty | Unit mass (g) | Unit $ | Notes |", "|---|---|---:|---:|---:|---|"]
        for r in sel:
            name = r["item"]
            if cat == "Printed":
                name = f"[{r['item']}](../cad/stl/{r['item']}.stl)"
            note = r["note"] if cat != "Purchased" else f"{r['description']} — {r['source']}"
            if cat == "Printed":
                note = f"{r['description']}. *{r['note']}*"
            md.append(f"| {r['id']} | {name} | {r['qty']} | {r['unit_mass_g']} | {r['unit_price_usd']:.2f} | {note} |")
        md.append("")
    (out_dir / "BOM.md").write_text("\n".join(md))
    (out_dir / "bom_summary.json").write_text(json.dumps(summary, indent=2))
    return summary

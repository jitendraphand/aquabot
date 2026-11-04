# Printing

All printed parts fit a 220 × 220 × 250 mm bed (QA check `P-MFG-01`) and have a ray-cast minimum wall of
≥ 0.8 mm with < 6 % of the surface below 1.2 mm (`P-MFG-02`). Per-part orientation and settings are in the
BOM notes and on each [drawing sheet](drawings).

| Material | Parts | Settings |
|---|---|---|
| **PETG** (white) | chassis_tub, bulkheads, top_deck, femurs, tibias, electronics_tray, gripper_jaw, gripper_pinion | 0.2 mm layers (0.12 mm for jaws & pinion), 4 walls, 30–35 % gyroid |
| **PETG** (green) | shoulder brackets, arm parts, electronics_cover, gripper_base | 4–5 walls, 40 % gyroid around bearing seats |
| **PETG** (blue) | tank_body, tank_lid, fill_cap | 6 walls / 100 % solid, slow outer wall, then epoxy-seal |
| **TPU 95A** | foot_pad ×4 | 3 walls, 25 % infill, 25 mm/s |

Quantities: 34 prints total (24 unique). Approximate filament: ≈ 1.0 kg in parts; budget 1 kg of PETG plus
~40 g TPU. Estimated print time on a 0.4 mm nozzle Ender-class printer: ~55 h.

**Tolerances.** Servo windows are modelled with 0.3 mm clearance per side, bearing seats at nominal
diameter (press fit in PETG — print one test coupon first). If your printer over-extrudes, scale the
`c = 0.3` clearance constants in the part generators, rebuild and re-run QA.

**Supports.** Only the femur/upper-arm idler arms and the gripper end plates need (tree) supports; all other
parts print support-free in the stated orientation.

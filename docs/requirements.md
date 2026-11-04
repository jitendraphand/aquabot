# Requirements & verification

Generated from `aquabot/params.py::REQ` and the latest QA run (2026-11-04T11:08:09+00:00). Every requirement is verified automatically; the method column names the QA check(s).

| ID | Requirement | Target | Verified by | Result | Measured |
|---|---|---|---|:---:|---|
| R1 | Total mass with a full tank | ≤ 3200.0 g | `S-MAS-01` | ✅ | 3193.7 |
| R2 | Usable water volume | ≥ 400.0 ml | `S-WAT-01` | ✅ | 404.5 |
| R3 | Static servo torque safety factor (legs and arm) | ≥ 2.0 | `S-TRQ-01, S-TRQ-02` | ✅ | 2.76; 2.15 |
| R4 | Simulated servo torque, 50 ms peak | ≤ 0.9 × stall | `S-SIM-03` | ✅ | 0.826 |
| R5 | Simulated servo torque, 99th percentile | ≤ 0.75 × stall | `S-SIM-04` | ✅ | 0.489 |
| R6 | Static stability margin, all mission configurations | ≥ 25.0 mm | `S-STB-01` | ✅ | 60.2 |
| R7 | Walking speed with full tank | ≥ 0.02 m/s | `S-SIM-01` | ✅ | 0.0272 |
| R8 | Body tilt while walking / turning | ≤ 12.0° | `S-SIM-02` | ✅ | 2.29 |
| R9 | Mission runtime per charge | ≥ 20.0 min | `S-PWR-01` | ✅ | 24.0 |
| R10 | Water landing in the target pot | ≥ 0.85 of emitted | `S-SIM-06` | ✅ | 1.0 |
| R11 | IK round-trip error | ≤ 1e-06 mm | `S-KIN-01, S-KIN-03` | ✅ | 0.0; 0.0 |
| R12 | Firmware IK parity (float32) | ≤ 30 × 0.001° | `S-FW-03` | ✅ | {"max_angle_err_deg": 0.000509, "max_gait_err_mm": 9e-05} |
| R13 | Pick & place: lift height / placement error | ≥ 30.0 mm / ≤ 25 mm | `S-SIM-08` | ✅ | {"lift_mm": 54.5, "place_err_mm": 1.2, "unreachable": 0, "ik_clamps": 0} |
| R14 | Part interference at nominal pose | ≤ 0.5 mm³ per pair | `A-INT-01` | ✅ | 6/6 nodes pass |

Functional requirements (demonstrated in simulation):

* **F1** Walk along a crop row and turn in place — `S-SIM-01…05`, video 01.
* **F2** Aim and dose water into a pot beside the robot — `S-SIM-06/07`, video 02.
* **F3** Pick a 40 g object from the ground and place it — `S-SIM-08/09`, video 03.
* **F4** Every part printable on a 220 × 220 mm bed with a 0.4 mm nozzle — `P-MFG-01/02`.
* **F5** All geometry, BOM, models and firmware derived from one parameter file — `S-FW-01`, `S-MOD-01/02`, `S-BOM-01…03`.

# Design calculations

Every number here is computed by code and re-verified on each QA run; the live values are in
[qa/QA_REPORT.md](../qa/QA_REPORT.md) and the summary table in the [README](../README.md#key-numbers).

## Mass budget

Printed parts: OpenCascade volume and surface area per part,
$m=\rho\,[A\,t_{shell}/2+\phi\,(V-A\,t_{shell}/2)]$ with $t_{shell}=1.6$ mm (4 perimeters), infill
$\phi=0.30$ (0.25 TPU), $\rho_{PETG}=1.27$ g/cm³; water-tight parts solid. Purchased parts: datasheet
masses. Fasteners counted from the CAD hole interfaces (311 items) plus 85 g wiring/tubing.

| Group | ≈ mass |
|---|---|
| 17 servos (15 × DS3218 60 g, 2 × MG90S 13.4 g) | 927 g |
| Printed parts (34 prints) | 1000 g |
| Battery, UBECs, electronics, pump | 270 g |
| Bearings, horns, fasteners, wiring | 600 g |
| **Dry / with 400 ml water** | **2.79 kg / 3.19 kg** |

## Leg torque

Crawl support is always ≥ 3 feet. For each lifted leg the vertical reactions $F_i$ of the three stance
feet follow from force and moment balance with the CoG at the planned sway position
($\sum F_i=W$, $\sum F_i x_i=Wx_G$, $\sum F_i y_i=Wy_G$); joint torques are $\tau=J^TF$. Worst case
(QA `S-TRQ-01`) is the FR leg while FL swings: 15.4 N on the foot, 0.71 N·m at the hip abduction and
0.69 N·m at the knee → static safety factor ≈ 2.8 on DS3218MG (1.96 N·m stall at 6 V). For a trot (2-foot
support) the static SF drops to ≈ 2.1, and the dynamic simulation saturates the hip servos — the reason the
crawl is the default.

## Arm torque

Horizontal full reach (q = 0) with a 100 g payload at the TCP, moments of every distal link's CoG about
each pitch axis (QA `S-TRQ-02`): shoulder 0.66 N·m (SF 3.0), elbow 0.31 N·m (SF 6.4), wrist MG90S
0.10 N·m (SF 2.1, the critical joint — swap for an MG92B if heavier payloads are needed).

## Gripper

Rack & pinion m1 z20 → pitch radius 10 mm, so the MG90S's 0.216 N·m gives $F=\tau/r\approx21.6$ N of
closing force; the slip force for 100 g at μ = 0.6 with SF 2 needs 1.6 N (QA `A-GRP-02`). 20 mm stroke per
jaw = 115° of servo rotation (< 180°).

## Stability

Static margin = distance from the ground projection of the CoG (CAD mass model, arm pose, water level,
payload) to the nearest edge of the support polygon. Crawl: the planner keeps ≥ 20 mm inside the
triangle; QA samples 720 instants including turns (`S-STB-02`).

## Water

* Tank cavity from CAD (box interior minus corner bosses and baffle): ≈ 401 ml (`S-WAT-01`).
* Jet: $v_0 = Q/A = 1.667\times10^{-6}/1.767\times10^{-6} = 0.94$ m/s; at −50° and 55 mm drop the jet
  travels ≈ 34 mm horizontally.
* A 10 ml dose takes 6 s; one tank waters 40 seedlings at 10 ml.

## Power & runtime

A geared hobby servo draws current roughly proportional to its output torque, *including while holding a
load without moving* — for a standing quadruped this dominates the mechanical power. The simulation logs,
for every servo at every step, $I = I_{idle} + I_{stall}\,|\tau|/\tau_{stall}$ (DS3218: 2.5 A stall at 6 V).

$$P = \frac{6\,\text{V}\cdot\overline{\textstyle\sum I}}{\eta_{UBEC}} + \frac{P_{logic}}{0.85} + P_{pump}$$

with $\eta_{UBEC}=0.9$, $P_{logic}=1.6$ W and the pump at 25 % duty. Usable energy 80 % × 7.4 V × 2.2 Ah
= 13 Wh. Walking continuously the servo rail averages ≈ 4.7 A (≈ 34 W total), giving ≈ 23 min
(`S-PWR-01`); the simulated peak of 9 A is well inside the two 10 A UBECs (`S-PWR-02`). Watering stops spend
most of the time standing, so a typical mission lasts longer than the walking-only figure. A 3S pack with
higher-voltage servos or a 3000 mAh 2S pack are the easiest runtime upgrades.

# Design log — QA-driven iterations

AquaBot was developed in closed loop with its own QA suite: design → generate → verify → fix → repeat.
Each entry records what a check found and what changed. Parameter changes carry a `QA iter N` comment in
[`aquabot/params.py`](../aquabot/params.py) and the part generators.

| Iter | Finding (check) | Root cause | Change |
|---:|---|---|---|
| 1 | Pedestal ⟂ electronics tray/cover, 540–800 mm³ (`A-INT-01`) | pedestal flange overlapped the cover footprint | pedestal clamped by 4 × M3×50 through-bolts in its posts; flange trimmed; tray/cover shortened |
| 1 | Yaw servo ⟂ pedestal flange, 1200 mm³ (`A-INT-01`) | servo case bottom 1.5 mm into the flange | window cut in the flange under the servo |
| 1 | Femur / upper-arm idler bridge ⟂ servo case, 58.5 mm³ ×5 (`A-INT-01`) | bridge overlapped the case bottom by 0.75 mm | bridge stepped around the case (0.35 mm gap) |
| 1 | Nozzle ⟂ gripper base, 17.6 mm³ (`A-INT-01`) | clip bore stopped short of the mount plate | bore carried through the plate |
| 1 | 68 horn holes without partner (`A-FAS-01`) | horn model was missing its hole interfaces | hole data added (then 360/360 matched) |
| 2 | Front femur hits the tub floor at ~55° hip pitch, stance needs 45° + stride (FCL sweep) | knee housing swings under the belly | hip-pitch axis lowered from 32 → 45 mm below the abduction axis |
| 3 | Knee holds 0.63 N·m in 4-foot stance → SF ≈ 1.5 in a trot (static torque) | Spot-style knees put the knee far behind the foot | ANYmal-style "X" layout (rear knees forward) using the *same* printed parts rotated 180°; taller stance |
| 4 | Trot saturates hip-pitch servos (sim, p99 = 100 % stall) | (a) MuJoCo position actuator damps against absolute velocity — swing legs fought it; (b) real limit: reflected rotor inertia ≈ 0.0045 kg·m² and 2-foot support at 3.2 kg | (a) proper hobby-servo model (slew + PD + feed-forward + DC-motor envelope); (b) statically stable crawl with CoG sway becomes the default gait |
| 5 | Crawl tipped over (120° roll) | (a) sway of 48 mm in 0.16 s pushed the ZMP outside the support; (b) sway computed from *neutral* feet while stance feet were ±37 mm along their stride | cycle 2.4 s with a 0.34 s 4-foot sway window; sway computed from the planned moving feet; heading-hold bounded; stance widened 10 mm |
| 6 | Hip pitch hits its stop when lifting a foot (`A-KIN-03`) | feet 25 mm inboard of J2 (torque-balance for trot) leaned the femurs too far | stance height 225 mm, feet under J2 (foot_in 0), stance shifted 5 mm forward → ≥ 5° margin |
| 7 | Shoulder limit −20° was unsafe — contact starts at −16° (fine FCL sweep) | crossbar half-width missing from the hand calculation | turntable front trimmed, shoulder axis +8 mm → contact at −31°, limit −27° |
| 8 | 262/277 checks: mass 3217 g > 3200 g (`S-MAS-01`) | — | tub side-wall windows, 2.4 mm tank lid, larger upper-arm window → 3194 g |
| 8 | Foot pad wall 0.43 mm (`P-MFG-02`) | knife edge where the bore exits the sphere | flat top at 9 mm above the sphere centre |
| 8 | Tub STL not 2-manifold (`P-GEO-03`) | bulkhead bosses exactly tangent to the floor | bosses embedded 0.5 mm |
| 8 | Gripper horn holes unmatched (`A-FAS-01`) | horn modelled on the wrist link although it turns with the pinion | horn moved to the pinion link |
| 8 | Pick crouch outside knee/hip limits and rear femur touching the tub (`A-KIN-04`) — the simulation had been silently clamping | 150 mm crouch with walking stance | feet re-stepped (front +20, rear −30 mm) with a stable sway routine before crouching; roll −10°, lean 20 mm; clamping now counted and required to be 0 |
| 9 | IK showcase target unreachable after iter 7 (`ERR:sub:simulation`) | circle centred for the old shoulder height | circle re-centred |
| 9 | Foot-pad STL degenerate triangle at the sphere pole (`P-GEO-03`) | tessellator artefact | STL export removes degenerate faces and re-checks manifoldness |
| 9 | Gripper/wrist PWM on ESP32 strapping pins 12/14 (review) | boot-mode risk | moved to GPIO 32/33 |
| 9 | Runtime estimate 84 min was optimistic (review of `S-PWR-01`) | power model counted only mechanical work; servos draw current ∝ torque while *holding* | simulation logs per-servo current $I=I_0+I_{stall}\,|\tau|/\tau_{stall}$ → 23 min walking continuously (still ≥ 20 min) |
| 10 | Control-loop demo from an aggressive gait: first fixer never converged (`S-SIM-03`) | it grew the cycle time faster than it cut speed, so strides got *longer* | fixer now shortens strides (v·T × 0.92) while lengthening the swing; converges in 3 iterations ([demo](../qa/control_loop_demo.md)) |
| 10 | Same demo: turning in place tipped over for 1.9–2.35 s cycles, 2.4 s nominal was on the edge (`S-SIM-05`) | diagonal rocking mode during turn-in-place (also with the IMU loop off) | dedicated 3.0 s turn cycle (stable at ±10 %); new robustness check `S-SIM-11` perturbs both cycle times ±10 % on every QA run |

## Known limitations

* **Speed.** Crawl ≈ 2.5 cm/s with a full tank. Trot exists but needs ≥ 35 kg·cm servos (e.g. DS3235) or
  ~30 % less mass to stay below 90 % of stall.
* **Pick from ground** works at the edge of the envelope (≥ 1.2° joint margin in the crouch). Objects must be
  ~150 mm beside the body; a longer forearm (≥ 130 mm) would add margin.
* **Not yet built.** Servo envelopes and horn patterns are datasheet values; print a test bracket first.
* **Water.** The jet model ignores air drag and wind (negligible at 0.94 m/s over 55 mm, but outdoors gusts
  matter); there is no level sensor — the tank volume is integrated from pump time.
* **Terrain.** Flat ground only in simulation; no foothold planning or perception (the ToF is wired but only
  used for obstacle stop in the firmware roadmap).

## Roadmap

1. Physical prototype + calibration jig; measure servo stiffness to tune the simulation servo model.
2. Capacitive soil-moisture probe carried by the gripper ("measure, then water").
3. ESP32-CAM plant detection and row following.
4. Higher-torque leg servos → trot at 10–15 cm/s.

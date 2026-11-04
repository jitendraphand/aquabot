# Simulation

The MuJoCo model ([`sim/models/aquabot.xml`](../sim/models/aquabot.xml)) is **generated**, not hand-written,
by [`aquabot/sim/mjcf.py`](../aquabot/sim/mjcf.py) from the same link tree that places the CAD parts:

* **Bodies and joints** — one MuJoCo body per link, joint axes and limits from `robot.py`.
* **Mass properties** — per-part volume, CoG and inertia tensor from OpenCascade, scaled by a printed-part
  model (4 × 0.4 mm shell + 30 % infill) or the datasheet mass for purchased parts, then combined per link
  (parallel-axis theorem). QA `S-MOD-01` checks MJCF mass == CAD mass within 0.5 %.
* **Water** — a separate body whose mass and CoG follow the remaining volume as the pump runs.
* **Visuals** — the exported part meshes; **collisions** — primitive spheres (feet), capsules (legs),
  boxes (body, finger pads).

## Servo model

Hobby servos are not ideal position sources. [`aquabot/sim/runtime.py`](../aquabot/sim/runtime.py) models
each one as:

1. the commanded angle slewed at ≤ 95 % of the no-load speed (DS3218 ≈ 6.7 rad/s);
2. PD on the slewed command with velocity feed-forward, $\tau = k_p(q_c-q)+k_d(\dot q_c-\dot q)$;
3. a DC-motor envelope $|\tau|\le\tau_{stall}(1-|\dot q|/\omega_0)$ when driving, $\tau_{stall}$ when braking;
4. joint armature 0.0045 kg·m² (rotor inertia × gear ratio² ≈ 1e-7 × 240²) and friction.

Using MuJoCo's built-in position actuator instead (damping against absolute velocity) made swing legs
saturate artificially — see design-log iteration 4.

## Scenarios ([`aquabot/sim/scenarios.py`](../aquabot/sim/scenarios.py))

| Video | What happens | Verified by |
|---|---|---|
| [01_crawl_walk_turn](../sim/videos/01_crawl_walk_turn.mp4) | 12 s crawl forward, 7 s turn in place | `S-SIM-01…05` speed, tilt, torque, turn |
| [02_watering_mission](../sim/videos/02_watering_mission.mp4) | walk a row of 3 potted seedlings, crouch + lean, ballistic aim, dose 10 ml each | `S-SIM-06/07` delivery ≥ 85 %, mass balance |
| [03_pick_and_place](../sim/videos/03_pick_and_place.mp4) | re-step feet, crouch, grasp a 40 g stone from the ground, carry and place it in a tray | `S-SIM-08/09` lift, placement error, clamping |
| [04_ik_showcase](../sim/videos/04_ik_showcase.mp4) | body pose IK with planted feet, then the TCP traces a circle | `S-SIM-10` |
| [05_exploded_assembly](../sim/videos/05_exploded_assembly.mp4) | exploded → assembled CAD animation | — |

Metrics from the rendered runs are in [`sim/videos/metrics.json`](../sim/videos/metrics.json).

```bash
make videos                 # MUJOCO_GL=egl headless rendering, 1280x720 @ 30 fps
python -c "from aquabot.sim import scenarios as s; print(s.walk())"   # metrics only, no video
```

# Assembly guide

Estimated build time: 8–10 h after printing. Tools: 2.0/2.5/3.0 mm hex keys, soldering iron with heat-set
insert tip, 3.2 mm drill, multimeter, servo tester (or the firmware's `calib` command).

Part names below match the files in [`cad/stl`](../cad/stl) and the IDs in the [BOM](../bom/BOM.md).
Exploded view: [sim/videos/05_exploded_assembly.mp4](../sim/videos/05_exploded_assembly.mp4).

## 1. Print

See [printing.md](printing.md). Print `femur_L/R`, `tibia_L/R` and `shoulder_bracket_L/R` **twice each**
(`_R` parts are mirror images of `_L`; rear legs use the opposite-hand femur/tibia turned 180°, so you need
2 × L and 2 × R of each). Print 4 × `foot_pad` in TPU and 2 × `gripper_jaw`.

## 2. Heat-set inserts (100 × M3x5.7, 8 × M3x4)

Install inserts in every hole flagged *insert* in the CAD (they are listed per part in the BOM). Typical:
tub end bosses (10) and deck bosses (6); bulkhead servo flanges (16); shoulder brackets (8 each: 4 horn,
4 servo flange); femurs and upper arm (4 servo flange each); tank corners (4); pedestal shelf (4); turntable
servo plate (4); electronics tray posts (4); deck (8 × M3x4 for pedestal & tray).

## 3. Servo preparation (all 17)

1. Centre every servo (1500 µs) with a tester before fitting a horn.
2. DS3218 aluminium horns: **drill the 4 horn holes to Ø3.2 mm** for the J1 (hip abduction) joints only —
   those horns are screwed from the horn side into inserts in the shoulder bracket.
3. Label each servo with its joint name (e.g. `FL_J2`).

## 4. Body

1. Seal the tank: coat the inside of `tank_body` and the underside of `tank_lid` with food-safe epoxy, cure,
   then fit the 1 mm silicone gasket and screw the lid down (4 × M3x10 + nylon washers). **Leak-test** with
   400 ml for 12 h. Push a 50 mm silicone dip tube onto the lid's inner barb.
2. Drop the tank into `chassis_tub`.
3. Mount the four J1 servos through the windows of `bulkhead_front` / `bulkhead_rear` (flange on the
   outside face, 4 × M3x10 each), shaft pointing outward. Fit the ToF board behind the front window, the
   rocker switch and XT60 panel connector in the rear bulkhead.
4. Screw both bulkheads to the tub (5 × M3x12 each).
5. On the tank lid (VHB tape): pump (barbs facing the rear-centre), the two UBECs, the 5 V buck, the IMU.
   Strap the battery on top of the lid with the hook-and-loop strap.

## 5. Legs (repeat ×4)

1. **Shoulder bracket:** press a 625-2RS bearing into the bracket's idler arm. Fit the J2 servo through the
   ear plate window (shaft outboard), 4 × M3x10 into the inserts.
2. Screw a J1 horn to the back plate from the horn side (4 × M3x10 countersunk into the inserts) and press the
   bracket onto the centred J1 servo spline so the leg hangs vertical; fit the horn centre screw.
3. **Femur:** fit the J3 servo into the femur housing (4 × M3x10), press the knee 625 bearing into the femur's
   idler arm. Fit a J2 horn to the J2 servo. Offer the femur up: outer tine onto the J2 horn (4 × M3x10),
   inner tine onto the bracket idler with an M5x20 low-head screw (head in the tine counterbore) + washer +
   nyloc on the bracket side. Snug only — the joint must turn freely.
4. **Tibia:** same as the femur on the J3 servo / femur idler.
5. **Foot:** push the TPU foot over the tibia spigot and pin it with an M3x16 + nyloc.
6. Front legs: knee points **backwards**; rear legs: knee points **forwards** (use the opposite-hand femur
   and tibia rotated 180°: rear-left uses `femur_R`/`tibia_R`).

## 6. Arm

1. Yaw servo into `arm_pedestal` (flange on the internal shelf, 4 × M3x10), lead out through the slot.
2. Place `arm_bearing_plate` on the pedestal, press the 61806-2RS bearing over its boss.
3. Clamp pedestal + bearing plate to the deck with 4 × **M3x50** through the posts into the deck inserts.
4. Fit the yaw horn to the centred servo, lower `arm_turntable` over the bearing (outer race in the skirt) and
   screw it to the horn (4 × M3x10 from the top).
5. Shoulder servo on the turntable plate (4 × M3x10), shoulder idler bearing in the upright.
6. `upper_arm` (elbow servo + idler bearing inside) on the shoulder fork; `forearm` (MG90S wrist servo inside)
   on the elbow fork — same fork/idler procedure as the legs.
7. **Gripper:** MG90S into `gripper_base` (2 × M2x8), pinion bolted to its round horn (4 × M2x8), the two
   `gripper_jaw`s on the Ø3 × 74 mm rods (jaws mirror each other by a 180° turn), centre the pinion with the
   jaws 1 mm apart when the servo is at −57°. Screw the gripper base to the wrist horn (4 × M2x6).
8. Push the brass nozzle into the clip under the gripper; route the silicone tube from the pump outlet through
   the deck hole at the rear of the cover, along the arm with clips, to the nozzle. Leave slack for full yaw.

## 7. Electronics

Mount `electronics_tray` (4 × M3x8), PCA9685 on 4 × M2.5 standoffs, ESP32 in its cradle, MOSFET module.
Wire per [wiring.md](wiring.md). Fit `electronics_cover` (4 × M3x8 button head).

## 8. Deck

Route all servo leads through the tub cable holes and the deck slot, close the deck (6 × M3x10). Fill cap
with O-ring into the neck.

## 9. Calibration

1. Flash the firmware ([firmware.md](firmware.md)), open the serial monitor at 115200.
2. Put the robot on a stand (feet off the ground). `stand 225` puts every joint at the IK angles of the
   standing pose.
3. For each joint that is visibly off, `calib <leg> <joint> <offset_deg>` until the femur/tibia angles
   match the printed calibration gauge (`docs/images/leg_FL_side.png` shows the target pose: hip pitch
   ±27–31°, knee ∓56°). Copy the final offsets into `LEG_OFFSET` in `hal.cpp`.
4. `pump 1` for 60 s into a measuring cup; if the flow differs from 100 ml/min, adjust `flow_ml_min` in
   `params.py` and regenerate `config.h`.
5. Put it on the floor: `stand`, then `walk 35 0 10`.

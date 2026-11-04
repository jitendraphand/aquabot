# AquaBot v1.0.0 — Bill of Materials

Generated from the CAD link tree by `aquabot/bom.py` — do not edit by hand.

* **Estimated cost:** $397 (indicative 2026 USD, excl. shipping/tools)
* **Mass:** 2.79 kg dry, 3.19 kg with 400 ml water
* **Printed parts:** 24 unique / 34 prints, ≈ 1000 g filament in parts

## Printed

| ID | Item | Qty | Unit mass (g) | Unit $ | Notes |
|---|---|---:|---:|---:|---|
| P01 | [arm_bearing_plate](../cad/stl/arm_bearing_plate.stl) | 1 | 7.6 | 0.17 | Bearing plate: carries the 61806 thin-section yaw bearing on a Ø30 boss. *Print flat, boss up, 5 walls (bearing seat), 40 % infill* |
| P02 | [arm_pedestal](../cad/stl/arm_pedestal.stl) | 1 | 26.0 | 0.57 | Arm pedestal: houses the yaw servo (flange on internal shelf); clamped by 4 M3x50 through-bolts. *Print upright, 4 walls, 30 % gyroid, no supports (shelf bridges 21 mm)* |
| P03 | [arm_turntable](../cad/stl/arm_turntable.stl) | 1 | 24.1 | 0.53 | Turntable on yaw horn + 61806 bearing; shoulder servo plate + shoulder idler. *Print plate-down, 5 walls, 40 % gyroid, supports under idler boss only* |
| P04 | [bulkhead_front](../cad/stl/bulkhead_front.stl) | 1 | 23.8 | 0.52 | Front bulkhead: carries two J1 hip-abduction servos + ToF sensor window. *Print flat, 5 walls around insert holes, 40 % gyroid* |
| P05 | [bulkhead_rear](../cad/stl/bulkhead_rear.stl) | 1 | 23.1 | 0.51 | Rear bulkhead: carries two J1 hip-abduction servos + power switch & XT60 charge port. *Print flat, 5 walls around insert holes, 40 % gyroid* |
| P06 | [chassis_tub](../cad/stl/chassis_tub.stl) | 1 | 120.8 | 2.66 | Main body tub: floor + side walls, insert bosses for bulkheads and deck. *Print upright (open top up), 4 walls, 30 % gyroid, no supports* |
| P07 | [electronics_cover](../cad/stl/electronics_cover.stl) | 1 | 29.2 | 0.64 | Splash cover over ESP32 + PCA9685 (vent slots, USB access). *Print upside-down (top on bed), 3 walls* |
| P08 | [electronics_tray](../cad/stl/electronics_tray.stl) | 1 | 18.5 | 0.41 | Electronics tray: PCA9685 standoffs, ESP32 cradle, cover posts. *Print flat, 3 walls, 20 % infill* |
| P09 | [femur_L](../cad/stl/femur_L.stl) | 2 | 48.2 | 1.06 | Femur (left): fork on hip-pitch servo, houses knee servo, carries knee idler. *Print lying on the outer tine face, 4 walls, 35 % gyroid, tree supports under idler* |
| P10 | [femur_R](../cad/stl/femur_R.stl) | 2 | 48.2 | 1.06 | Femur (right): fork on hip-pitch servo, houses knee servo, carries knee idler. *Print lying on the outer tine face, 4 walls, 35 % gyroid, tree supports under idler* |
| P11 | [fill_cap](../cad/stl/fill_cap.stl) | 1 | 9.7 | 0.21 | Fill cap: push-fit plug with 16x2 O-ring and Ø1.2 air vent. *Print head-down, 100 % infill* |
| P12 | [foot_pad](../cad/stl/foot_pad.stl) | 4 | 4.1 | 0.09 | Ø26 TPU foot (sphere contact, slips over the tibia spigot). *TPU 95A, print bore-up, 3 walls, 25 % infill, slow (25 mm/s)* |
| P13 | [forearm](../cad/stl/forearm.stl) | 1 | 31.2 | 0.69 | Forearm: fork on elbow servo, houses the MG90S wrist servo. *Print lying on the outer tine face, 4 walls, 35 % gyroid* |
| P14 | [gripper_base](../cad/stl/gripper_base.stl) | 1 | 20.2 | 0.44 | Gripper base: on wrist horn, carries gripper servo, guide rods, nozzle clip. *Print with the mount plate on the bed, 4 walls, 40 % infill, supports for end plates* |
| P15 | [gripper_jaw](../cad/stl/gripper_jaw.stl) | 2 | 4.5 | 0.10 | Gripper jaw with integral rack (two identical jaws, mirrored by rotation). *Print on the carriage side, 0.12 mm layers, 5 walls; add 1 mm TPU pad tape* |
| P16 | [gripper_pinion](../cad/stl/gripper_pinion.stl) | 1 | 1.8 | 0.04 | Pinion m1 z20, bolted to MG90S horn. *Print flat, 0.12 mm layers, 100 % infill* |
| P17 | [shoulder_bracket_L](../cad/stl/shoulder_bracket_L.stl) | 2 | 26.8 | 0.59 | Shoulder bracket (FL / RR): on J1 horn, holds hip-pitch servo + femur idler. *Print standing on the back plate, 4 walls, 40 % gyroid* |
| P18 | [shoulder_bracket_R](../cad/stl/shoulder_bracket_R.stl) | 2 | 26.8 | 0.59 | Shoulder bracket (FL / RR): on J1 horn, holds hip-pitch servo + femur idler. *Print standing on the back plate, 4 walls, 40 % gyroid* |
| P19 | [tank_body](../cad/stl/tank_body.stl) | 1 | 94.7 | 2.08 | 400 ml water reservoir with anti-slosh baffle (sits on tub floor). *PETG, 100 % solid walls (or 6 walls), 0.2 mm layers, then seal inside with food-safe epoxy; leak-test before install* |
| P20 | [tank_lid](../cad/stl/tank_lid.stl) | 1 | 42.3 | 0.93 | Tank lid with fill neck (Ø20) and dip-tube barb for the peristaltic pump. *Print flat, neck up, 6 walls; gasket: 1 mm silicone sheet* |
| P21 | [tibia_L](../cad/stl/tibia_L.stl) | 2 | 24.4 | 0.54 | Tibia (left): fork on knee servo, tapered shin, foot spigot. *Print lying on the outer tine face, 4 walls, 35 % gyroid* |
| P22 | [tibia_R](../cad/stl/tibia_R.stl) | 2 | 24.4 | 0.54 | Tibia (right): fork on knee servo, tapered shin, foot spigot. *Print lying on the outer tine face, 4 walls, 35 % gyroid* |
| P23 | [top_deck](../cad/stl/top_deck.stl) | 1 | 47.4 | 1.04 | Top deck: closes the tub, carries arm pedestal, electronics tray, fill neck. *Print flat, 4 walls, 30 % gyroid* |
| P24 | [upper_arm](../cad/stl/upper_arm.stl) | 1 | 56.4 | 1.24 | Upper arm: fork on shoulder servo, houses elbow servo + elbow idler. *Print lying on the outer tine face, 4 walls, 35 % gyroid* |

## Purchased

| ID | Item | Qty | Unit mass (g) | Unit $ | Notes |
|---|---|---:|---:|---:|---|
| V01 | battery_2s_2200 | 1 | 118.0 | 16.00 | 2S 7.4 V 2200 mAh LiPo (XT60) — hobby shop |
| V02 | bearing_61806 | 1 | 18.0 | 4.00 | 61806-2RS ball bearing 30x42x7 — AliExpress / bearing shop |
| V03 | bearing_625 | 10 | 5.0 | 0.60 | 625-2RS ball bearing 5x16x5 — AliExpress / bearing shop |
| V04 | buck_5v | 1 | 2.0 | 1.00 | Mini-360 buck 5 V 2 A — AliExpress / Amazon |
| V05 | esp32_devkitc | 1 | 10.0 | 8.00 | ESP32-DevKitC V4 (WROOM-32E) — AliExpress / Amazon |
| V06 | horn_ds3218 | 15 | 6.0 | 1.50 | 25T aluminium disc horn Ø25 — AliExpress (25T alu horn) |
| V07 | horn_mg90s | 2 | 1.5 | 0.00 | MG90S round horn Ø20 — included w/ servo |
| V08 | imu_mpu6050 | 1 | 2.0 | 2.00 | MPU-6050 6-axis IMU (GY-521) — AliExpress / Amazon |
| V09 | mosfet_module | 1 | 2.0 | 1.50 | AO3400 logic-level MOSFET module (pump) — AliExpress / Amazon |
| V10 | nozzle_brass | 1 | 2.0 | 0.80 | Brass dispensing tip, Ø1.5 mm orifice, barbed for 2x4 mm tube — AliExpress (blunt dispensing tip) |
| V11 | pca9685 | 1 | 9.0 | 4.00 | PCA9685 16-ch PWM driver — AliExpress / Amazon |
| V12 | pump_peristaltic | 1 | 62.0 | 9.00 | 6 V DC mini peristaltic pump, 2x3 mm tube head, 100 ml/min — AliExpress (Kamoer-style NKP) |
| V13 | rocker_switch | 1 | 4.0 | 0.50 | KCD1 rocker switch 10 A (13x19 mm cut-out) — AliExpress |
| V14 | rod_3x74 | 2 | 4.1 | 0.30 | Ø3 x 74 mm ground steel rod (cut from 3 mm silver steel) — hardware store |
| V15 | servo_ds3218 | 15 | 60.0 | 14.00 | DS3218MG 270° servo, 1.96 N·m stall @6 V — AliExpress / Amazon (DSSERVO) |
| V16 | servo_mg90s | 2 | 13.4 | 4.00 | MG90S servo, 0.22 N·m stall @6 V — AliExpress / Amazon (DSSERVO) |
| V17 | tof_vl53l0x | 1 | 1.0 | 4.00 | VL53L0X ToF range sensor — AliExpress / Amazon |
| V18 | ubec_6v_10a | 2 | 28.0 | 9.00 | 6.0 V 10 A UBEC — hobby shop |
| V19 | xt60_panel_mount | 1 | 6.0 | 1.50 | XT60 panel-mount female (charge port) — hobby shop |

## Fastener

| ID | Item | Qty | Unit mass (g) | Unit $ | Notes |
|---|---|---:|---:|---:|---|
| F01 | M2x6 self-tapping | 4 | 0.0 | 0.03 | count derived from CAD hole interfaces |
| F02 | M2x8 self-tapping | 8 | 0.0 | 0.03 | count derived from CAD hole interfaces |
| F03 | M3 heat-set insert (M3x4) | 8 | 0.0 | 0.04 | count derived from CAD hole interfaces |
| F04 | M3 heat-set insert (M3x5.7) | 100 | 0.0 | 0.04 | count derived from CAD hole interfaces |
| F05 | M3x10 SHCS | 110 | 0.0 | 0.05 | count derived from CAD hole interfaces |
| F06 | M3x10 SHCS + Ø7 nylon washer | 4 | 0.0 | 0.07 | count derived from CAD hole interfaces |
| F07 | M3x10 countersunk (horn holes drilled Ø3.2) | 16 | 0.0 | 0.05 | count derived from CAD hole interfaces |
| F08 | M3x12 SHCS | 10 | 0.0 | 0.05 | count derived from CAD hole interfaces |
| F09 | M3x16 SHCS + M3 nyloc | 4 | 0.0 | 0.08 | count derived from CAD hole interfaces |
| F10 | M3x50 SHCS (through pedestal post into deck insert) | 4 | 0.0 | 0.12 | count derived from CAD hole interfaces |
| F11 | M3x6 horn centre screw (supplied with horn) | 15 | 0.0 | 0.00 | count derived from CAD hole interfaces |
| F12 | M3x8 SHCS | 4 | 0.0 | 0.05 | count derived from CAD hole interfaces |
| F13 | M3x8 button head | 4 | 0.0 | 0.05 | count derived from CAD hole interfaces |
| F14 | M5 nyloc nut + M5 washer | 10 | 0.0 | 0.06 | count derived from CAD hole interfaces |
| F15 | M5x20 low-head SHCS (DIN 7984) | 10 | 0.0 | 0.15 | count derived from CAD hole interfaces |

## Electronics

| ID | Item | Qty | Unit mass (g) | Unit $ | Notes |
|---|---|---:|---:|---:|---|
| X01 | Servo extension lead 25 cm (JR/Futaba) | 8 | 5.0 | 0.35 | J2/J3/arm servos to PCA9685 |
| X02 | Dupont jumper set + 2.54 mm headers | 1 | 10.0 | 3.00 | I2C, PWM, sensor wiring |
| X03 | Silicone wire 16 AWG, red+black, 1 m each | 1 | 20.0 | 3.00 | battery -> switch -> UBECs |
| X04 | Blade fuse holder + 15 A fuse | 1 | 6.0 | 1.50 | battery positive lead |
| X05 | Electrolytic capacitor 1000 µF / 16 V | 2 | 2.0 | 0.30 | one per servo rail at PCA9685 V+ |
| X06 | Voltage divider 100k/47k + 100 nF | 1 | 0.5 | 0.10 | battery voltage -> ESP32 ADC (GPIO35) |

## Water

| ID | Item | Qty | Unit mass (g) | Unit $ | Notes |
|---|---|---:|---:|---:|---|
| X07 | Silicone tube 2 x 4 mm, 1 m | 1 | 12.0 | 1.50 | dip tube, pump, arm run to nozzle |
| X08 | O-ring 16 x 2 mm NBR | 2 | 0.3 | 0.10 | fill cap (one spare) |
| X09 | Silicone sheet 1 mm, 130 x 90 mm | 1 | 12.0 | 1.00 | tank lid gasket |
| X10 | Food-safe epoxy coating (2-part, 10 ml) | 1 | 0.0 | 4.00 | seal inside of tank |
| X11 | Tube clips, printed or 4 mm cable clips | 6 | 0.5 | 0.05 | route tube along arm |

## Hardware

| ID | Item | Qty | Unit mass (g) | Unit $ | Notes |
|---|---|---:|---:|---:|---|
| X12 | VHB double-sided foam tape 19 mm, 0.5 m | 1 | 3.0 | 2.00 | pump, UBECs, buck, IMU, ESP32 |
| X13 | Hook-and-loop battery strap 200 mm | 1 | 4.0 | 0.80 | battery on tank lid |
| X14 | M2.5 x 6 nylon standoff + screws | 4 | 0.4 | 0.10 | PCA9685 on tray |
| X15 | M2 x 6 self-tapping screw | 2 | 0.2 | 0.03 | ToF board on front bulkhead |
| X16 | M2.5 x 8 SHCS + nut | 2 | 0.5 | 0.05 | XT60 panel mount |
| X17 | Zip ties 100 mm | 20 | 0.2 | 0.02 | cable management |

## Consumable

| ID | Item | Qty | Unit mass (g) | Unit $ | Notes |
|---|---|---:|---:|---:|---|
| X18 | PETG filament (≈ 0.9 kg incl. supports & tests) | 1 | 0.0 | 18.00 | 1 kg spool |
| X19 | TPU 95A filament (≈ 40 g) | 1 | 0.0 | 3.00 | feet |

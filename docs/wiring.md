# Wiring

Pin assignments are generated into [`firmware/aquabot/config.h`](../firmware/aquabot/config.h) by
[`aquabot/firmware_config.py`](../aquabot/firmware_config.py); change them there, not in the header.

## Power tree

```mermaid
flowchart LR
  B["2S LiPo 7.4 V 2200 mAh"] -->|XT60| F["15 A blade fuse"] --> S["rocker switch (rear bulkhead)"]
  C["XT60 charge port (rear)"] -.->|balance lead + main| B
  S --> U1["UBEC L 6.0 V / 10 A"] --> RL["servo rail L: FL, RL legs + arm yaw, shoulder"]
  S --> U2["UBEC R 6.0 V / 10 A"] --> RR["servo rail R: FR, RR legs + elbow, wrist, gripper"]
  S --> K["Mini-360 buck 5 V"] --> E["ESP32 VIN · PCA9685 VCC · ToF/IMU via ESP32 3V3"]
  U2 --> M["AO3400 MOSFET (low side)"] --> P["peristaltic pump 6 V"]
  S --> D["100k/47k divider"] --> A["ESP32 GPIO35 (battery voltage)"]
```

All grounds are common. Put a 1000 µF / 16 V capacitor across each servo rail at the PCA9685 V+ terminal.
The PCA9685 **V+ terminal is fed from the UBEC rails**, never from the 5 V logic supply.

## Signals

| Function | Device | Pin / channel |
|---|---|---|
| I²C SDA / SCL (PCA9685 0x40, MPU-6050 0x68, VL53L0X 0x29) | ESP32 | GPIO 21 / GPIO 22 |
| FL hip-abd / hip-pitch / knee | PCA9685 | ch 0 / 1 / 2 |
| FR hip-abd / hip-pitch / knee | PCA9685 | ch 4 / 5 / 6 |
| RL hip-abd / hip-pitch / knee | PCA9685 | ch 8 / 9 / 10 |
| RR hip-abd / hip-pitch / knee | PCA9685 | ch 12 / 13 / 14 |
| Arm yaw / shoulder / elbow | ESP32 LEDC | GPIO 25 / 26 / 27 |
| Wrist / gripper (MG90S) | ESP32 LEDC | GPIO 32 / 33 |
| Pump MOSFET gate | ESP32 LEDC 2 kHz | GPIO 13 |
| Battery sense | ESP32 ADC1 | GPIO 35 |
| ToF XSHUT | ESP32 | GPIO 4 |
| Status LED | ESP32 | GPIO 2 |

Servo leads: J1 servos sit inside the body; J2/J3 leads run down the outside of the bracket and enter the tub
through the Ø12 mm side holes and bulkhead grommets (25 cm extensions). The arm leads run inside the pedestal
and out through its slot; leave a service loop for ±120° yaw.

## Current budget

| Load | Typical | Peak |
|---|---|---|
| 17 servos, crawl gait (simulated, I ∝ torque incl. holding) | 4.7 A mean | 9.1 A |
| arm alone, reaching with payload | 0.5–1 A | 2.5 A |
| ESP32 + sensors | 0.2 A @ 5 V | 0.35 A |
| Pump | 0.35 A @ 6 V | 0.4 A |

Two 10 A UBECs give ≈ 2× headroom over the simulated peak (QA `S-PWR-02`). Use 16 AWG from the battery to
the UBECs and 20 AWG from the UBECs to the PCA9685 V+ terminals.

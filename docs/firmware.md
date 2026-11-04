# Firmware

ESP32-DevKitC (Arduino core ≥ 3.0) driving 12 leg servos through a PCA9685 and the 5 arm servos plus the pump
MOSFET from the ESP32's LEDC PWM. No external libraries are required (the PCA9685 and MPU-6050 are driven with
raw I²C register writes).

| File | Purpose |
|---|---|
| [`aquabot.ino`](../firmware/aquabot/aquabot.ino) | 100 Hz control loop, mode state machine, serial command parser |
| [`config.h`](../firmware/aquabot/config.h) | **generated** geometry, joint limits, gait parameters, pin map |
| [`kinematics.cpp`](../firmware/aquabot/kinematics.cpp) | leg IK/FK, arm IK/FK with tool offsets, ballistic aiming |
| [`gait.cpp`](../firmware/aquabot/gait.cpp) | crawl planner with CoG sway (identical algorithm to Python) |
| [`hal.cpp`](../firmware/aquabot/hal.cpp) | PCA9685, LEDC, pump, IMU complementary filter, battery ADC |
| [`test/host_ik.cpp`](../firmware/test/host_ik.cpp) | host harness used by QA for Python ↔ C++ parity |
| [`test/host_build.cpp`](../firmware/test/host_build.cpp) | compiles and runs the whole sketch on a PC with stubs |

## Serial commands (115200 baud)

```
stand [height_mm]            stand up / change body height (150–235)
sit                          crouch
walk <vx mm/s> <wz deg/s> <seconds>
water <x> <y> <z> <ml>       aim at a soil point (body frame, mm) and dose
pick <x> <y> <z> | place <x> <y> <z> | grip <mm> | stow
pump <0|1>                   manual pump
calib <leg 0-3> <joint 0-2> <offset_deg>
status                       battery V, tank estimate, attitude
```

Example watering a seedling 235 mm to the left of the arm, soil 145 mm below the body:
`stand 195` → `water 55 235 -113 10`.

## Verification

`make firmware-test` (and QA checks `S-FW-01…03`):

* `config.h` is byte-identical to what `params.py` generates;
* the full sketch compiles on the host with `-Wall -Wextra -Werror` and runs 300 control ticks;
* 400 leg-IK, 300 arm-IK (nozzle tool) and 60 crawl-planner vectors match Python within float32 precision.

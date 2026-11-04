// Hardware abstraction: PCA9685 (legs), ESP32 LEDC (arm), pump MOSFET, MPU-6050, battery ADC.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

namespace hal {

void begin();

// ---- servos ----
void setLegAngle(int leg, int joint, float rad);   // joint angle (rad) -> calibrated pulse
void setArmAngle(int joint, float rad);            // 0 yaw, 1 shoulder, 2 elbow, 3 wrist
void setGripper(float mm);                         // jaw opening per side 0..GRIP_STROKE
void relaxAll();

// ---- water ----
void pump(bool on);

// ---- sensors ----
struct Imu { float roll, pitch, gx, gy, gz; };
bool readImu(Imu& out, float dt);                  // complementary filter, rad
float batteryVolts();

}  // namespace hal

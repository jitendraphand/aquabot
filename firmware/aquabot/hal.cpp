#include "hal.h"
#include <math.h>

float LEG_OFFSET[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

namespace hal {

static const float RAD2DEG = 57.2957795f;

// ------------------------------------------------------------ PCA9685
static void pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

static void pcaPulseUs(uint8_t ch, float us) {
  // 50 Hz -> 20000 us per 4096 ticks
  uint16_t off = (uint16_t)(us * 4096.0f / 20000.0f + 0.5f);
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x06 + 4 * ch);
  Wire.write(0); Wire.write(0);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  Wire.endTransmission();
}

static void pcaBegin() {
  pcaWrite(0x00, 0x10);                 // sleep
  pcaWrite(0xFE, 121);                  // prescale: 25 MHz / (4096 * 50 Hz) - 1
  pcaWrite(0x00, 0x20);                 // wake, auto-increment
  delay(2);
  pcaWrite(0x00, 0xA0);                 // restart
}

// ------------------------------------------------------------ LEDC (arm)
static const int ARM_PINS[5] = {PIN_ARM_YAW, PIN_ARM_SHOULDER, PIN_ARM_ELBOW, PIN_ARM_WRIST, PIN_ARM_GRIP};
static const float ARM_RANGE[5] = {DS_RANGE_DEG, DS_RANGE_DEG, DS_RANGE_DEG, MG_RANGE_DEG, MG_RANGE_DEG};
static const int8_t ARM_DIR[5] = {1, -1, 1, 1, 1};
float ARM_OFFSET[5] = {0, 0, 0, 0, 0};

static void ledcPulseUs(int idx, float us) {
#ifndef HOST_BUILD
  ledcWrite(ARM_PINS[idx], (uint32_t)(us / 20000.0f * 65535.0f));
#else
  (void)idx; (void)us;
#endif
}

static float degToUs(float deg, float range) {
  float us = 1500.0f + deg * (2000.0f / range);
  if (us < 500) us = 500;
  if (us > 2500) us = 2500;
  return us;
}

void begin() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  pcaBegin();
#ifndef HOST_BUILD
  for (int i = 0; i < 5; ++i) ledcAttach(ARM_PINS[i], 50, 16);
  ledcAttach(PIN_PUMP, 2000, 8);
#endif
  pinMode(PIN_LED, OUTPUT);
  // MPU-6050: wake, ±500 dps, ±4 g, DLPF 44 Hz
  const uint8_t init[][2] = {{0x6B, 0x00}, {0x1B, 0x08}, {0x1C, 0x08}, {0x1A, 0x03}};
  for (auto& r : init) {
    Wire.beginTransmission(0x68); Wire.write(r[0]); Wire.write(r[1]); Wire.endTransmission();
  }
}

void setLegAngle(int leg, int joint, float rad) {
  const float deg = LEG_DIR[leg][joint] * rad * RAD2DEG + LEG_OFFSET[leg][joint];
  pcaPulseUs(LEG_CH[leg][joint], degToUs(deg, DS_RANGE_DEG));
}

void setArmAngle(int j, float rad) {
  const float deg = ARM_DIR[j] * rad * RAD2DEG + ARM_OFFSET[j];
  ledcPulseUs(j, degToUs(deg, ARM_RANGE[j]));
}

void setGripper(float mm) {
  // rack & pinion: jaw travel = r_pitch * theta  (r_pitch = 10 mm)
  float deg = (mm / 10.0f) * RAD2DEG - 57.0f;     // fully closed ~ -57°, fully open ~ +57°
  ledcPulseUs(4, degToUs(deg, MG_RANGE_DEG));
}

void relaxAll() {
  for (int ch = 0; ch < 16; ++ch) pcaPulseUs(ch, 0);
  for (int i = 0; i < 5; ++i) ledcPulseUs(i, 0);
}

void pump(bool on) {
#ifndef HOST_BUILD
  ledcWrite(PIN_PUMP, on ? 255 : 0);
#else
  (void)on;
#endif
}

bool readImu(Imu& o, float dt) {
  Wire.beginTransmission(0x68); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(0x68, 14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; ++i) v[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  const float ax = v[0] / 8192.0f, ay = v[1] / 8192.0f, az = v[2] / 8192.0f;   // g
  o.gx = v[4] / 65.5f / RAD2DEG; o.gy = v[5] / 65.5f / RAD2DEG; o.gz = v[6] / 65.5f / RAD2DEG;
  const float accRoll = atan2f(ay, az), accPitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  const float k = 0.98f;
  o.roll = k * (o.roll + o.gx * dt) + (1 - k) * accRoll;
  o.pitch = k * (o.pitch + o.gy * dt) + (1 - k) * accPitch;
  return true;
}

float batteryVolts() {
  // 100k / 47k divider on GPIO35
  return analogRead(PIN_VBAT) / 4095.0f * 3.3f * (147.0f / 47.0f);
}

}  // namespace hal

// AquaBot firmware — ESP32-DevKitC + PCA9685
// Serial (115200) commands, one per line:
//   stand [height_mm]            stand up / change body height
//   sit                          lower and relax
//   walk <vx mm/s> <wz deg/s> <seconds>   crawl gait (3 feet always down)
//   water <x> <y> <z> <ml>       aim jet (ballistic IK) at soil point (body frame, mm) and dose
//   pick <x> <y> <z>             grasp object at point (body frame, mm)
//   place <x> <y> <z>            place held object
//   stow                         arm to transport pose
//   pump <0|1>                   manual pump
//   calib <leg 0-3> <joint 0-2> <offset_deg>
//   status                       print battery, tank estimate, IMU
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "kinematics.h"
#include "gait.h"
#include "hal.h"

using kin::Vec3;

static const float D2R = 0.0174532925f;
static const float STOW_Q[4] = {0.0f * D2R, 50.0f * D2R, -115.0f * D2R, -80.0f * D2R};

enum Mode { IDLE, STAND, WALK, ARM_MOVE, WATERING };
static Mode mode = IDLE;
static gait::Crawl crawl;
static float bodyH = STAND_HEIGHT;
static float walkVx = 0, walkWz = 0, walkT = 0, walkDur = 0;
static float armQ[4] = {STOW_Q[0], STOW_Q[1], STOW_Q[2], STOW_Q[3]};
static float armFrom[4], armTo[4], armT = 0, armDur = 1;
static float pumpLeft = 0;            // seconds of pumping left
static float tankMl = TANK_ML;
static hal::Imu imu = {0, 0, 0, 0, 0};
static float legCmd[4][3];

static float minjerk(float s) { if (s <= 0) return 0; if (s >= 1) return 1; return 10*s*s*s - 15*s*s*s*s + 6*s*s*s*s*s; }

static void commandLegs(Vec3 feet[4]) {
  // attitude feedback: rotate foot targets to level the body (small-angle PI on IMU)
  static float ir = 0, ip = 0;
  const float dt = 1.0f / CTRL_HZ;
  ir += -imu.roll * dt; ip += -imu.pitch * dt;
  ir = constrain(ir, -0.15f, 0.15f); ip = constrain(ip, -0.15f, 0.15f);
  const float cr = 0.6f * -imu.roll + 1.5f * ir, cp = 0.6f * -imu.pitch + 1.5f * ip;
  for (int l = 0; l < 4; ++l) {
    Vec3 p = feet[l];
    // R^T(roll=cr, pitch=cp) * p  (small angles)
    Vec3 q{p.x - cp * p.z, p.y + cr * p.z, p.z + cp * p.x - cr * p.y};
    float ang[3];
    kin::legIK((kin::Leg)l, q, ang, true);
    for (int j = 0; j < 3; ++j) {
      // slew limit to the servo's speed
      const float maxStep = DS_MAX_DPS * D2R * dt;
      float d = ang[j] - legCmd[l][j];
      if (d > maxStep) d = maxStep;
      if (d < -maxStep) d = -maxStep;
      legCmd[l][j] += d;
      hal::setLegAngle(l, j, legCmd[l][j]);
    }
  }
}

static void startArm(const float q[4], float seconds) {
  for (int i = 0; i < 4; ++i) { armFrom[i] = armQ[i]; armTo[i] = q[i]; }
  armT = 0; armDur = seconds; mode = ARM_MOVE;
}

static void handleLine(String s) {
  s.trim();
  char cmd[16] = {0};
  float a = 0, b = 0, c = 0, d = 0;
  int n = sscanf(s.c_str(), "%15s %f %f %f %f", cmd, &a, &b, &c, &d);
  if (n < 1) return;
  String C(cmd);
  if (C == "stand") { if (n >= 2) bodyH = constrain(a, CROUCH_HEIGHT, STAND_HEIGHT + 10); mode = STAND; }
  else if (C == "sit") { bodyH = CROUCH_HEIGHT; mode = STAND; }
  else if (C == "walk" && n >= 4) {
    walkVx = a; walkWz = b * D2R; walkDur = c; walkT = 0; mode = WALK;
    // turning in place uses a slower cycle (QA: shorter cycles excite a diagonal rocking mode)
    crawl.period = (fabsf(a) < 1e-3f && fabsf(b) > 0) ? CRAWL_TURN_PERIOD : CRAWL_PERIOD;
  }
  else if (C == "stow") startArm(STOW_Q, 1.5f);
  else if (C == "pump") { hal::pump(a > 0.5f); }
  else if (C == "water" && n >= 5) {
    Vec3 target{a, b, c};
    Vec3 from{SHOULDER_X - a, SHOULDER_Y - b, 0};
    Vec3 tip = kin::aimNozzle(target, from, -50 * D2R, 55.0f, kin::jetSpeed());
    float q[4];
    if (kin::armIK(tip, -50 * D2R, NOZZLE_A, NOZZLE_B, q)) {
      startArm(q, 1.5f);
      pumpLeft = d / PUMP_ML_PER_S;
      Serial.printf("aim ok, dose %.1f ml (%.1f s)\n", d, pumpLeft);
    } else Serial.println("ERR target out of reach");
  }
  else if ((C == "pick" || C == "place") && n >= 4) {
    float q[4];
    if (kin::armIK(Vec3{a, b, c}, -75 * D2R, L_TCP, 0, q)) {
      startArm(q, 1.5f);
      hal::setGripper(C == "pick" ? GRIP_STROKE * 0.85f : GRIP_STROKE * 0.75f);
    } else Serial.println("ERR target out of reach");
  }
  else if (C == "grip") hal::setGripper(a);
  else if (C == "calib" && n >= 4) { LEG_OFFSET[(int)a][(int)b] = c; Serial.println("ok"); }
  else if (C == "status") {
    Serial.printf("vbat %.2f V  tank %.0f ml  roll %.1f pitch %.1f deg  mode %d\n", hal::batteryVolts(), tankMl,
                  imu.roll / D2R, imu.pitch / D2R, (int)mode);
  }
}

void setup() {
  Serial.begin(115200);
  hal::begin();
  for (int l = 0; l < 4; ++l) {
    Vec3 p = kin::neutralFoot((kin::Leg)l, CROUCH_HEIGHT);
    kin::legIK((kin::Leg)l, p, legCmd[l], true);
  }
  Serial.println("AquaBot ready");
}

void loop() {
  static uint32_t last = 0;
  static String line;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') { handleLine(line); line = ""; } else line += ch;
  }
  const uint32_t now = micros();
  if (now - last < 1000000UL / CTRL_HZ) return;
  last = now;
  const float dt = 1.0f / CTRL_HZ;
  hal::readImu(imu, dt);

  Vec3 feet[4];
  bool contact[4];
  if (mode == WALK) {
    const float amp = constrain(fminf(walkT / (0.5f * crawl.period), (walkDur - walkT) / (0.5f * crawl.period)), 0.0f, 1.0f);
    crawl.targets(walkT, walkVx, 0, walkWz, bodyH, amp, feet, contact);
    walkT += dt;
    if (walkT >= walkDur) mode = STAND;
  } else {
    for (int l = 0; l < 4; ++l) feet[l] = kin::neutralFoot((kin::Leg)l, bodyH);
  }
  if (mode != IDLE) commandLegs(feet);

  if (mode == ARM_MOVE || mode == WATERING) {
    armT += dt;
    const float s = minjerk(armT / armDur);
    for (int i = 0; i < 4; ++i) armQ[i] = armFrom[i] + (armTo[i] - armFrom[i]) * s;
    if (armT >= armDur) mode = (pumpLeft > 0) ? WATERING : STAND;
  }
  for (int i = 0; i < 4; ++i) hal::setArmAngle(i, armQ[i]);

  if (mode == WATERING) {
    hal::pump(true);
    pumpLeft -= dt; tankMl -= PUMP_ML_PER_S * dt;
    if (pumpLeft <= 0 || tankMl <= 5) { hal::pump(false); pumpLeft = 0; mode = STAND; Serial.println("dose done"); }
  }
  digitalWrite(PIN_LED, (millis() / 500) % 2);
}

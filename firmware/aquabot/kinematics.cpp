#include "kinematics.h"
#include "config.h"
#include <math.h>

namespace kin {

static const float DEG = 3.14159265358979f / 180.0f;

static void limits(Leg leg, float lo[3], float hi[3]) {
  switch (leg) {
    case FL: lo[0] = LIM_FL_ABD_LO; hi[0] = LIM_FL_ABD_HI; lo[1] = LIM_FL_PITCH_LO; hi[1] = LIM_FL_PITCH_HI;
             lo[2] = LIM_FL_KNEE_LO; hi[2] = LIM_FL_KNEE_HI; break;
    case FR: lo[0] = LIM_FR_ABD_LO; hi[0] = LIM_FR_ABD_HI; lo[1] = LIM_FR_PITCH_LO; hi[1] = LIM_FR_PITCH_HI;
             lo[2] = LIM_FR_KNEE_LO; hi[2] = LIM_FR_KNEE_HI; break;
    case RL: lo[0] = LIM_RL_ABD_LO; hi[0] = LIM_RL_ABD_HI; lo[1] = LIM_RL_PITCH_LO; hi[1] = LIM_RL_PITCH_HI;
             lo[2] = LIM_RL_KNEE_LO; hi[2] = LIM_RL_KNEE_HI; break;
    default: lo[0] = LIM_RR_ABD_LO; hi[0] = LIM_RR_ABD_HI; lo[1] = LIM_RR_PITCH_LO; hi[1] = LIM_RR_PITCH_HI;
             lo[2] = LIM_RR_KNEE_LO; hi[2] = LIM_RR_KNEE_HI; break;
  }
  for (int i = 0; i < 3; ++i) { lo[i] *= DEG; hi[i] *= DEG; }
}

Vec3 neutralFoot(Leg leg, float height) {
  Vec3 p;
  p.x = signE(leg) * (HIP_X + J2_DX - FOOT_IN) + STANCE_DX;
  p.y = signS(leg) * (HIP_Y + J2_DY + STANCE_DY);
  p.z = -height + FOOT_R;
  return p;
}

Vec3 legFK(Leg leg, const float q[3]) {
  const float e = signE(leg), s = signS(leg);
  const float xs = -L_FEMUR * sinf(q[1]) - L_TIBIA * sinf(q[1] + q[2]);
  const float zs = -L_FEMUR * cosf(q[1]) - L_TIBIA * cosf(q[1] + q[2]);
  const float vx = e * J2_DX + xs, vy = s * J2_DY, vz = J2_DZ + zs;
  const float c1 = cosf(q[0]), s1 = sinf(q[0]);
  return Vec3{e * HIP_X + vx, s * HIP_Y + c1 * vy - s1 * vz, s1 * vy + c1 * vz};
}

bool legIK(Leg leg, const Vec3& p, float q[3], bool checkLimits) {
  const float e = signE(leg), s = signS(leg);
  bool ok = true;
  const float vx = p.x - e * HIP_X, vy = p.y - s * HIP_Y, vz = p.z;
  const float a = s * J2_DY;
  float r2 = vy * vy + vz * vz - a * a;
  if (r2 <= 0.0f) { r2 = 1.0f; ok = false; }
  const float b = -sqrtf(r2);
  float q1 = atan2f(vz, vy) - atan2f(b, a);
  while (q1 > 3.14159265f) q1 -= 6.2831853f;
  while (q1 < -3.14159265f) q1 += 6.2831853f;
  const float xs = vx - e * J2_DX, zs = b - J2_DZ;
  const float u = -xs, w = -zs;
  float D = (u * u + w * w - L_FEMUR * L_FEMUR - L_TIBIA * L_TIBIA) / (2.0f * L_FEMUR * L_TIBIA);
  if (D > 1.0f) { D = 0.999f; ok = false; }
  if (D < -1.0f) { D = -0.999f; ok = false; }
  const float q3 = kneeSign(leg) * acosf(D);
  const float q2 = atan2f(u, w) - atan2f(L_TIBIA * sinf(q3), L_FEMUR + L_TIBIA * cosf(q3));
  q[0] = q1; q[1] = q2; q[2] = q3;
  if (checkLimits) {
    float lo[3], hi[3];
    limits(leg, lo, hi);
    for (int i = 0; i < 3; ++i) {
      if (q[i] < lo[i]) { q[i] = lo[i]; ok = false; }
      if (q[i] > hi[i]) { q[i] = hi[i]; ok = false; }
    }
  }
  return ok;
}

Vec3 armFK(const float q[4], float A, float B, float* phiOut) {
  const float p1 = q[1], p2 = q[1] + q[2], p3 = q[1] + q[2] + q[3];
  const float r = L_ARM1 * cosf(p1) + L_ARM2 * cosf(p2) + A * cosf(p3) - B * sinf(p3);
  const float z = L_ARM1 * sinf(p1) + L_ARM2 * sinf(p2) + A * sinf(p3) + B * cosf(p3);
  if (phiOut) *phiOut = p3;
  return Vec3{SHOULDER_X + r * cosf(q[0]), SHOULDER_Y + r * sinf(q[0]), SHOULDER_Z + z};
}

bool armIK(const Vec3& p, float phi, float A, float B, float q[4], bool elbowUp) {
  const float dx = p.x - SHOULDER_X, dy = p.y - SHOULDER_Y, z = p.z - SHOULDER_Z;
  const float qy = atan2f(dy, dx);
  const float r = sqrtf(dx * dx + dy * dy);
  const float wr = r - A * cosf(phi) + B * sinf(phi);
  const float wz = z - A * sinf(phi) - B * cosf(phi);
  float D = (wr * wr + wz * wz - L_ARM1 * L_ARM1 - L_ARM2 * L_ARM2) / (2.0f * L_ARM1 * L_ARM2);
  bool ok = true;
  if (D > 1.0f) { D = 1.0f; ok = false; }
  if (D < -1.0f) { D = -1.0f; ok = false; }
  const float qe = elbowUp ? -acosf(D) : acosf(D);
  const float qs = atan2f(wz, wr) - atan2f(L_ARM2 * sinf(qe), L_ARM1 + L_ARM2 * cosf(qe));
  float qw = phi - qs - qe;
  while (qw > 3.14159265f) qw -= 6.2831853f;
  while (qw < -3.14159265f) qw += 6.2831853f;
  q[0] = qy; q[1] = qs; q[2] = qe; q[3] = qw;
  const float lo[4] = {LIM_ARM_YAW_LO * DEG, LIM_ARM_SHOULDER_LO * DEG, LIM_ARM_ELBOW_LO * DEG, LIM_ARM_WRIST_LO * DEG};
  const float hi[4] = {LIM_ARM_YAW_HI * DEG, LIM_ARM_SHOULDER_HI * DEG, LIM_ARM_ELBOW_HI * DEG, LIM_ARM_WRIST_HI * DEG};
  for (int i = 0; i < 4; ++i)
    if (q[i] < lo[i] - 1e-6f || q[i] > hi[i] + 1e-6f) ok = false;
  return ok;
}

float jetSpeed() {
  const float Q = PUMP_ML_PER_S * 1e-6f;
  const float r = NOZZLE_ORIFICE * 0.5e-3f;
  return Q / (3.14159265f * r * r);
}

Vec3 aimNozzle(const Vec3& target, const Vec3& from, float pitch, float height, float v0) {
  const float g = 9.81f;
  const float vh = v0 * cosf(pitch), vz = v0 * sinf(pitch);
  const float h = height / 1000.0f;
  const float t = (vz + sqrtf(vz * vz + 2.0f * g * h)) / g;
  const float dist = vh * t * 1000.0f;
  float ux = from.x, uy = from.y;
  const float n = sqrtf(ux * ux + uy * uy);
  ux /= n; uy /= n;
  return Vec3{target.x + ux * dist, target.y + uy * dist, target.z + height};
}

}  // namespace kin

// Statically stable crawl gait with CoG sway — port of aquabot/kinematics/gait.py::Crawl
#pragma once
#include "kinematics.h"

namespace gait {

struct Crawl {
  float period, swing, stepHeight, margin;
  float cogX = 12.0f, cogY = 0.0f;       // body-frame CoG (mm), refreshed from the mass model
  Crawl();
  // Body-frame foot targets at time t (s since start) for velocity (vx, vy mm/s) and yaw rate wz (rad/s).
  // amp in [0,1] ramps the velocity in/out. contact[i] = true when leg i is planned on the ground.
  void targets(float t, float vx, float vy, float wz, float height, float amp, kin::Vec3 out[4], bool contact[4]) const;
  void shiftFor(int swingLeg, float height, float vx, float vy, float wz, float& sx, float& sy) const;
  kin::Vec3 planned(int leg, float phase, float height, float vx, float vy, float wz) const;
};

}  // namespace gait

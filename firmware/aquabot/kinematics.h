// AquaBot kinematics — C++ port of aquabot/kinematics/{legs,arm}.py
// Units: mm, radians. Body frame: +X forward, +Y left, +Z up.
#pragma once
#include <stdint.h>

namespace kin {

enum Leg : uint8_t { FL = 0, FR = 1, RL = 2, RR = 3 };

struct Vec3 { float x, y, z; };

inline int signE(Leg l) { return (l == FL || l == FR) ? 1 : -1; }
inline int signS(Leg l) { return (l == FL || l == RL) ? 1 : -1; }
inline int kneeSign(Leg l) { return (l == FL || l == FR) ? -1 : 1; }  // front knees back, rear forward

// Returns false if the target is out of reach or outside joint limits (q still filled, clamped).
bool legIK(Leg leg, const Vec3& p, float q[3], bool checkLimits = true);
Vec3 legFK(Leg leg, const float q[3]);
Vec3 neutralFoot(Leg leg, float height);

// Arm: tool point p (body frame) with tool pitch phi (rad, + = up, outward-referenced).
// tool = {a along gripper, b 'above' gripper}. TCP = {L_TCP, 0}; nozzle = {NOZZLE_A, NOZZLE_B}.
bool armIK(const Vec3& p, float phi, float toolA, float toolB, float q[4], bool elbowUp = true);
Vec3 armFK(const float q[4], float toolA, float toolB, float* phiOut = nullptr);

// Ballistic aiming of the water jet: nozzle tip & pitch so the jet lands on `target`.
Vec3 aimNozzle(const Vec3& target, const Vec3& approachFrom, float pitch, float height, float v0);
float jetSpeed();   // m/s from pump flow and orifice

}  // namespace kin

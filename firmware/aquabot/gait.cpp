#include "gait.h"
#include "config.h"
#include <math.h>

namespace gait {

static const int ORDER[4] = {kin::RL, kin::FL, kin::RR, kin::FR};   // swing sequence

static int slotOf(int leg) {
  for (int i = 0; i < 4; ++i) if (ORDER[i] == leg) return i;
  return 0;
}
static float minjerk(float s) { return 10 * s * s * s - 15 * s * s * s * s + 6 * s * s * s * s * s; }
static float lift(float s) { return (s < 0 || s > 1) ? 0.0f : 64.0f * s * s * s * (1 - s) * (1 - s) * (1 - s); }
static float wrap01(float x) { x = fmodf(x, 1.0f); return x < 0 ? x + 1.0f : x; }

Crawl::Crawl() : period(CRAWL_PERIOD), swing(CRAWL_SWING), stepHeight(CRAWL_STEP_HEIGHT), margin(CRAWL_MARGIN) {}

kin::Vec3 Crawl::planned(int leg, float phase, float h, float vx, float vy, float wz) const {
  kin::Vec3 p0 = kin::neutralFoot((kin::Leg)leg, h);
  const float rx = -(vx - wz * p0.y), ry = -(vy + wz * p0.x);
  const float k = period * (1.0f - swing);
  const float dx = rx * k, dy = ry * k;
  const float ph = wrap01(phase - slotOf(leg) / 4.0f);
  float f;
  if (ph < swing) f = 0.5f - minjerk(ph / swing);
  else f = (ph - swing) / (1.0f - swing) - 0.5f;
  return kin::Vec3{p0.x + dx * f, p0.y + dy * f, p0.z};
}

void Crawl::shiftFor(int swingLeg, float h, float vx, float vy, float wz, float& sx, float& sy) const {
  const float ph0 = slotOf(swingLeg) / 4.0f;
  int a = kin::FL, b = kin::RR;
  if (swingLeg == kin::FL || swingLeg == kin::RR) { a = kin::FR; b = kin::RL; }
  float best = -1e9f; sx = sy = 0;
  const kin::Vec3 sw = kin::neutralFoot((kin::Leg)swingLeg, h);
  for (int i = 0; i < 5; ++i) {
    const float ph = ph0 + swing * i / 4.0f;
    const kin::Vec3 A = planned(a, ph, h, vx, vy, wz), Bp = planned(b, ph, h, vx, vy, wz);
    float tx = Bp.x - A.x, ty = Bp.y - A.y;
    const float n = sqrtf(tx * tx + ty * ty); tx /= n; ty /= n;
    float nx = -ty, ny = tx;
    if ((sw.x - A.x) * nx + (sw.y - A.y) * ny > 0) { nx = -nx; ny = -ny; }
    const float dist = (cogX - A.x) * nx + (cogY - A.y) * ny;
    const float need = margin - dist;
    if (need > best) { best = need; sx = nx * need; sy = ny * need; }
  }
}

void Crawl::targets(float t, float vx, float vy, float wz, float h, float amp, kin::Vec3 out[4], bool contact[4]) const {
  const float phase = wrap01(t / period);
  vx *= amp; vy *= amp; wz *= amp;
  // body sway: hold during the current swing, blend to the next leg's shift in the 4-foot window
  const int k = (int)floorf(phase * 4.0f) % 4;
  float s0x, s0y, s1x, s1y;
  shiftFor(ORDER[k], h, vx, vy, wz, s0x, s0y);
  shiftFor(ORDER[(k + 1) % 4], h, vx, vy, wz, s1x, s1y);
  const float u = phase * 4.0f - floorf(phase * 4.0f), sw = swing * 4.0f;
  float a = (u < sw) ? 0.0f : minjerk((u - sw) / (1.0f - sw));
  const float shx = s0x + (s1x - s0x) * a, shy = s0y + (s1y - s0y) * a;
  for (int leg = 0; leg < 4; ++leg) {
    kin::Vec3 p = planned(leg, phase, h, vx, vy, wz);
    const float ph = wrap01(phase - slotOf(leg) / 4.0f);
    contact[leg] = ph >= swing;
    if (!contact[leg]) p.z += stepHeight * lift(ph / swing);
    p.x -= shx; p.y -= shy;
    out[leg] = p;
  }
}

}  // namespace gait

// Host-side parity harness: reads test vectors from stdin, prints C++ IK/FK results.
//   L <leg> x y z           -> q1 q2 q3 (deg) ok
//   A x y z phi_deg a b     -> qy qs qe qw (deg) ok
//   G t vx wz height        -> 4 x (x y z contact) crawl targets
#include <cstdio>
#include <cstring>
#include <cstdint>
#include "../aquabot/config.h"
#include "../aquabot/kinematics.h"
#include "../aquabot/gait.h"

int main() {
  char tag[4];
  const double R2D = 57.29577951308232;
  gait::Crawl crawl;
  while (scanf("%3s", tag) == 1) {
    if (!strcmp(tag, "L")) {
      int leg; float x, y, z;
      if (scanf("%d %f %f %f", &leg, &x, &y, &z) != 4) return 1;
      float q[3];
      bool ok = kin::legIK((kin::Leg)leg, kin::Vec3{x, y, z}, q, false);
      printf("%.6f %.6f %.6f %d\n", q[0] * R2D, q[1] * R2D, q[2] * R2D, ok ? 1 : 0);
    } else if (!strcmp(tag, "A")) {
      float x, y, z, phi, a, b;
      if (scanf("%f %f %f %f %f %f", &x, &y, &z, &phi, &a, &b) != 6) return 1;
      float q[4];
      bool ok = kin::armIK(kin::Vec3{x, y, z}, phi / R2D, a, b, q, true);
      printf("%.6f %.6f %.6f %.6f %d\n", q[0] * R2D, q[1] * R2D, q[2] * R2D, q[3] * R2D, ok ? 1 : 0);
    } else if (!strcmp(tag, "G")) {
      float t, vx, wz, h;
      if (scanf("%f %f %f %f", &t, &vx, &wz, &h) != 4) return 1;
      kin::Vec3 f[4]; bool c[4];
      crawl.targets(t, vx, 0, wz, h, 1.0f, f, c);
      for (int i = 0; i < 4; ++i) printf("%.4f %.4f %.4f %d ", f[i].x, f[i].y, f[i].z, c[i] ? 1 : 0);
      printf("\n");
    }
  }
  return 0;
}

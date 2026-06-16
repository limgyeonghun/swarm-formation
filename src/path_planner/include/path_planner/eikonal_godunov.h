#ifndef PATH_PLANNER_EIKONAL_GODUNOV_H_
#define PATH_PLANNER_EIKONAL_GODUNOV_H_

// Shared Godunov upwind update for the eikonal |grad T| * F = 1, used by both
// the CPU FMM and the GPU FIM so the two solvers produce identical numbers.
// Mirrors the 1-D/2-D/3-D quadratic in PathSearcher::fm2SolveEikonal::solveQuad.

#include <cmath>

#if defined(__CUDACC__)
  #define PP_HD __host__ __device__
#else
  #define PP_HD
#endif

namespace path_planner {

// Sentinel for "unreached / no finite neighbour". Large but finite so plain
// float arithmetic never produces NaN/Inf.
constexpr float kEikInf = 1e18f;

// Per-axis minimum frozen-neighbour arrival times mx/my/mz (kEikInf if none),
// rhs = cres / F. Returns the new arrival time at the cell (kEikInf if no
// finite neighbour exists). Isotropic spacing (one cres for all axes).
PP_HD inline float eikSolve(float mx, float my, float mz, float rhs) {
  // sort a0 <= a1 <= a2 (3-element sorting network)
  float a0 = mx, a1 = my, a2 = mz, t;
  if (a0 > a1) { t = a0; a0 = a1; a1 = t; }
  if (a1 > a2) { t = a1; a1 = a2; a2 = t; }
  if (a0 > a1) { t = a0; a0 = a1; a1 = t; }

  if (a0 >= kEikInf) return kEikInf;        // no finite neighbour

  float T = a0 + rhs;                       // 1-D update
  if (a1 < kEikInf && T > a1) {
    // 2-D: (T-a0)^2 + (T-a1)^2 = rhs^2
    const float s = a0 + a1;
    const float q = a0 * a0 + a1 * a1 - rhs * rhs;
    const float disc = s * s - 2.0f * q;
    if (disc >= 0.0f) T = 0.5f * (s + sqrtf(disc));
    if (a2 < kEikInf && T > a2) {
      // 3-D quadratic
      const float S = a0 + a1 + a2;
      const float Q = a0 * a0 + a1 * a1 + a2 * a2 - rhs * rhs;
      const float D = S * S - 3.0f * Q;
      if (D >= 0.0f) T = (S + sqrtf(D)) / 3.0f;
    }
  }
  return T;
}

}  // namespace path_planner

#endif  // PATH_PLANNER_EIKONAL_GODUNOV_H_

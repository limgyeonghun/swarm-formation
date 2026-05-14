// Sanity test for the message-level shape used by ReplanFSM::loadObstaclesCallback.
//
// This test does NOT exercise the rclcpp subscription path. It validates the
// per-spec dispatch logic by constructing DynamicObstacleArray messages and
// running the same loop body the callback executes.
//
// The wiring (subscription / publisher / network) is covered by the manual
// RViz checklist documented in docs/research/manual_test_dynamic_obstacle.md.

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

#include "path_manager/msg/dynamic_obstacle_array.hpp"
#include "path_manager/msg/dynamic_obstacle_spec.hpp"

namespace {

int g_passed = 0;
int g_failed = 0;

void check(bool cond, const std::string& label) {
  std::cout << "  " << (cond ? "[OK]   " : "[FAIL] ")
            << std::left << std::setw(48) << label << "\n";
  if (cond) ++g_passed; else ++g_failed;
}

// Replicates the dispatch loop from ReplanFSM::loadObstaclesCallback
// without depending on a real PathManager. Counts spheres that *would*
// be added vs skipped.
struct DispatchResult { size_t added = 0; size_t skipped = 0; };

DispatchResult simulate_dispatch(
    const path_manager::msg::DynamicObstacleArray& msg)
{
  DispatchResult r;
  for (const auto& spec : msg.obstacles) {
    if (spec.kind != path_manager::msg::DynamicObstacleSpec::KIND_SPHERE) {
      ++r.skipped;
      continue;
    }
    ++r.added;
  }
  return r;
}

void test_empty_array() {
  std::cout << "[test_empty_array]\n";
  path_manager::msg::DynamicObstacleArray msg;
  auto r = simulate_dispatch(msg);
  check(r.added == 0, "empty: added == 0");
  check(r.skipped == 0, "empty: skipped == 0");
}

void test_all_spheres() {
  std::cout << "[test_all_spheres]\n";
  path_manager::msg::DynamicObstacleArray msg;
  for (int i = 0; i < 3; ++i) {
    path_manager::msg::DynamicObstacleSpec s;
    s.kind = path_manager::msg::DynamicObstacleSpec::KIND_SPHERE;
    s.center.x = 1.0 * i; s.center.y = 2.0 * i; s.center.z = 3.0 * i;
    s.radius = 4.0 + i;
    msg.obstacles.push_back(s);
  }
  auto r = simulate_dispatch(msg);
  check(r.added == 3, "3 spheres: added == 3");
  check(r.skipped == 0, "3 spheres: skipped == 0");
}

void test_mixed_kinds() {
  std::cout << "[test_mixed_kinds]\n";
  path_manager::msg::DynamicObstacleArray msg;
  path_manager::msg::DynamicObstacleSpec sphere;
  sphere.kind = path_manager::msg::DynamicObstacleSpec::KIND_SPHERE;
  sphere.radius = 5.0;
  msg.obstacles.push_back(sphere);

  path_manager::msg::DynamicObstacleSpec cube;
  cube.kind = path_manager::msg::DynamicObstacleSpec::KIND_CUBE;
  cube.size.x = 1.0; cube.size.y = 1.0; cube.size.z = 1.0;
  msg.obstacles.push_back(cube);

  path_manager::msg::DynamicObstacleSpec cyl;
  cyl.kind = path_manager::msg::DynamicObstacleSpec::KIND_CYLINDER;
  msg.obstacles.push_back(cyl);

  auto r = simulate_dispatch(msg);
  check(r.added == 1, "mixed: only the sphere counts");
  check(r.skipped == 2, "mixed: cube+cylinder skipped");
}

void test_constants_match_spec() {
  std::cout << "[test_constants_match_spec]\n";
  check(path_manager::msg::DynamicObstacleSpec::KIND_SPHERE == 0,
        "KIND_SPHERE == 0");
  check(path_manager::msg::DynamicObstacleSpec::KIND_CUBE == 1,
        "KIND_CUBE == 1");
  check(path_manager::msg::DynamicObstacleSpec::KIND_CYLINDER == 2,
        "KIND_CYLINDER == 2");
}

}  // namespace

int main() {
  test_empty_array();
  test_all_spheres();
  test_mixed_kinds();
  test_constants_match_spec();
  std::cout << "\n==== load_obstacles_test: passed=" << g_passed
            << " failed=" << g_failed << " ====\n";
  return g_failed == 0 ? 0 : 1;
}

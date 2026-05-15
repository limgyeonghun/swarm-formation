// V3 scenario harness for risk-aware A*.
// Builds a synthetic flat ESDF in memory (no terrain dependency), loads
// scenarios from yaml, runs A*, prints metrics per scenario.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "path_planner/dyn_a_star.h"
#include "path_planner/sdf/sdf_manager.h"

namespace {

using path_planner::astar::AStar;
using path_planner::astar::RiskZoneLite;
using path_planner::sdf::SDFManager;

int g_passed = 0;
int g_failed = 0;

void check(bool cond, const std::string &label) {
  std::cout << "  " << (cond ? "[OK]   " : "[FAIL] ")
            << std::left << std::setw(48) << label << "\n";
  if (cond) ++g_passed; else ++g_failed;
}

struct Scenario {
  std::string name;
  Eigen::Vector3d start, goal;
  std::vector<RiskZoneLite> zones;
  std::string expected;
};

struct MapSpec {
  Eigen::Vector3d size = Eigen::Vector3d(200, 200, 20);
  double voxel = 1.0;
  // If non-empty, load this ESDF file instead of building flat free-space.
  // Bounding box [bbox_lo, bbox_hi] is read from the file when set.
  std::string esdf_path;
  Eigen::Vector3d bbox_lo = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d bbox_hi = Eigen::Vector3d(1000, 1000, 100);
};

struct LoadResult {
  MapSpec map;
  std::vector<Scenario> scenarios;
};

LoadResult load(const std::string &path) {
  YAML::Node root = YAML::LoadFile(path);
  LoadResult out;
  if (root["map"]) {
    auto m = root["map"];
    if (m["size"]) {
      auto s = m["size"];
      out.map.size = Eigen::Vector3d(s[0].as<double>(),
                                     s[1].as<double>(),
                                     s[2].as<double>());
    }
    if (m["voxel"]) out.map.voxel = m["voxel"].as<double>();
    if (m["esdf"]) out.map.esdf_path = m["esdf"].as<std::string>();
    if (m["bbox_lo"]) {
      auto v = m["bbox_lo"];
      out.map.bbox_lo = Eigen::Vector3d(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
    }
    if (m["bbox_hi"]) {
      auto v = m["bbox_hi"];
      out.map.bbox_hi = Eigen::Vector3d(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
    }
  }
  for (const auto &n : root["scenarios"]) {
    Scenario s;
    s.name = n["name"].as<std::string>();
    auto a = n["start"];
    auto b = n["goal"];
    s.start = Eigen::Vector3d(a[0].as<double>(), a[1].as<double>(), a[2].as<double>());
    s.goal  = Eigen::Vector3d(b[0].as<double>(), b[1].as<double>(), b[2].as<double>());
    for (const auto &z : n["zones"]) {
      RiskZoneLite tz;
      auto c = z["center"];
      tz.center = Eigen::Vector3d(c[0].as<double>(), c[1].as<double>(), c[2].as<double>());
      tz.reach  = z["reach"].as<double>();
      tz.peak   = z["peak"].as<double>();
      s.zones.push_back(tz);
    }
    s.expected = n["expected"].as<std::string>();
    out.scenarios.push_back(s);
  }
  return out;
}

double pathLength(const std::vector<Eigen::Vector3d> &p) {
  double L = 0;
  for (size_t i = 1; i < p.size(); ++i) L += (p[i] - p[i-1]).norm();
  return L;
}

// Path-integrated risk metric: sum segment_len * risk(midpoint).
// Uses the same OR-moat formula as AStar::getRiskCost (with alpha=1).
double pathRisk(const std::vector<Eigen::Vector3d> &p,
                const std::vector<RiskZoneLite> &zones) {
  double total = 0.0;
  for (size_t i = 1; i < p.size(); ++i) {
    const auto &a = p[i-1];
    const auto &b = p[i];
    const Eigen::Vector3d m = 0.5 * (a + b);
    double survival = 1.0;
    for (const auto &tz : zones) {
      const double d = (m - tz.center).norm();
      if (d >= tz.reach) continue;
      const double u = 1.0 - d / tz.reach;
      const double moat = std::min(tz.peak * u * u, 1.0 - 1e-3);
      survival *= (1.0 - moat);
    }
    total += (b - a).norm() * (1.0 - survival);
  }
  return total;
}

bool buildFlatSDF(SDFManager &sdf, const MapSpec &map) {
  if (!sdf.initialize(map.voxel)) {
    std::cerr << "SDF init failed\n";
    return false;
  }
  if (!map.esdf_path.empty()) {
    if (!sdf.loadFromFile(map.esdf_path, map.bbox_lo, map.bbox_hi)) {
      std::cerr << "SDF loadFromFile failed: " << map.esdf_path << "\n";
      return false;
    }
    return true;
  }
  const int nx = static_cast<int>(map.size.x() / map.voxel);
  const int ny = static_cast<int>(map.size.y() / map.voxel);
  const int nz = static_cast<int>(map.size.z() / map.voxel);
  std::vector<uint8_t> occ(static_cast<size_t>(nx) * ny * nz, 0);
  Eigen::Vector3d origin(0, 0, 0);
  if (!sdf.buildFromVoxels(occ.data(), nx, ny, nz, origin)) {
    std::cerr << "SDF buildFromVoxels failed\n";
    return false;
  }
  return true;
}

void run(const Scenario &s, double alpha, double h_weight, const MapSpec &map) {
  std::cout << "\n[" << s.name << " alpha=" << alpha
            << " hw=" << h_weight << "]\n";

  SDFManager sdf;
  if (!buildFlatSDF(sdf, map)) {
    check(false, "SDF build");
    return;
  }

  AStar astar;
  Eigen::Vector3d origin(0, 0, 0);
  astar.setSDF(&sdf, origin, map.size, map.voxel);
  astar.setRiskZones(&s.zones);
  astar.setObstacleMargin(0.5);
  astar.setRiskAlpha(alpha);
  // Treat the CLI "h_weight" as transit_smha_w. detour mode runs as
  // strict anchor (smha_w = 1.0).
  astar.setDetourSmhaW(1.0);
  astar.setTransitSmhaW(h_weight);
  astar.setGoalInZoneThreshold(0.05);
  Eigen::Vector3i pool(
      static_cast<int>(map.size.x() / map.voxel),
      static_cast<int>(map.size.y() / map.voxel),
      static_cast<int>(map.size.z() / map.voxel));
  astar.initGridMap(pool);

  auto t0 = std::chrono::steady_clock::now();
  auto path = astar.astarSearchAndGetSimplePath(map.voxel, s.start, s.goal, 0);
  auto t1 = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  check(path.size() >= 2, "A* returned a path");
  if (path.size() < 2) return;

  double L = pathLength(path);
  double R = pathRisk(path, s.zones);
  double Ldirect = (s.goal - s.start).norm();

  // Deepest penetration: how far inside any zone any waypoint sits.
  double max_depth = 0.0;
  for (const auto &p : path) {
    for (const auto &tz : s.zones) {
      double d = (p - tz.center).norm();
      if (d < tz.reach) {
        double depth = tz.reach - d;
        if (depth > max_depth) max_depth = depth;
      }
    }
  }

  std::printf("  length=%.1fm direct=%.1fm risk_int=%.3f max_depth=%.1fm time=%.1fms wp=%zu\n",
              L, Ldirect, R, max_depth, ms, path.size());

  // DETOUR success = path stays mostly outside zones (low risk_int).
  // Threshold scales with direct length so it's resolution-independent.
  const double low_risk = 0.05 * Ldirect;
  if (s.expected == "DETOUR") {
    check(R < low_risk,        "DETOUR: risk_int < 5% of direct");
    check(max_depth < 5.0,     "DETOUR: max_depth < 5m");
  }
  if (s.expected == "TRANSIT") {
    // Forced transit: path must reach the goal and not explode.
    check(L < Ldirect * 2.0,   "TRANSIT: length < 2× direct");
    check(ms < 18000.0,        "TRANSIT: time < 18s (no timeout)");
  }
  if (s.expected == "APPROACH") {
    check(L < Ldirect * 1.5,   "APPROACH: length < 1.5× direct");
    check(ms < 18000.0,        "APPROACH: time < 18s (no timeout)");
  }
}

}  // namespace

int main(int argc, char **argv) {
  std::string yaml_path = (argc > 1)
      ? argv[1]
      : "src/mmp_terrain/data/risk_scenarios/synthetic_flat.yaml";

  LoadResult loaded;
  try {
    loaded = load(yaml_path);
  } catch (const std::exception &e) {
    std::cerr << "Failed to load yaml: " << e.what() << "\n";
    return 2;
  }
  std::cout << "Loaded " << loaded.scenarios.size()
            << " scenarios from " << yaml_path
            << "  (map " << loaded.map.size.transpose()
            << " voxel " << loaded.map.voxel << ")\n";

  std::vector<double> alphas = {0.1, 0.3, 1.0, 3.0, 10.0};
  if (argc > 2) alphas = {std::stod(argv[2])};

  double h_weight = (argc > 3) ? std::stod(argv[3]) : 1.0;

  for (double a : alphas) {
    for (const auto &s : loaded.scenarios) run(s, a, h_weight, loaded.map);
  }

  std::cout << "\n==== risk_scenarios_test: passed=" << g_passed
            << " failed=" << g_failed << " ====\n";
  return g_failed == 0 ? 0 : 1;
}

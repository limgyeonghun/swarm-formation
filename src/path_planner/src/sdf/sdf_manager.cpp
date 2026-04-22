// SDFManager: CPU-only signed Euclidean distance field using the
// Felzenszwalb-Huttenlocher separable transform with OpenMP parallelization,
// and a flat binary save/load format. No CUDA/NVBlox dependencies.

#include "path_planner/sdf/sdf_manager.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace path_planner {
namespace sdf {

namespace {

// One-dimensional squared-distance transform (Felzenszwalb-Huttenlocher).
// f_get(q) returns the seed value at grid index q (0 for obstacle, +inf otherwise).
// f_set(q, val) stores the resulting squared distance at index q.
inline void fillEDT1D(const std::function<double(int)>& f_get,
                      const std::function<void(int, double)>& f_set,
                      int start, int end) {
  const int len = end - start + 1;
  // Use start-indexed arrays (matches the original Felzenszwalb-Huttenlocher
  // formulation from main/GridMap::fillESDF).
  std::vector<int> v(len + start);
  std::vector<double> z(len + start + 1);

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; ++q) {
    ++k;
    double s;
    do {
      --k;
      double num = (f_get(q) + static_cast<double>(q) * q) -
                   (f_get(v[k]) + static_cast<double>(v[k]) * v[k]);
      double den = 2.0 * (q - v[k]);
      s = num / den;
    } while (s <= z[k]);
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;
  for (int q = start; q <= end; ++q) {
    while (z[k + 1] < q) ++k;
    double dq = q - v[k];
    f_set(q, dq * dq + f_get(v[k]));
  }
}

}  // namespace

struct SDFManagerImpl {
  double voxel_size = 0.0;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  int nx = 0, ny = 0, nz = 0;

  // Final signed distance in meters. +inf for out-of-map queries.
  // Flat layout: ((x * ny) + y) * nz + z.
  std::vector<float> distance_cache;

  bool initialized = false;
  bool has_data = false;

  inline size_t flatIdx(int xi, int yi, int zi) const {
    return ((size_t(xi) * ny) + yi) * nz + zi;
  }

  inline Eigen::Vector3d worldToVoxelF(const Eigen::Vector3d& p) const {
    return (p - origin) / voxel_size;
  }
};

SDFManager::SDFManager() : impl_(std::make_unique<SDFManagerImpl>()) {}
SDFManager::~SDFManager() = default;

bool SDFManager::initialize(double voxel_size) {
  if (voxel_size <= 0.0) {
    std::cerr << "[SDFManager] invalid voxel_size " << voxel_size << "\n";
    return false;
  }
  impl_->voxel_size = voxel_size;
  impl_->initialized = true;
  impl_->has_data = false;
  impl_->distance_cache.clear();
  return true;
}

bool SDFManager::buildFromVoxels(const uint8_t* occupancy,
                                  int nx, int ny, int nz,
                                  const Eigen::Vector3d& origin) {
  if (!impl_->initialized) {
    std::cerr << "[SDFManager] buildFromVoxels: not initialized\n";
    return false;
  }
  if (!occupancy || nx <= 0 || ny <= 0 || nz <= 0) {
    std::cerr << "[SDFManager] buildFromVoxels: invalid input\n";
    return false;
  }

  impl_->origin = origin;
  impl_->nx = nx;
  impl_->ny = ny;
  impl_->nz = nz;

  const size_t N = size_t(nx) * ny * nz;
  const double res = impl_->voxel_size;
  const double INF = std::numeric_limits<double>::max();

  auto idx = [nx_ = size_t(nx), ny_ = size_t(ny), nz_ = size_t(nz)]
             (int x, int y, int z) {
    return ((size_t(x) * ny_) + y) * nz_ + z;
  };

  // Special case: no obstacles at all. fillEDT1D with all-INF seeds produces
  // NaN (inf - inf in the parabola intersection). Fill the whole cache with
  // a large finite free-distance value and return.
  bool any_occupied = false;
  for (size_t i = 0; i < N; ++i) {
    if (occupancy[i] != 0) { any_occupied = true; break; }
  }
  if (!any_occupied) {
    const float kLargeFree = static_cast<float>(res) *
                             static_cast<float>(std::max({nx, ny, nz}));
    impl_->distance_cache.assign(N, kLargeFree);
    impl_->has_data = true;
    std::cerr << "[SDFManager] built (no obstacles): shape=(" << nx << ","
              << ny << "," << nz << ") voxel=" << res << " voxels=" << N
              << " free_distance=" << kLargeFree << "\n";
    return true;
  }

  // Positive DT on the occupied set (obstacles = 0, free = +inf).
  std::vector<double> tmp1(N), tmp2(N);
  std::vector<double> d_pos(N);

#ifdef _OPENMP
  const bool use_par = (N > 10000);
#else
  const bool use_par = false;
#endif

  // Sweep Z
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int x = 0; x < nx; ++x) {
    for (int y = 0; y < ny; ++y) {
      fillEDT1D(
          [&](int z) { return occupancy[idx(x, y, z)] != 0 ? 0.0 : INF; },
          [&](int z, double v) { tmp1[idx(x, y, z)] = v; },
          0, nz - 1);
    }
  }
  // Sweep Y
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int x = 0; x < nx; ++x) {
    for (int z = 0; z < nz; ++z) {
      fillEDT1D(
          [&](int y) { return tmp1[idx(x, y, z)]; },
          [&](int y, double v) { tmp2[idx(x, y, z)] = v; },
          0, ny - 1);
    }
  }
  // Sweep X (final, take sqrt in meters)
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int y = 0; y < ny; ++y) {
    for (int z = 0; z < nz; ++z) {
      fillEDT1D(
          [&](int x) { return tmp2[idx(x, y, z)]; },
          [&](int x, double v) {
            d_pos[idx(x, y, z)] = res * std::sqrt(v);
          },
          0, nx - 1);
    }
  }

  // Negative DT on the complement (free voxels become obstacles).
  std::vector<double> d_neg(N);
#pragma omp parallel for schedule(static) if(use_par)
  for (size_t i = 0; i < N; ++i) tmp1[i] = 0.0;
#pragma omp parallel for schedule(static) if(use_par)
  for (size_t i = 0; i < N; ++i) tmp2[i] = 0.0;

#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int x = 0; x < nx; ++x) {
    for (int y = 0; y < ny; ++y) {
      fillEDT1D(
          [&](int z) { return occupancy[idx(x, y, z)] == 0 ? 0.0 : INF; },
          [&](int z, double v) { tmp1[idx(x, y, z)] = v; },
          0, nz - 1);
    }
  }
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int x = 0; x < nx; ++x) {
    for (int z = 0; z < nz; ++z) {
      fillEDT1D(
          [&](int y) { return tmp1[idx(x, y, z)]; },
          [&](int y, double v) { tmp2[idx(x, y, z)] = v; },
          0, ny - 1);
    }
  }
#pragma omp parallel for collapse(2) schedule(static) if(use_par)
  for (int y = 0; y < ny; ++y) {
    for (int z = 0; z < nz; ++z) {
      fillEDT1D(
          [&](int x) { return tmp2[idx(x, y, z)]; },
          [&](int x, double v) {
            d_neg[idx(x, y, z)] = res * std::sqrt(v);
          },
          0, nx - 1);
    }
  }

  // Signed distance: positive outside obstacles, negative inside.
  // For a free voxel: d_pos > 0, d_neg = 0 -> signed = +d_pos.
  // For an obstacle:  d_pos = 0, d_neg > 0 -> signed = -d_neg.
  impl_->distance_cache.assign(N, std::numeric_limits<float>::infinity());
#pragma omp parallel for schedule(static) if(use_par)
  for (size_t i = 0; i < N; ++i) {
    double signed_d = (occupancy[i] != 0) ? -d_neg[i] : d_pos[i];
    impl_->distance_cache[i] = static_cast<float>(signed_d);
  }

  impl_->has_data = true;
  std::cerr << "[SDFManager] built: shape=(" << nx << "," << ny << "," << nz
            << ") voxel=" << res << " voxels=" << N << "\n";
  return true;
}

namespace {

// Single-file binary format: magic + header + flat float distance array.
// Layout:
//   uint32  magic          = 'MESD'
//   uint32  version        = 1
//   double  voxel_size
//   double  origin_x, y, z
//   int32   nx, ny, nz
//   float[] distance_cache (nx*ny*nz)
struct EsdfHeader {
  uint32_t magic = 0x4D455344;  // "MESD"
  uint32_t version = 1;
  double voxel_size = 0.0;
  double origin_x = 0.0, origin_y = 0.0, origin_z = 0.0;
  int32_t nx = 0, ny = 0, nz = 0;
};

}  // namespace

bool SDFManager::saveToFile(const std::string& path) const {
  if (!impl_->initialized || !impl_->has_data) {
    std::cerr << "[SDFManager] saveToFile: no data\n";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    std::cerr << "[SDFManager] saveToFile: cannot open " << path << "\n";
    return false;
  }

  EsdfHeader h;
  h.voxel_size = impl_->voxel_size;
  h.origin_x = impl_->origin.x();
  h.origin_y = impl_->origin.y();
  h.origin_z = impl_->origin.z();
  h.nx = impl_->nx;
  h.ny = impl_->ny;
  h.nz = impl_->nz;
  f.write(reinterpret_cast<const char*>(&h), sizeof(h));

  const size_t N = size_t(h.nx) * h.ny * h.nz;
  f.write(reinterpret_cast<const char*>(impl_->distance_cache.data()),
          N * sizeof(float));
  if (!f) {
    std::cerr << "[SDFManager] saveToFile: write failed\n";
    return false;
  }

  std::cerr << "[SDFManager] saved " << path << " shape=(" << h.nx << ","
            << h.ny << "," << h.nz << ") voxel=" << h.voxel_size
            << " bytes=" << (sizeof(h) + N * sizeof(float)) << "\n";
  return true;
}

bool SDFManager::loadFromFile(const std::string& path,
                               const Eigen::Vector3d& /*bbox_lo*/,
                               const Eigen::Vector3d& /*bbox_hi*/) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    std::cerr << "[SDFManager] loadFromFile: cannot open " << path << "\n";
    return false;
  }

  EsdfHeader h;
  f.read(reinterpret_cast<char*>(&h), sizeof(h));
  if (!f || h.magic != 0x4D455344 || h.version != 1) {
    std::cerr << "[SDFManager] loadFromFile: bad header (magic=" << std::hex
              << h.magic << std::dec << ", version=" << h.version << ")\n";
    return false;
  }
  if (h.nx <= 0 || h.ny <= 0 || h.nz <= 0 || h.voxel_size <= 0.0) {
    std::cerr << "[SDFManager] loadFromFile: invalid dims/voxel\n";
    return false;
  }

  if (!impl_->initialized || impl_->voxel_size != h.voxel_size) {
    initialize(h.voxel_size);
  }

  impl_->origin = Eigen::Vector3d(h.origin_x, h.origin_y, h.origin_z);
  impl_->nx = h.nx;
  impl_->ny = h.ny;
  impl_->nz = h.nz;

  const size_t N = size_t(h.nx) * h.ny * h.nz;
  impl_->distance_cache.assign(N, std::numeric_limits<float>::infinity());
  f.read(reinterpret_cast<char*>(impl_->distance_cache.data()),
         N * sizeof(float));
  if (!f) {
    std::cerr << "[SDFManager] loadFromFile: read failed (expected "
              << (N * sizeof(float)) << " bytes)\n";
    return false;
  }

  impl_->has_data = true;
  std::cerr << "[SDFManager] loaded " << path << " shape=(" << h.nx << ","
            << h.ny << "," << h.nz << ") voxel=" << h.voxel_size << "\n";
  return true;
}

float SDFManager::getDistance(const Eigen::Vector3d& pos) const {
  if (!impl_->initialized || !impl_->has_data) {
    return std::numeric_limits<float>::infinity();
  }
  Eigen::Vector3d vf = impl_->worldToVoxelF(pos);
  int xi = int(std::floor(vf.x()));
  int yi = int(std::floor(vf.y()));
  int zi = int(std::floor(vf.z()));
  if (xi < 0 || yi < 0 || zi < 0 ||
      xi >= impl_->nx || yi >= impl_->ny || zi >= impl_->nz) {
    return std::numeric_limits<float>::infinity();
  }
  return impl_->distance_cache[impl_->flatIdx(xi, yi, zi)];
}

bool SDFManager::getDistanceAndGradient(const Eigen::Vector3d& pos,
                                         float* distance,
                                         Eigen::Vector3d* gradient) const {
  if (distance) *distance = std::numeric_limits<float>::infinity();
  if (gradient) gradient->setZero();

  if (!impl_->initialized || !impl_->has_data) return false;

  float d_c = getDistance(pos);
  if (!std::isfinite(d_c)) return false;
  if (distance) *distance = d_c;

  const double h = impl_->voxel_size;
  float dx_p = getDistance(pos + Eigen::Vector3d(h, 0, 0));
  float dx_m = getDistance(pos - Eigen::Vector3d(h, 0, 0));
  float dy_p = getDistance(pos + Eigen::Vector3d(0, h, 0));
  float dy_m = getDistance(pos - Eigen::Vector3d(0, h, 0));
  float dz_p = getDistance(pos + Eigen::Vector3d(0, 0, h));
  float dz_m = getDistance(pos - Eigen::Vector3d(0, 0, h));

  auto safe = [](float v, float fb) { return std::isfinite(v) ? v : fb; };
  dx_p = safe(dx_p, d_c); dx_m = safe(dx_m, d_c);
  dy_p = safe(dy_p, d_c); dy_m = safe(dy_m, d_c);
  dz_p = safe(dz_p, d_c); dz_m = safe(dz_m, d_c);

  double inv_2h = 1.0 / (2.0 * h);
  if (gradient) {
    (*gradient) << (dx_p - dx_m) * inv_2h,
                   (dy_p - dy_m) * inv_2h,
                   (dz_p - dz_m) * inv_2h;
  }
  return true;
}

bool SDFManager::isInitialized() const { return impl_->initialized; }
bool SDFManager::hasData() const { return impl_->has_data; }
double SDFManager::voxelSize() const { return impl_->voxel_size; }

size_t SDFManager::numAllocatedBlocks() const {
  // Retained for API compatibility. No block concept here; report voxel count.
  return impl_->distance_cache.size();
}

}  // namespace sdf
}  // namespace path_planner

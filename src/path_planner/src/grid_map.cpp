#include "path_planner/grid_map.h"

void GridMap::initMap(const Eigen::Vector3d& map_size, double resolution) {
  mp_.resolution_ = resolution;
  mp_.resolution_inv_ = 1.0 / resolution;
  mp_.map_origin_ = Eigen::Vector3d(-map_size(0) / 2.0, -map_size(1) / 2.0, 0.0);
  mp_.map_size_ = map_size;
  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  md_.occupancy_buffer_.resize(buffer_size, 0.0);
  md_.occupancy_buffer_inflate_.resize(buffer_size, 0);
  md_.distance_buffer_.resize(buffer_size, 10000.0);
  md_.distance_buffer_neg_.resize(buffer_size, 10000.0);
  md_.distance_buffer_all_.resize(buffer_size, 10000.0);
  md_.tmp_buffer1_.resize(buffer_size, 0.0);
  md_.tmp_buffer2_.resize(buffer_size, 0.0);

  std::cout << "Map initialized with size: " << mp_.map_voxel_num_.transpose() << " (" << buffer_size << " voxels)" << std::endl;
}

void GridMap::setStaticMap(const std::vector<double>& static_occupancy) {
  if (static_occupancy.size() != md_.occupancy_buffer_.size()) {
    std::cerr << "Error: Static map size (" << static_occupancy.size() 
              << ") does not match buffer size (" << md_.occupancy_buffer_.size() << ")!" << std::endl;
    return;
  }

  md_.occupancy_buffer_ = static_occupancy;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x) {
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y) {
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        int idx = toAddress(x, y, z);
        if (md_.occupancy_buffer_[idx] > 0.5) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step);
        }
      }
    }
  }

  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = 0; x < mp_.map_voxel_num_(0); ++x) {
      for (int y = 0; y < mp_.map_voxel_num_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
    }
  }
}

void GridMap::setOccupancy(const Eigen::Vector3i& id, double occ) {
  if (!isInMap(id)) return;
  if (occ != 0 && occ != 1) {
    std::cerr << "Occupancy value must be 0 or 1!" << std::endl;
    return;
  }
  md_.occupancy_buffer_[toAddress(id)] = occ;
}

void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step) {
  for (int x = -step; x <= step; ++x) {
    for (int y = -step; y <= step; ++y) {
      for (int z = -step; z <= step; ++z) {
        Eigen::Vector3i inf_pt = pt + Eigen::Vector3i(x, y, z);
        if (isInMap(inf_pt)) {
          md_.occupancy_buffer_inflate_[toAddress(inf_pt)] = 1;
        }
      }
    }
  }
}

template <typename F_get_val, typename F_set_val>
void GridMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  std::vector<int> v(mp_.map_voxel_num_(dim));
  std::vector<double> z(mp_.map_voxel_num_(dim) + 1);

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; ++q) {
    k++;
    double s;
    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);
    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;
  for (int q = start; q <= end; ++q) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void GridMap::updateESDF3d() {
  Eigen::Vector3i min_esdf(0, 0, 0);
  Eigen::Vector3i max_esdf = mp_.map_voxel_num_ - Eigen::Vector3i(1, 1, 1);

  // Positive ESDF
  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
      fillESDF(
          [&](int z) { return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max(); },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; },
          min_esdf[2], max_esdf[2], 2);
    }
  }
  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; ++z) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; },
               min_esdf[1], max_esdf[1], 1);
    }
  }
  for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; ++z) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) { md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val); },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  // Negative ESDF
  md_.occupancy_buffer_neg_.resize(md_.occupancy_buffer_.size(), 0);
  for (int x = min_esdf(0); x <= max_esdf(0); ++x) {
    for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_neg_[idx] = (md_.occupancy_buffer_inflate_[idx] == 0) ? 1 : 0;
      }
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
      fillESDF(
          [&](int z) { return md_.occupancy_buffer_neg_[toAddress(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max(); },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; },
          min_esdf[2], max_esdf[2], 2);
    }
  }
  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; ++z) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; },
               min_esdf[1], max_esdf[1], 1);
    }
  }
  for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; ++z) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) { md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val); },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  // Combine Positive and Negative
  for (int x = min_esdf(0); x <= max_esdf(0); ++x) {
    for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];
        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
      }
    }
  }

  std::cout << "ESDF updated for static map" << std::endl;
}

void GridMap::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff) {
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        indexToPos(current_idx, pts[x][y][z]);
      }
    }
  }
}

void GridMap::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        dists[x][y][z] = getDistance(pts[x][y][z]);
      }
    }
  }
}

void GridMap::interpolateTrilinearEDT(double values[2][2][2], const Eigen::Vector3d& diff, double& value) {
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;
  value = (1 - diff(2)) * v0 + diff(2) * v1;
}

void GridMap::interpolateTrilinearFirstGrad(double values[2][2][2],
                                            const Eigen::Vector3d &diff,
                                            Eigen::Vector3d &grad)
{
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;                          // e
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;                          // f

  grad[2] = (v1 - v0) * mp_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_.resolution_inv_;
}

void GridMap::evaluateEDT(const Eigen::Vector3d& pos, double& dist) {
  if (!isInMap(pos)) {
    dist = 10000.0;
    return;
  }
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearEDT(dists, diff, dist);
}

void GridMap::evaluateFirstGrad(const Eigen::Vector3d &pos,
                                Eigen::Vector3d &grad)
{
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearFirstGrad(dists, diff, grad);
}
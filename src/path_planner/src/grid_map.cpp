#include "path_planner/grid_map.h"
#include <rclcpp/rclcpp.hpp>

void GridMap::initMap(const std::shared_ptr<rclcpp::Node>& node) {
  node_ = node;

  /* get parameter */
  node_->declare_parameter("grid_map/resolution", -1.0);
  node_->declare_parameter("grid_map/map_size_x", -1.0);
  node_->declare_parameter("grid_map/map_size_y", -1.0);
  node_->declare_parameter("grid_map/map_size_z", -1.0);
  node_->declare_parameter("grid_map/obstacles_inflation", 0.1);
  node_->declare_parameter("grid_map/virtual_ceil_height", -0.1);
  node_->declare_parameter("grid_map/local_map_margin", 1);
  node_->declare_parameter("grid_map/frame_id", std::string("world"));
  node_->declare_parameter("grid_map/esdf_slice_height", -0.1);
  node_->declare_parameter("grid_map/show_esdf_time", false);
  node_->declare_parameter("grid_map/local_bound_inflate", 1.0);
  
  // Road boundary parameters
  node_->declare_parameter("grid_map/use_road_boundary", false);
  node_->declare_parameter("grid_map/road_segments", std::vector<double>());
  node_->declare_parameter("grid_map/road_width", 8.0);
  node_->declare_parameter("grid_map/road_margin", 0.5);

  mp_.resolution_ = node_->get_parameter("grid_map/resolution").as_double();
  double x_size = node_->get_parameter("grid_map/map_size_x").as_double();
  double y_size = node_->get_parameter("grid_map/map_size_y").as_double();
  double z_size = node_->get_parameter("grid_map/map_size_z").as_double();
  mp_.obstacles_inflation_ = node_->get_parameter("grid_map/obstacles_inflation").as_double();
  mp_.virtual_ceil_height_ = node_->get_parameter("grid_map/virtual_ceil_height").as_double();
  mp_.local_map_margin_ = node_->get_parameter("grid_map/local_map_margin").as_int();
  mp_.frame_id_ = node_->get_parameter("grid_map/frame_id").as_string();
  mp_.esdf_slice_height_ = node_->get_parameter("grid_map/esdf_slice_height").as_double();
  mp_.show_esdf_time_ = node_->get_parameter("grid_map/show_esdf_time").as_bool();
  mp_.local_bound_inflate_ = node_->get_parameter("grid_map/local_bound_inflate").as_double();
  
  // Road boundary parameters
  mp_.use_road_boundary_ = node_->get_parameter("grid_map/use_road_boundary").as_bool();
  mp_.road_width_ = node_->get_parameter("grid_map/road_width").as_double();
  mp_.road_margin_ = node_->get_parameter("grid_map/road_margin").as_double();

  auto road_segments = node_->get_parameter("grid_map/road_segments").as_double_array();
  if (road_segments.size() % 5 == 0) {
    for (size_t i = 0; i < road_segments.size(); i += 5) {
      RoadSegment segment;
      segment.start_x = road_segments[i];
      segment.start_y = road_segments[i + 1];
      segment.end_x = road_segments[i + 2];
      segment.end_y = road_segments[i + 3];
      segment.width = road_segments[i + 4];
      mp_.road_segments_.push_back(segment);
    }
  } else
    RCLCPP_WARN(node_->get_logger(), "Invalid road segments data. Each segment should have 5 values (including width).");

  std::cout << "GridMap parameters: " << std::endl;
  std::cout << "  resolution: " << mp_.resolution_ << std::endl;
  std::cout << "  map_size_x: " << x_size << std::endl;
  std::cout << "  map_size_y: " << y_size << std::endl;
  std::cout << "  map_size_z: " << z_size << std::endl;
  std::cout << "  obstacles_inflation: " << mp_.obstacles_inflation_ << std::endl;
  std::cout << "  virtual_ceil_height: " << mp_.virtual_ceil_height_ << std::endl;
  std::cout << "  local_map_margin: " << mp_.local_map_margin_ << std::endl;
  std::cout << "  frame_id: " << mp_.frame_id_ << std::endl;
  std::cout << "  esdf_slice_height: " << mp_.esdf_slice_height_ << std::endl;
  std::cout << "  show_esdf_time: " << mp_.show_esdf_time_ << std::endl;
  std::cout << "  local_bound_inflate: " << mp_.local_bound_inflate_ << std::endl;
  std::cout << "  use_road_boundary: " << mp_.use_road_boundary_ << std::endl;
  std::cout << "  road_width: " << mp_.road_width_ << std::endl;
  std::cout << "  road_segments: " << mp_.road_segments_.size() << " segments" << std::endl;
  for (size_t i = 0; i < mp_.road_segments_.size(); ++i) {
    const auto& seg = mp_.road_segments_[i];
    std::cout << "    segment " << i << ": (" << seg.start_x << "," << seg.start_y 
              << ") -> (" << seg.end_x << "," << seg.end_y << "), width: " << seg.width << "m" << std::endl;
  }
  std::cout << "  road_margin: " << mp_.road_margin_ << std::endl;

  mp_.local_bound_inflate_ = std::max(mp_.resolution_, mp_.local_bound_inflate_);
  mp_.resolution_inv_ = 1.0 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, -0.01);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  if (mp_.virtual_ceil_height_ >= z_size) {
    mp_.virtual_ceil_height_ = z_size - mp_.resolution_;
  }

  // int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  md_.occupancy_buffer_.resize(buffer_size, 0.0);
  md_.occupancy_buffer_inflate_.resize(buffer_size, 0);
  md_.occupancy_buffer_neg_.resize(buffer_size, 0);
  md_.distance_buffer_.resize(buffer_size, 10000.0);
  md_.distance_buffer_neg_.resize(buffer_size, 10000.0);
  md_.distance_buffer_all_.resize(buffer_size, 10000.0);
  md_.tmp_buffer1_.resize(buffer_size, 0.0);
  md_.tmp_buffer2_.resize(buffer_size, 0.0);

  distance_buffer_local_.clear();
  local_esdf_min_ = Eigen::Vector3i(0, 0, 0);
  local_esdf_max_ = Eigen::Vector3i(0, 0, 0);
}

void GridMap::setStaticMap(const std::vector<double>& static_occupancy) {
  if (static_occupancy.size() != md_.occupancy_buffer_.size()) {
    RCLCPP_ERROR(node_->get_logger(), "Static map size (%zu) does not match buffer size (%zu)!",
                 static_occupancy.size(), md_.occupancy_buffer_.size());
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
  updateESDF3d(Eigen::Vector3i(0,0,0), mp_.map_voxel_num_ - Eigen::Vector3i(1,1,1));
  RCLCPP_INFO(node_->get_logger(), "Full ESDF computed after setting static map.");
  md_.esdf_need_update_ = true;
}

void GridMap::setOccupancy(const Eigen::Vector3i& id, double occ) {
  if (!isInMap(id)) return;
  if (occ != 0 && occ != 1) {
    RCLCPP_ERROR(node_->get_logger(), "Occupancy value must be 0 or 1!");
    return;
  }
  md_.occupancy_buffer_[toAddress(id)] = occ;
  md_.esdf_need_update_ = true;
}

// void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step) {
//   for (int x = -step; x <= step; ++x) {
//     for (int y = -step; y <= step; ++y) {
//       for (int z = -step; z <= step; ++z) {
//         Eigen::Vector3i inf_pt = pt + Eigen::Vector3i(x, y, z);
//         if (isInMap(inf_pt)) {
//           md_.occupancy_buffer_inflate_[toAddress(inf_pt)] = 1;
//         }
//       }
//     }
//   }
// }

void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step) {
    const int z_idx = std::max(pt.z() - step, 0);
    const int x_min = std::max(pt.x() - step, 0);
    const int x_max = std::min(pt.x() + step, mp_.map_voxel_num_(0) - 1);
    const int y_min = std::max(pt.y() - step, 0);
    const int y_max = std::min(pt.y() + step, mp_.map_voxel_num_(1) - 1);

    // Memory access optimization: access contiguous memory regions
    for (int x = x_min; x <= x_max; ++x) {
        for (int y = y_min; y <= y_max; ++y) {
            Eigen::Vector3i inf_pt(x, y, z_idx);
            
            // Check if point is within road boundary before inflating
            if (mp_.use_road_boundary_) {
                Eigen::Vector3d pos;
                indexToPos(inf_pt, pos);
                if (!isInRoadBoundary(pos)) {
                    md_.occupancy_buffer_inflate_[toAddress(inf_pt)] = 1;
                    continue;
                }
            }
            
            md_.occupancy_buffer_inflate_[toAddress(inf_pt)] = 1;
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

void GridMap::updateESDF3d(const Eigen::Vector3i &min_esdf, const Eigen::Vector3i &max_esdf) {
  for (int i = 0; i < 3; ++i) {
    if (min_esdf[i] < 0 || max_esdf[i] >= mp_.map_voxel_num_[i]) {
      RCLCPP_ERROR(node_->get_logger(), "Out of bounds! Dim %d: min=%d, max=%d, voxel_num=%d",
                   i, min_esdf[i], max_esdf[i], mp_.map_voxel_num_[i]);
      return;
    }
  }

  auto start_total = rclcpp::Clock().now();

  Eigen::Vector3i esdf_voxel_size = max_esdf - min_esdf + Eigen::Vector3i(1, 1, 1);
  int esdf_voxel_count = esdf_voxel_size(0) * esdf_voxel_size(1) * esdf_voxel_size(2);

  // Disable parallelization for small data (prevent overhead)
  bool use_parallel = esdf_voxel_count > 10000; // Adjustable threshold

  // RCLCPP_INFO(node_->get_logger(), "ESDF processing voxel size: %d %d %d (%d voxels), parallel=%s",
  //             esdf_voxel_size(0), esdf_voxel_size(1), esdf_voxel_size(2), esdf_voxel_count, 
  //             use_parallel ? "true" : "false");

  /* ========== compute positive DT ========== */
  auto start_positive = rclcpp::Clock().now();
  
  if (use_parallel) {
    #pragma omp parallel for collapse(2) schedule(static)
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
          [&](int z) {
            int idx = toAddress(x, y, z);
            if (mp_.use_road_boundary_) {
              Eigen::Vector3d pos;
              indexToPos(Eigen::Vector3i(x, y, z), pos);
              if (!isInRoadBoundary(pos)) {
                return 0.0;  // Road boundary is treated as obstacle
              }
            }
            return md_.occupancy_buffer_inflate_[idx] == 1 ?
                   0 : std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; },
          min_esdf[2], max_esdf[2], 2);
      }
    }

    #pragma omp parallel for collapse(2) schedule(static)
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
                 [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; },
                 min_esdf[1], max_esdf[1], 1);
      }
    }

    #pragma omp parallel for collapse(2) schedule(static)
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
                 [&](int x, double val) {
                   md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 },
                 min_esdf[0], max_esdf[0], 0);
      }
    }
  } else {
    // Sequential processing (for small data)
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
          [&](int z) {
            int idx = toAddress(x, y, z);
            if (mp_.use_road_boundary_) {
              Eigen::Vector3d pos;
              indexToPos(Eigen::Vector3i(x, y, z), pos);
              if (!isInRoadBoundary(pos)) {
                return 0.0;  // Road boundary is treated as obstacle
              }
            }
            return md_.occupancy_buffer_inflate_[idx] == 1 ?
                   0 : std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; },
          min_esdf[2], max_esdf[2], 2);
      }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
                 [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; },
                 min_esdf[1], max_esdf[1], 1);
      }
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
                 [&](int x, double val) {
                   md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 },
                 min_esdf[0], max_esdf[0], 0);
      }
    }
  }

  auto end_positive = rclcpp::Clock().now();

  /* ========== compute negative DT ========== */
  auto start_negative = rclcpp::Clock().now();

  if (use_parallel) {
    #pragma omp parallel for collapse(3) schedule(static)
    for (int x = min_esdf(0); x <= max_esdf(0); ++x) {
      for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress(x, y, z);
          md_.occupancy_buffer_neg_[idx] = (md_.occupancy_buffer_inflate_[idx] == 0) ? 1 : 0;
        }
      }
    }

    #pragma omp parallel for schedule(static)
    for (size_t i = 0; i < md_.tmp_buffer1_.size(); ++i) md_.tmp_buffer1_[i] = 0.0;
    #pragma omp parallel for schedule(static)
    for (size_t i = 0; i < md_.tmp_buffer2_.size(); ++i) md_.tmp_buffer2_[i] = 0.0;

    #pragma omp parallel for collapse(2) schedule(static)
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_.occupancy_buffer_neg_[toAddress(x, y, z)] == 1 ?
                     0 : std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; },
            min_esdf[2], max_esdf[2], 2);
      }
    }

    #pragma omp parallel for collapse(2) schedule(static)
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
                 [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; },
                 min_esdf[1], max_esdf[1], 1);
      }
    }

    #pragma omp parallel for collapse(2) schedule(static)
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
                 [&](int x, double val) {
                   md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 },
                 min_esdf[0], max_esdf[0], 0);
      }
    }
  } else {
    // Sequential processing (for small data)
    for (int x = min_esdf(0); x <= max_esdf(0); ++x) {
      for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress(x, y, z);
          md_.occupancy_buffer_neg_[idx] = (md_.occupancy_buffer_inflate_[idx] == 0) ? 1 : 0;
        }
      }
    }

    for (size_t i = 0; i < md_.tmp_buffer1_.size(); ++i) md_.tmp_buffer1_[i] = 0.0;
    for (size_t i = 0; i < md_.tmp_buffer2_.size(); ++i) md_.tmp_buffer2_[i] = 0.0;

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_.occupancy_buffer_neg_[toAddress(x, y, z)] == 1 ?
                     0 : std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; },
            min_esdf[2], max_esdf[2], 2);
      }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
                 [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; },
                 min_esdf[1], max_esdf[1], 1);
      }
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
                 [&](int x, double val) {
                   md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 },
                 min_esdf[0], max_esdf[0], 0);
      }
    }
  }

  auto end_negative = rclcpp::Clock().now();

  /* ========== combine pos and neg DT ========== */
  auto start_combine = rclcpp::Clock().now();

  if (use_parallel) {
    #pragma omp parallel for collapse(3) schedule(static)
    for (int x = min_esdf(0); x <= max_esdf(0); ++x) {
      for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress(x, y, z);
          double v = md_.distance_buffer_[idx];
          double vn = md_.distance_buffer_neg_[idx];
          md_.distance_buffer_all_[idx] = (vn > 0.0) ? (v - vn + mp_.resolution_ + v) : v;
        }
      }
    }
  } else {
    for (int x = min_esdf(0); x <= max_esdf(0); ++x) {
      for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress(x, y, z);
          double v = md_.distance_buffer_[idx];
          double vn = md_.distance_buffer_neg_[idx];
          md_.distance_buffer_all_[idx] = (vn > 0.0) ? (v - vn + mp_.resolution_ + v) : v;
        }
      }
    }
  }

  auto end_combine = rclcpp::Clock().now();
  auto end_total = rclcpp::Clock().now();

  /* ========== timing result output ========== */
  double total_duration = (end_total - start_total).seconds() * 1000.0;
  double positive_duration = (end_positive - start_positive).seconds() * 1000.0;
  double negative_duration = (end_negative - start_negative).seconds() * 1000.0;
  double combine_duration = (end_combine - start_combine).seconds() * 1000.0;
  double avg_esdf_time = total_duration / static_cast<double>(esdf_voxel_count);
  double max_esdf_time = std::max({positive_duration, negative_duration, combine_duration});

  // Only log ESDF timing if explicitly enabled and significant time spent
  if (mp_.show_esdf_time_ && total_duration > 10.0) {
    RCLCPP_INFO(node_->get_logger(),
                "voxels=%d, total=%.2f ms, positive=%.2f ms, negative=%.2f ms, combine=%.2f ms, avg=%.2f ms, max=%.2f ms",
                esdf_voxel_count, total_duration, positive_duration, negative_duration,
                combine_duration, avg_esdf_time, max_esdf_time);
  }

  md_.esdf_time_ += total_duration / 1000.0;
  md_.max_esdf_time_ = std::max(md_.max_esdf_time_, total_duration / 1000.0);
  md_.update_num_++;
}

void GridMap::updateESDF3d() {
  updateESDF3d(Eigen::Vector3i(0, 0, 0), mp_.map_voxel_num_ - Eigen::Vector3i(1, 1, 1));
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

void GridMap::interpolateTrilinearFirstGrad(double values[2][2][2], const Eigen::Vector3d& diff, Eigen::Vector3d& grad) {
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

  grad[2] = (v1 - v0) * mp_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_.resolution_inv_;
}

void GridMap::updateESDFLocal(const Eigen::Vector3d& center_pos) {
  rclcpp::Time t1 = rclcpp::Clock().now();

  Eigen::Vector3i center_idx;
  posToIndex(center_pos, center_idx);
  int inf = std::ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  local_esdf_min_ = center_idx - Eigen::Vector3i(inf, inf, inf);
  local_esdf_max_ = center_idx + Eigen::Vector3i(inf, inf, inf);
  boundIndex(local_esdf_min_);
  boundIndex(local_esdf_max_);

  // Local ESDF update optimization: update only small region
  updateESDF3d(local_esdf_min_, local_esdf_max_);

  Eigen::Vector3i local_size = local_esdf_max_ - local_esdf_min_ + Eigen::Vector3i(1, 1, 1);
  int local_buffer_size = local_size(0) * local_size(1) * local_size(2);
  distance_buffer_local_.resize(local_buffer_size);
  
  // Memory copy optimization
  int local_idx = 0;
  for (int x = local_esdf_min_(0); x <= local_esdf_max_(0); ++x) {
    for (int y = local_esdf_min_(1); y <= local_esdf_max_(1); ++y) {
      for (int z = local_esdf_min_(2); z <= local_esdf_max_(2); ++z) {
        int global_idx = toAddress(x, y, z);
        distance_buffer_local_[local_idx++] = md_.distance_buffer_all_[global_idx];
      }
    }
  }

  rclcpp::Time t2 = rclcpp::Clock().now();
  if (mp_.show_esdf_time_) {
    RCLCPP_INFO(node_->get_logger(), "Local ESDF update: %.3f ms", (t2 - t1).seconds() * 1000.0);
  }
}

void GridMap::evaluateEDT(const Eigen::Vector3d& pos, double& dist) {
  if (!isInMap(pos)) {
    dist = 10000.0;
    return;
  }

  Eigen::Vector3i idx;
  posToIndex(pos, idx);

  if (idx(0) >= local_esdf_min_(0) && idx(0) <= local_esdf_max_(0) &&
      idx(1) >= local_esdf_min_(1) && idx(1) <= local_esdf_max_(1) &&
      idx(2) >= local_esdf_min_(2) && idx(2) <= local_esdf_max_(2)) {
    
    Eigen::Vector3i local_size = local_esdf_max_ - local_esdf_min_ + Eigen::Vector3i(1, 1, 1);
    int local_idx = (idx(0) - local_esdf_min_(0)) * local_size(1) * local_size(2) +
                    (idx(1) - local_esdf_min_(1)) * local_size(2) +
                    (idx(2) - local_esdf_min_(2));
    
    dist = distance_buffer_local_[local_idx];
    // std::cout << "Local ESDF distance: " << dist << std::endl;
    return;
  }

  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearEDT(dists, diff, dist);
}

void GridMap::evaluateFirstGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearFirstGrad(dists, diff, grad);
}
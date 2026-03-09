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
  node_->declare_parameter("grid_map/ground_height", 0.0);
  node_->declare_parameter("grid_map/local_map_margin", 1);
  node_->declare_parameter("grid_map/frame_id", std::string("map"));
  node_->declare_parameter("grid_map/esdf_slice_height", -0.1);
  node_->declare_parameter("grid_map/show_esdf_time", false);
  node_->declare_parameter("grid_map/local_bound_inflate", 1.0);
  node_->declare_parameter("grid_map/map_origin_x", 0.0);
  node_->declare_parameter("grid_map/map_origin_y", 0.0);

  // Road boundary parameters
  node_->declare_parameter("grid_map/use_road_boundary", false);
  node_->declare_parameter("grid_map/road_segments", std::vector<double>());
  node_->declare_parameter("grid_map/road_width", 8.0);
  node_->declare_parameter("grid_map/road_margin", 0.5);

  // Threat zone parameters
  node_->declare_parameter("grid_map/use_threat_zones", false);
  node_->declare_parameter("grid_map/threat_cost_weight", 1.0);
  node_->declare_parameter("threat_zones", std::vector<double>());

  // Terrain gridmap parameters
  node_->declare_parameter("grid_map/use_terrain_obstacles", false);
  node_->declare_parameter("grid_map/terrain_obstacle_threshold", 0.0);
  node_->declare_parameter("grid_map/terrain_target_cell_size", 50.0);

  // Visualization parameters
  node_->declare_parameter("grid_map/publish_global_esdf_viz", false);

  mp_.resolution_ = node_->get_parameter("grid_map/resolution").as_double();
  double x_size = node_->get_parameter("grid_map/map_size_x").as_double();
  double y_size = node_->get_parameter("grid_map/map_size_y").as_double();
  double z_size = node_->get_parameter("grid_map/map_size_z").as_double();
  mp_.obstacles_inflation_ = node_->get_parameter("grid_map/obstacles_inflation").as_double();
  mp_.virtual_ceil_height_ = node_->get_parameter("grid_map/virtual_ceil_height").as_double();
  mp_.ground_height_ = node_->get_parameter("grid_map/ground_height").as_double();
  mp_.local_map_margin_ = node_->get_parameter("grid_map/local_map_margin").as_int();
  mp_.frame_id_ = node_->get_parameter("grid_map/frame_id").as_string();
  mp_.esdf_slice_height_ = node_->get_parameter("grid_map/esdf_slice_height").as_double();
  mp_.show_esdf_time_ = node_->get_parameter("grid_map/show_esdf_time").as_bool();
  mp_.local_bound_inflate_ = node_->get_parameter("grid_map/local_bound_inflate").as_double();
  
  // Road boundary parameters
  mp_.use_road_boundary_ = node_->get_parameter("grid_map/use_road_boundary").as_bool();
  mp_.road_width_ = node_->get_parameter("grid_map/road_width").as_double();
  mp_.road_margin_ = node_->get_parameter("grid_map/road_margin").as_double();

  // Threat zone parameters
  mp_.use_threat_zones_ = node_->get_parameter("grid_map/use_threat_zones").as_bool();
  mp_.threat_cost_weight_ = node_->get_parameter("grid_map/threat_cost_weight").as_double();

  // Terrain gridmap parameters
  mp_.use_terrain_obstacles_ = node_->get_parameter("grid_map/use_terrain_obstacles").as_bool();
  mp_.terrain_obstacle_threshold_ = node_->get_parameter("grid_map/terrain_obstacle_threshold").as_double();
  mp_.terrain_target_cell_size_ = node_->get_parameter("grid_map/terrain_target_cell_size").as_double();

  // Visualization parameters
  mp_.publish_global_esdf_viz_ = node_->get_parameter("grid_map/publish_global_esdf_viz").as_bool();

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

  // Load threat zones
  auto threat_zones_array = node_->get_parameter("threat_zones").as_double_array();
  if (threat_zones_array.size() % 6 == 0 && threat_zones_array.size() > 0) {
    for (size_t i = 0; i < threat_zones_array.size(); i += 6) {
      ThreatZone zone;
      zone.center = Eigen::Vector3d(threat_zones_array[i], threat_zones_array[i + 1], threat_zones_array[i + 2]);
      zone.detection_range = threat_zones_array[i + 3];
      zone.engagement_range = threat_zones_array[i + 4];
      zone.max_threat_level = threat_zones_array[i + 5];
      zone.name = "ThreatZone_" + std::to_string(i / 6);
      mp_.threat_zones_.push_back(zone);
    }
  } else if (threat_zones_array.size() > 0) {
    RCLCPP_WARN(node_->get_logger(), "Invalid threat zones data. Each zone should have 6 values (cx, cy, cz, detection, engagement, threat).");
  }

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
  std::cout << "  map_origin_x: " << node_->get_parameter("grid_map/map_origin_x").as_double() << std::endl;
  std::cout << "  map_origin_y: " << node_->get_parameter("grid_map/map_origin_y").as_double() << std::endl;
  std::cout << "  use_threat_zones: " << mp_.use_threat_zones_ << std::endl;
  std::cout << "  threat_cost_weight: " << mp_.threat_cost_weight_ << std::endl;
  std::cout << "  threat_zones: " << mp_.threat_zones_.size() << " zones" << std::endl;
  for (size_t i = 0; i < mp_.threat_zones_.size(); ++i) {
    const auto& zone = mp_.threat_zones_[i];
    std::cout << "    zone " << i << ": center=(" << zone.center.x() << "," << zone.center.y() << "," << zone.center.z()
              << "), detection=" << zone.detection_range << "m, engagement=" << zone.engagement_range
              << "m, threat=" << zone.max_threat_level << std::endl;
  }
  std::cout << "  use_terrain_obstacles: " << mp_.use_terrain_obstacles_ << std::endl;
  std::cout << "  terrain_obstacle_threshold: " << mp_.terrain_obstacle_threshold_ << "m" << std::endl;
  std::cout << "  terrain_target_cell_size: " << mp_.terrain_target_cell_size_ << "m" << std::endl;
  std::cout << "  publish_global_esdf_viz: " << mp_.publish_global_esdf_viz_ << std::endl;

  mp_.local_bound_inflate_ = std::max(mp_.resolution_, mp_.local_bound_inflate_);
  mp_.resolution_inv_ = 1.0 / mp_.resolution_;

  // Get custom map origin if provided, otherwise use default centered origin
  double map_origin_x = node_->get_parameter("grid_map/map_origin_x").as_double();
  double map_origin_y = node_->get_parameter("grid_map/map_origin_y").as_double();

  if (map_origin_x == 0.0 && map_origin_y == 0.0) {
    // Default behavior: center map at (0, 0)
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  } else {
    // Use custom origin for optimized map placement
    mp_.map_origin_ = Eigen::Vector3d(map_origin_x, map_origin_y, mp_.ground_height_);
  }

  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  
  // Initialize ESDF parameters
  auto logit = [](double x) { return log(x / (1.0 - x)); };
  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // Adjust virtual_ceil_height relative to ground_height (like ROS1 version)
  if (mp_.virtual_ceil_height_ >= z_size + mp_.ground_height_) {
    mp_.virtual_ceil_height_ = z_size + mp_.ground_height_ - mp_.resolution_;
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
  md_.threat_buffer_.resize(buffer_size, 0.0);

  distance_buffer_local_.clear();
  local_esdf_min_ = Eigen::Vector3i(0, 0, 0);
  local_esdf_max_ = Eigen::Vector3i(0, 0, 0);

  // Subscribe to terrain gridmap if enabled
  if (mp_.use_terrain_obstacles_) {
    terrain_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
      "/terrain/grid_map",
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
        this->processTerrainGridMap(msg);
      });
  }

  global_esdf_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/global_esdf_visualization",
    rclcpp::QoS(1).transient_local().reliable());
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
    // Create a vector to store inflation points
    std::vector<Eigen::Vector3i> inf_pts(pow(2 * step + 1, 3));
    
    // Use the overloaded function to get inflation points
    inflatePoint(pt, step, inf_pts);
    
    // Apply inflation to occupancy buffer
    for (const auto& inf_pt : inf_pts) {
        if (isInMap(inf_pt)) {
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

void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step, std::vector<Eigen::Vector3i>& pts) {
    int num = 0;

    // Use full 3D inflation (same as original code)
    // This is needed for depth sensor / point cloud inflation
    for (int x = -step; x <= step; ++x) {
        for (int y = -step; y <= step; ++y) {
            for (int z = -step; z <= step; ++z) {
                if (num < (int)pts.size()) {
                    pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
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

void GridMap::clearAndInflateLocalMap() {
  // Clear outdated data in local bounds
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x) {
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
    }
  }

  // Inflate obstacles using obstacles_inflation parameter
  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  std::vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x) {
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
          
          // Apply inflation points to occupancy buffer
          for (int k = 0; k < (int)inf_pts.size(); ++k) {
            Eigen::Vector3i inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 || 
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }
    }
  }

  // Add virtual ceiling
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x) {
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
    }
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
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
    md_.esdf_time_ += (t2 - t1).seconds();
    md_.max_esdf_time_ = std::max(md_.max_esdf_time_, (t2 - t1).seconds());
  }
}

void GridMap::evaluateEDT(const Eigen::Vector3d& pos, double& dist) {
  // Ground constraint: Z < 0.0 is treated as obstacle (distance = 0)
  if (pos.z() < 0.0) {
    dist = 0.0;
    return;
  }

  Eigen::Vector3i idx;
  posToIndex(pos, idx);

  // Local ESDF check (use raw index before clamping)
  if (idx(0) >= local_esdf_min_(0) && idx(0) <= local_esdf_max_(0) &&
      idx(1) >= local_esdf_min_(1) && idx(1) <= local_esdf_max_(1) &&
      idx(2) >= local_esdf_min_(2) && idx(2) <= local_esdf_max_(2)) {

    Eigen::Vector3i local_size = local_esdf_max_ - local_esdf_min_ + Eigen::Vector3i(1, 1, 1);
    int local_idx = (idx(0) - local_esdf_min_(0)) * local_size(1) * local_size(2) +
                    (idx(1) - local_esdf_min_(1)) * local_size(2) +
                    (idx(2) - local_esdf_min_(2));

    dist = distance_buffer_local_[local_idx];
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
  // Ground constraint: Z < 0.0 has upward gradient to push away from ground
  if (pos.z() < 0.0) {
    grad = Eigen::Vector3d(0.0, 0.0, 1.0);  // Push upward
    return;
  }

  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearFirstGrad(dists, diff, grad);
}

// ============================================================================
// Threat Zone Functions (Air Defense Penetration)
// ============================================================================

void GridMap::addThreatZone(const ThreatZone& zone) {
  mp_.threat_zones_.push_back(zone);
  updateThreatField();
}

void GridMap::clearThreatZones() {
  mp_.threat_zones_.clear();
  std::fill(md_.threat_buffer_.begin(), md_.threat_buffer_.end(), 0.0);
}

void GridMap::updateThreatField() {
  if (!mp_.use_threat_zones_ || mp_.threat_zones_.empty()) {
    return;
  }

  auto start_time = rclcpp::Clock().now();

  // Reset threat buffer
  std::fill(md_.threat_buffer_.begin(), md_.threat_buffer_.end(), 0.0);

  // For each voxel in the map
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x) {
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y) {
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        Eigen::Vector3i idx(x, y, z);
        Eigen::Vector3d pos;
        indexToPos(idx, pos);

        // Calculate cumulative threat from all zones
        double total_threat = 0.0;

        for (const auto& zone : mp_.threat_zones_) {
          Eigen::Vector3d diff = pos - zone.center;
          double dist = diff.norm();

          // Gaussian decay within engagement range (high threat)
          if (dist < zone.engagement_range) {
            double sigma = zone.engagement_range / 3.0;  // 99.7% within 3*sigma
            double normalized_dist = dist / sigma;
            total_threat += zone.max_threat_level * exp(-0.5 * normalized_dist * normalized_dist);
          }
          // Linear decay in detection range (medium threat)
          else if (dist < zone.detection_range) {
            double ratio = (dist - zone.engagement_range) /
                          (zone.detection_range - zone.engagement_range);
            total_threat += zone.max_threat_level * 0.3 * (1.0 - ratio);
          }
          // Outside detection range: no threat
        }

        // Store threat level in buffer
        int idx_addr = toAddress(idx);
        md_.threat_buffer_[idx_addr] = total_threat;
      }
    }
  }

  auto end_time = rclcpp::Clock().now();
  double elapsed_ms = (end_time - start_time).seconds() * 1000.0;

}

double GridMap::getThreatLevel(const Eigen::Vector3d& pos) const {
  if (!mp_.use_threat_zones_ || mp_.threat_zones_.empty()) {
    return 0.0;
  }

  // Real-time calculation (more accurate for positions between voxels)
  double total_threat = 0.0;

  for (const auto& zone : mp_.threat_zones_) {
    Eigen::Vector3d diff = pos - zone.center;
    double dist = diff.norm();

    // Gaussian decay within engagement range
    if (dist < zone.engagement_range) {
      double sigma = zone.engagement_range / 3.0;
      double normalized_dist = dist / sigma;
      total_threat += zone.max_threat_level * exp(-0.5 * normalized_dist * normalized_dist);
    }
    // Linear decay in detection range
    else if (dist < zone.detection_range) {
      double ratio = (dist - zone.engagement_range) /
                    (zone.detection_range - zone.engagement_range);
      total_threat += zone.max_threat_level * 0.3 * (1.0 - ratio);
    }
  }

  return total_threat;
}

Eigen::Vector3d GridMap::getThreatGradient(const Eigen::Vector3d& pos) const {
  if (!mp_.use_threat_zones_ || mp_.threat_zones_.empty()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d grad = Eigen::Vector3d::Zero();

  for (const auto& zone : mp_.threat_zones_) {
    Eigen::Vector3d diff = pos - zone.center;
    double dist = diff.norm();

    if (dist < 1e-6) continue;  // Avoid division by zero at center

    Eigen::Vector3d dir = diff / dist;  // Normalized direction (pointing away from threat)

    // Gradient of Gaussian: d/dx[exp(-0.5*(x/σ)²)] = -(x/σ²)*exp(-0.5*(x/σ)²)
    if (dist < zone.engagement_range) {
      double sigma = zone.engagement_range / 3.0;
      double sigma_sq = sigma * sigma;
      double normalized_dist = dist / sigma;
      double gaussian = exp(-0.5 * normalized_dist * normalized_dist);

      // Gradient magnitude (negative because threat decreases away from center)
      double grad_magnitude = -zone.max_threat_level * (dist / sigma_sq) * gaussian;
      grad += grad_magnitude * dir;
    }
    // Gradient of linear decay
    else if (dist < zone.detection_range) {
      double range_diff = zone.detection_range - zone.engagement_range;
      double grad_magnitude = -zone.max_threat_level * 0.3 / range_diff;
      grad += grad_magnitude * dir;
    }
  }

  return grad;
}

// ============================================================================
// Terrain GridMap Functions
// ============================================================================

void GridMap::processTerrainGridMap(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
  if (!mp_.use_terrain_obstacles_) {
    return;
  }

  double terrain_scale_factor = msg->info.resolution;

  // Find elevation and water layers
  int elevation_idx = -1;
  int water_idx = -1;
  for (size_t i = 0; i < msg->layers.size(); ++i) {
    if (msg->layers[i] == "elevation") {
      elevation_idx = i;
    } else if (msg->layers[i] == "water") {
      water_idx = i;
    }
  }

  if (elevation_idx < 0) {
    RCLCPP_ERROR(node_->get_logger(), "No 'elevation' layer found in terrain gridmap!");
    return;
  }

  const auto& elevation_layer = msg->data[elevation_idx];
  const auto* water_layer = (water_idx >= 0) ? &msg->data[water_idx] : nullptr;

  // Get terrain gridmap dimensions
  if (elevation_layer.layout.dim.size() < 2) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid elevation layer dimensions!");
    return;
  }

  int terrain_cols = elevation_layer.layout.dim[0].size;
  int terrain_rows = elevation_layer.layout.dim[1].size;

  double terrain_origin_x = msg->info.pose.position.x - msg->info.length_x / 2.0;
  double terrain_origin_y = msg->info.pose.position.y - msg->info.length_y / 2.0;

  double terrain_cell_size_x = msg->info.length_x / terrain_cols;
  double terrain_cell_size_y = msg->info.length_y / terrain_rows;

  // Process terrain gridmap and mark obstacles
  int obstacle_count = 0;
  static bool out_of_bounds_warned = false;

  for (int terrain_x = 0; terrain_x < terrain_cols; ++terrain_x) {
    for (int terrain_y = 0; terrain_y < terrain_rows; ++terrain_y) {

      // GridMap uses column-major order (Fortran-style)
      int terrain_idx = terrain_x * terrain_rows + terrain_y;

      if (terrain_idx >= (int)elevation_layer.data.size()) {
        continue;
      }

      // Get elevation value (scaled by (scale_factor / TARGET_CELL_SIZE_M) in terrain_publisher.py)
      float elevation_normalized = elevation_layer.data[terrain_idx];

      // Check if this is a NaN (water/invalid cell in elevation layer)
      if (std::isnan(elevation_normalized)) {
        continue;
      }

      // terrain_publisher: elevation_normalized = (elevation_meters * scale_factor) / TARGET_CELL_SIZE_M
      // elevation_normalized is a dimensionless ratio representing height in units of TARGET_CELL_SIZE_M
      //
      // Our grid uses mp_.resolution_ (0.2m) per cell in a NORMALIZED coordinate system
      // where 1.0 unit = TARGET_CELL_SIZE_M (mp_.terrain_target_cell_size_) in real world
      // So elevation_normalized directly represents height in our normalized grid units
      //
      // Example: real elevation = 947m, scale_factor = 0.4411, TARGET_CELL_SIZE_M = 50m
      // normalized = (947 * 0.4411) / 50 = 8.36
      // In our grid: 8.36 units (where 1 unit = 50m concept)
      // In grid cells: 8.36 / mp_.resolution_ = 8.36 / 0.2 = 41.8 cells
      // elevation_normalized is already in normalized grid units (same coordinate system as XY)
      double elevation = elevation_normalized;

      // Note: elevation layer already filters out water (NaN values)
      // If we reach here, it's valid land terrain

      // Check if elevation exceeds threshold (obstacle must be taller than threshold)
      // Convert threshold from real meters to normalized units for comparison
      double threshold_normalized = (mp_.terrain_obstacle_threshold_ * terrain_scale_factor) / mp_.terrain_target_cell_size_;
      if (elevation > threshold_normalized) {
        // Convert terrain grid position to world coordinates
        double world_x_temp = terrain_origin_x + (terrain_x + 0.5) * terrain_cell_size_x;
        double world_y_temp = terrain_origin_y + (terrain_y + 0.5) * terrain_cell_size_y;

        double terrain_center_x = terrain_origin_x + msg->info.length_x / 2.0;
        double terrain_center_y = terrain_origin_y + msg->info.length_y / 2.0;

        double flipped_x = 2.0 * terrain_center_x - world_x_temp;
        double flipped_y = world_y_temp;

        double rel_x = flipped_x - terrain_center_x;
        double rel_y = flipped_y - terrain_center_y;

        double world_x = terrain_center_x - rel_y;
        double world_y = terrain_center_y + rel_x;
        double world_z = 0.0;

        Eigen::Vector3d world_pos(world_x, world_y, world_z);

        // Check if world position is within our map bounds
        if (!isInMap(world_pos)) {
          if (!out_of_bounds_warned) {
            RCLCPP_WARN(node_->get_logger(),
                       "Some terrain obstacles are out of map bounds (terrain map is larger than planning map). This warning will only be shown once.");
            out_of_bounds_warned = true;
          }
          continue;
        }

        // Convert to our grid_map index for base position
        Eigen::Vector3i base_idx;
        posToIndex(world_pos, base_idx);

        // Calculate how many Z levels to fill based on normalized elevation
        // elevation is in normalized units, mp_.resolution_ is grid resolution in normalized units
        int z_cells_to_fill = static_cast<int>(std::ceil(elevation / mp_.resolution_));

        // Clamp to map height
        z_cells_to_fill = std::min(z_cells_to_fill, mp_.map_voxel_num_(2));

        // Set as obstacle - mark from ground up to elevation height
        // Apply 2D inflation (X, Y only) for terrain to avoid sticking to terrain
        int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);

        for (int dx = -inf_step; dx <= inf_step; ++dx) {
          for (int dy = -inf_step; dy <= inf_step; ++dy) {
            Eigen::Vector3i inflated_base(base_idx(0) + dx, base_idx(1) + dy, 0);

            // Fill Z only up to actual terrain height (no Z inflation)
            for (int z = 0; z < z_cells_to_fill; ++z) {
              Eigen::Vector3i idx_z(inflated_base(0), inflated_base(1), z);
              if (isInMap(idx_z)) {
                int idx = toAddress(idx_z);
                md_.occupancy_buffer_[idx] = 1.0;
                md_.occupancy_buffer_inflate_[idx] = 1;  // ESDF uses inflate buffer
              }
            }
          }
        }
        obstacle_count++;

      }
    }
  }

  if (obstacle_count == 0) {
    RCLCPP_WARN(node_->get_logger(), "No terrain obstacles found! Check threshold and terrain data.");
    return;
  }

  updateESDF3d();

  if (mp_.publish_global_esdf_viz_) {
    publishGlobalESDFVisualization();
  }
}

void GridMap::publishGlobalESDFVisualization() {
  if (!global_esdf_pub_) {
    global_esdf_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/global_esdf_visualization", 10);
  }

  visualization_msgs::msg::MarkerArray marker_array;

  double sample_resolution = 0.5;
  int marker_id = 0;

  double viz_x_min = 0.0;
  double viz_x_max = 100.0;
  double viz_y_min = 0.0;
  double viz_y_max = 100.0;
  double viz_z_min = 0.0;
  double viz_z_max = 20.0;

  for (double x = viz_x_min; x < viz_x_max; x += sample_resolution) {
    for (double y = viz_y_min; y < viz_y_max; y += sample_resolution) {
      for (double z = viz_z_min; z <= viz_z_max; z += sample_resolution) {
        Eigen::Vector3d pos(x, y, z);

        if (!isInMap(pos)) continue;

        double dist;
        evaluateEDT(pos, dist);

        if (dist < 0.01) continue;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "global_esdf";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = pos(0);
        marker.pose.position.y = pos(1);
        marker.pose.position.z = pos(2);
        marker.pose.orientation.w = 1.0;

        double scale = sample_resolution * 1.2;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        if (dist < 0.5) {
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 0.1;
        } else if (dist < 1.5) {
          float ratio = (dist - 0.5) / 1.0;
          marker.color.r = 1.0;
          marker.color.g = ratio;
          marker.color.b = 0.0;
          marker.color.a = 0.05;
        } else {
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          marker.color.a = 0.01;
        }

        marker_array.markers.push_back(marker);

        if (marker_id >= 600000) {
          goto publish_markers;
        }
      }
    }
  }

publish_markers:
  if (marker_id == 0) {
    RCLCPP_WARN(node_->get_logger(), "No ESDF markers generated! Check if obstacles exist in the map.");
  }

  global_esdf_pub_->publish(marker_array);
}
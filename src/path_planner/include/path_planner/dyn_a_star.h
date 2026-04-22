#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>
#include "path_planner/sdf/sdf_manager.h"
#include "../../common/log_manager.hpp"
#include <queue>
#include <vector>

constexpr double inf = 1e20;

namespace path_planner { namespace astar {

struct ThreatZoneLite {
    Eigen::Vector3d center;
    double detection_range;
    double max_threat_level;
};

struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode
{
    enum enum_state
    {
        OPENSET = 1,
        CLOSEDSET = 2,
        UNDEFINED = 3
    };

    int rounds{0};
    enum enum_state state
    {
        UNDEFINED
    };
    Eigen::Vector3i index;

    double gScore{inf}, fScore{inf};
    GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
    bool operator()(GridNodePtr node1, GridNodePtr node2)
    {
        return node1->fScore > node2->fScore;
    }
};

class AStar
{
private:
    // SDF query backend. We do not need a separate occupancy map: a voxel is
    // considered blocked when sdf_distance < obstacle_margin_.
    path_planner::sdf::SDFManager *sdf_ = nullptr;
    const std::vector<ThreatZoneLite> *threat_zones_ = nullptr;
    double obstacle_margin_ = 0.5;  // meters
    double threat_weight_ = 0.1;
    double map_resolution_ = 1.0;
    Eigen::Vector3d map_origin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d map_size_ = Eigen::Vector3d::Zero();

    swarm_formation::LogManager::Ptr log_manager_;

    double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
    double getManhHeu(GridNodePtr node1, GridNodePtr node2);
    double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
    inline double getHeu(GridNodePtr node1, GridNodePtr node2);

    bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

    inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
    inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

    // Collision / threat queries backed by SDF.
    inline bool checkOccupancy_esdf(const Eigen::Vector3d &pos) {
        if (!sdf_ || !sdf_->hasData()) return false;
        float d = sdf_->getDistance(pos);
        if (!std::isfinite(d)) return true;  // outside map = blocked
        return d < obstacle_margin_;
    }
    inline bool checkOccupancy(const Eigen::Vector3d &pos) {
        return checkOccupancy_esdf(pos);
    }

    inline double getThreatCost(const Eigen::Vector3d &pos) {
        if (!threat_zones_ || threat_zones_->empty()) return 0.0;
        double level = 0.0;
        for (const auto &tz : *threat_zones_) {
            double dist = (pos - tz.center).norm();
            if (dist >= tz.detection_range) continue;
            double sigma = tz.detection_range / 3.0;
            double g = std::exp(-(dist * dist) / (2.0 * sigma * sigma));
            level += tz.max_threat_level * g;
        }
        return level * threat_weight_;
    }

    std::vector<GridNodePtr> retrievePath(GridNodePtr current);

    double step_size_, inv_step_size_;
    Eigen::Vector3d center_;
    Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
    const double tie_breaker_ = 1.0 + 1.0 / 10000;
    const int max_iterations_ = 50000;

    std::vector<GridNodePtr> gridPath_;

    GridNodePtr ***GridNodeMap_ = nullptr;
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;
    int rounds_{0};

public:
    typedef std::shared_ptr<AStar> Ptr;

    AStar(){};
    ~AStar();

    void setLogManager(swarm_formation::LogManager::Ptr log_manager) { log_manager_ = log_manager; }

    void setSDF(path_planner::sdf::SDFManager *sdf,
                const Eigen::Vector3d &origin,
                const Eigen::Vector3d &size,
                double resolution) {
        sdf_ = sdf;
        map_origin_ = origin;
        map_size_ = size;
        map_resolution_ = resolution;
    }
    void setThreatZones(const std::vector<ThreatZoneLite> *zones) { threat_zones_ = zones; }
    void setObstacleMargin(double m) { obstacle_margin_ = m; }
    void setThreatWeight(double w) { threat_weight_ = w; }

    void initGridMap(const Eigen::Vector3i &pool_size);
    // Free the existing pool (if any) and allocate a new one. Use when the
    // map span changes between queries.
    void resizePool(const Eigen::Vector3i &pool_size);

    bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_esdf_check);

    std::vector<Eigen::Vector3d> getPath();

    std::vector<Eigen::Vector3d> astarSearchAndGetSimplePath(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, int drone_id);

    Eigen::Vector3d getOrigin() const { return map_origin_; }
    Eigen::Vector3d getMapSize() const { return map_size_; }
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
    return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
    // Use round() instead of "+0.5, cast<int>()" because the latter truncates
    // toward zero for negative values, which biases cells on the negative side
    // of center_ by one step and leaves path[0] up to ~1 step farther from
    // start_pt than the true sqrt(3)/2 quantization bound.
    Eigen::Vector3d rel = (pt - center_) * inv_step_size_;
    idx = Eigen::Vector3i(std::lround(rel(0)), std::lround(rel(1)), std::lround(rel(2))) + CENTER_IDX_;

    if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
    {
        return false;
    }

    return true;
};

}} // namespace path_planner::astar

#endif

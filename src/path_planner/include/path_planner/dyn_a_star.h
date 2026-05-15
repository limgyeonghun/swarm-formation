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

struct RiskZoneLite {
    Eigen::Vector3d center;
    double reach;   // meters; risk is exactly zero outside this ball
    double peak;    // dimensionless in (0, 1]
};

struct GridNode
{
    enum enum_state
    {
        OPENSET = 1,
        CLOSEDSET = 2,
        UNDEFINED = 3
    };

    int rounds{0};
    int state{UNDEFINED};
    double gScore{inf};
    // SMHA* keeps two f-scores per node: anchor (admissible) and
    // inadmissible (risk-inflated). g is shared.
    double fAnchor{inf};
    double fInadmis{inf};
    int cameFromFlat{-1};
};

class AStar;

class NodeComparatorAnchor
{
public:
    NodeComparatorAnchor() = default;
    explicit NodeComparatorAnchor(const std::vector<GridNode> *pool) : pool_(pool) {}
    bool operator()(int a, int b) const
    {
        return (*pool_)[a].fAnchor > (*pool_)[b].fAnchor;
    }
private:
    const std::vector<GridNode> *pool_ = nullptr;
};

class NodeComparatorInadmis
{
public:
    NodeComparatorInadmis() = default;
    explicit NodeComparatorInadmis(const std::vector<GridNode> *pool) : pool_(pool) {}
    bool operator()(int a, int b) const
    {
        return (*pool_)[a].fInadmis > (*pool_)[b].fInadmis;
    }
private:
    const std::vector<GridNode> *pool_ = nullptr;
};

class AStar
{
private:
    // SDF query backend. We do not need a separate occupancy map: a voxel is
    // considered blocked when sdf_distance < obstacle_margin_.
    path_planner::sdf::SDFManager *sdf_ = nullptr;
    const std::vector<RiskZoneLite> *risk_zones_ = nullptr;
    double obstacle_margin_ = 0.5;  // meters
    // When true, the A* graph expansion ignores obstacles (every voxel is
    // traversable); shortcut / downstream checks still use obstacle_margin_.
    bool search_ignores_obstacles_ = false;
    // Hard ground / ceiling for A* expansion. Cells at or below
    // ground_height_ (and at or above virtual_ceil_height_) are rejected
    // just like SDF-occupied voxels. Sentinel: ≤ -0.5 disables the plane.
    double ground_height_ = -1.0;
    double virtual_ceil_height_ = -1.0;
    double risk_alpha_ = 1.0;
    // SMHA* (Aine et al., IJRR 2016) shared-g, dual-heuristic A*.
    // - Anchor queue (admissible): h_anchor = euclidean (Diag) tie-broken.
    // - Inadmis queue (greedy):    h_inadmis = smha_w_ * euclidean.
    // Inadmis is preferred while INADMIS.top.f <= smha_w_ * ANCHOR.top.f.
    // smha_w_ == 1.0 disables SMHA* dispatch and falls back to anchor-only.
    // Auto-dispatch chooses smha_w_ per planning call (see astarSearch*).
    double smha_w_ = 1.0;
    // Auto-mode parameters (set via yaml). Pick smha_w_ at plan start
    // based on goal risk; retry with transit_w if detour run fails.
    double detour_smha_w_  = 1.0;   // when goal sits outside any zone
    double transit_smha_w_ = 3.0;   // when goal sits inside a zone
    double goal_in_zone_threshold_ = 0.05;  // risk(goal) > this -> transit
    // Debug toggle: when true, astarSearchAndGetSimplePath returns the raw
    // 1-voxel-step A* path without shortcut/visibility-thinning. Used to
    // verify front-end behavior independent of the shortcut filter.
    bool bypass_shortcut_ = false;
    double map_resolution_ = 1.0;
    Eigen::Vector3d map_origin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d map_size_ = Eigen::Vector3d::Zero();

    swarm_formation::LogManager::Ptr log_manager_;

    double getDiagHeu(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2);
    double getManhHeu(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2);
    double getEuclHeu(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2);
    // Admissible anchor heuristic (pure Euclidean diag with tie breaker).
    inline double getHeuAnchor(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2);
    // Inadmissible heuristic — same shape as anchor, scaled by (1 + alpha*risk(n))
    // evaluated at i1. Helps SMHA* escape depression regions.
    inline double getHeuInadmis(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2);

    bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

    inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
    inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

    // Collision / risk queries backed by SDF.
    inline bool checkOccupancy_esdf(const Eigen::Vector3d &pos) {
        if (!sdf_ || !sdf_->hasData()) return false;
        float d = sdf_->getDistance(pos);
        if (!std::isfinite(d)) return true;  // outside map = blocked
        return d < obstacle_margin_;
    }
    inline bool checkOccupancy(const Eigen::Vector3d &pos) {
        return checkOccupancy_esdf(pos);
    }

    // V3: quadratic moat + probabilistic-OR composition.
    // Per zone: moat_i(x) = peak_i * (1 - d/reach_i)^2 for d < reach_i.
    // Composed: risk(x) = 1 - prod_i (1 - moat_i(x)),  bounded in [0, 1].
    // Returns risk_alpha_ * risk(x). Compact support; AABB pre-filter
    // skips the sqrt for far zones.
    inline double getRiskCost(const Eigen::Vector3d &pos) const {
        if (!risk_zones_ || risk_zones_->empty()) return 0.0;
        double survival = 1.0;
        for (const auto &tz : *risk_zones_) {
            const double dx = pos.x() - tz.center.x();
            if (std::abs(dx) >= tz.reach) continue;
            const double dy = pos.y() - tz.center.y();
            if (std::abs(dy) >= tz.reach) continue;
            const double dz = pos.z() - tz.center.z();
            if (std::abs(dz) >= tz.reach) continue;
            const double d = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (d >= tz.reach) continue;
            const double u = 1.0 - d / tz.reach;
            const double moat = tz.peak * u * u;
            constexpr double kMoatCap = 1.0 - 1e-3;
            survival *= (1.0 - std::min(moat, kMoatCap));
        }
        return risk_alpha_ * (1.0 - survival);
    }

    std::vector<int> retrievePath(int current_flat);

    double step_size_, inv_step_size_;
    Eigen::Vector3d center_;
    Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
    const double tie_breaker_ = 1.0 + 1.0 / 10000;
    const int max_iterations_ = 50000;

    std::vector<int> gridPath_;

    std::vector<GridNode> pool_;
    int nx_{0}, ny_{0}, nz_{0};
    std::priority_queue<int, std::vector<int>, NodeComparatorAnchor> openSet_anchor_;
    std::priority_queue<int, std::vector<int>, NodeComparatorInadmis> openSet_inadmis_;

    // Flat 1D index helpers. Row-major: i fastest, k slowest.
    inline int flatIdx(int i, int j, int k) const {
        return i + nx_ * (j + ny_ * k);
    }
    inline int flatIdx(const Eigen::Vector3i &idx) const {
        return idx(0) + nx_ * (idx(1) + ny_ * idx(2));
    }
    inline Eigen::Vector3i flatToIdx(int flat) const {
        int k = flat / (nx_ * ny_);
        int r = flat - k * (nx_ * ny_);
        int j = r / nx_;
        int i = r - j * nx_;
        return Eigen::Vector3i(i, j, k);
    }
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
    void setRiskZones(const std::vector<RiskZoneLite> *zones) { risk_zones_ = zones; }
    void setObstacleMargin(double m) { obstacle_margin_ = m; }
    void setSearchIgnoresObstacles(bool b) { search_ignores_obstacles_ = b; }
    void setGroundHeight(double h)      { ground_height_ = h; }
    void setVirtualCeilHeight(double h) { virtual_ceil_height_ = h; }
    void setRiskAlpha(double a) { risk_alpha_ = a; }
    void setSmhaW(double w) { smha_w_ = w; }
    // Back-compat alias used by callers; treats heuristic weight as SMHA w.
    void setHeuristicWeight(double w) { smha_w_ = w; }
    void setDetourSmhaW(double w)  { detour_smha_w_  = w; }
    void setTransitSmhaW(double w) { transit_smha_w_ = w; }
    void setGoalInZoneThreshold(double t) { goal_in_zone_threshold_ = t; }
    void setBypassShortcut(bool b) { bypass_shortcut_ = b; }

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

inline double AStar::getHeuAnchor(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    return tie_breaker_ * getDiagHeu(i1, i2);
}

inline double AStar::getHeuInadmis(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    // Greedy depression-escape heuristic: pure inflated distance to goal.
    // Inadmissible by construction (overestimates by smha_w_), it drives
    // expansion straight toward the goal even when neighbors have inflated
    // g-scores from risk. The admissible anchor is the optimality backstop.
    return tie_breaker_ * smha_w_ * getDiagHeu(i1, i2);
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

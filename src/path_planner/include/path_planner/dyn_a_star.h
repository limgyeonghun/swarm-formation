#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>
#include "path_planner/sdf/distance_field.h"
#include "../../common/log_manager.hpp"
#include <queue>
#include <vector>

constexpr double inf = 1e20;

namespace path_planner { namespace search {

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

class PathSearcher;

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

class PathSearcher
{
public:
    // Front-end search selector (yaml manager/front_end).
    enum class FrontEnd { ASTAR, FM2 };

private:
    // SDF query backend. We do not need a separate occupancy map: a voxel is
    // considered blocked when sdf_distance < obstacle_margin_.
    const path_planner::sdf::IDistanceField *sdf_ = nullptr;
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
    // SMHA* (Aine et al., IJRR 2016) shared-g, dual-heuristic A*, run
    // unconditionally for every query — NO binary mode switch.
    //
    // - ANCHOR queue (admissible): h = euclidean (Diag) tie-broken.
    //   Bounds suboptimality (SMHA* 2-expand theorem) so detour quality
    //   is preserved.
    // - INADMIS queue: h = coarse risk-aware cost-to-go (see below).
    //   Dispatch: pop inadmis while INADMIS.top.f <= smha_w_ * ANCHOR.top.f.
    //
    // The inadmissible heuristic is a COARSE-GRID risk-aware value-to-go,
    // computed by Dijkstra from the goal on a K-times-downsampled grid
    // using the SAME edge cost  dist*(1 + alpha*risk).  Per Wilt & Ruml
    // (SoCS 2012, "When does Weighted A* Fail?"): greedy/weighted search
    // is fast iff the heuristic is correlated with true cost-to-go. A
    // reweighted Euclidean heuristic is *anti*-correlated inside a risk
    // depression (goal gets closer geometrically but more expensive),
    // which is exactly why every distance-based inadmissible heuristic we
    // tried stalled on goal-in-zone or bulldozed mid-path detours. A
    // coarse cost-to-go encodes the depression structure exactly, so the
    // SAME single heuristic:
    //   * routes around a mid-path zone (coarse value says detour is
    //     cheaper),
    //   * cuts straight through when the goal is inside a zone (coarse
    //     value says transit is the cheapest available),
    //   * never bulldozes a detour that is actually cheaper.
    // (Holte hierarchical A*, Felner additive PDB: an abstract search's
    // exact cost-to-go is a valid heuristic for the fine search.)
    double smha_w_ = 1.0;

    // Coarse value field for the inadmissible heuristic.
    int coarse_k_ = 8;                  // downsample factor (fine->coarse)
    int cnx_ = 0, cny_ = 0, cnz_ = 0;   // coarse grid dims
    std::vector<double> coarse_g_;      // cost-to-go from goal; inf if unreachable
    bool coarse_valid_ = false;
    Eigen::Vector3i coarse_goal_idx_{-1, -1, -1};

    inline int coarseFlat(int ci, int cj, int ck) const {
        return ci + cnx_ * (cj + cny_ * ck);
    }
    // World position -> coarse cost-to-go (inf-safe). Returns -1 if the
    // coarse field is not valid so the caller can fall back to euclidean.
    double coarseCostToGo(const Eigen::Vector3d &world) const;
    // (Re)compute the coarse Dijkstra value field rooted at `goal`.
    // Cheap (coarse grid has ~ fine/K^3 cells); called once per query.
    void buildCoarseValueField(const Eigen::Vector3d &goal_world);

    // ----- FM2 (Fast Marching Square) front-end -----
    // Heuristic-free Eikonal planner. Solves |∇T|·F = 1 from the goal on
    // a coarse risk-weighted speed map, then extracts the geodesic by
    // gradient descent. No local minima (Valero-Gomez et al.) so it has
    // none of the Wilt&Ruml depression-explosion of the A* family.
    // (FrontEnd enum is public — see below.)
    FrontEnd front_end_ = FrontEnd::ASTAR;
    int   fm2_coarse_k_ = 4;            // Eikonal grid downsample factor
    bool  fm2_star_ = true;             // FM2*: cost-to-go heuristic on
                                        // the FMM queue (same trajectory)
    int   fcnx_ = 0, fcny_ = 0, fcnz_ = 0;
    std::vector<float> fm2_T_;          // arrival time / cost-to-go
    std::vector<float> fm2_F_;          // speed map in (0, 1]
    bool  fm2_valid_ = false;

    inline int fm2Flat(int i, int j, int k) const {
        return i + fcnx_ * (j + fcny_ * k);
    }
    // Build speed map (ESDF + OR-moat risk) on the coarse grid.
    void fm2BuildSpeedMap();
    // Solve the Eikonal equation rooted at goal_world; fills fm2_T_.
    void fm2SolveEikonal(const Eigen::Vector3d &goal_world,
                         const Eigen::Vector3d &start_world);
    // Extract the geodesic start->goal by descending -∇T. World coords.
    std::vector<Eigen::Vector3d> fm2ExtractGeodesic(
        const Eigen::Vector3d &start_world,
        const Eigen::Vector3d &goal_world);
    // Trilinear sample of fm2_T_ at a world point; +inf if outside/blocked.
    double fm2SampleT(const Eigen::Vector3d &world) const;
    // Debug toggle: when true, astarSearchAndGetSimplePath returns the raw
    // 1-voxel-step A* path without shortcut/visibility-thinning. Used to
    // verify front-end behavior independent of the shortcut filter.
    bool bypass_shortcut_ = false;
    double map_resolution_ = 1.0;
    Eigen::Vector3d map_origin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d map_size_ = Eigen::Vector3d::Zero();

    swarm_formation::LogManager::Ptr log_manager_;

    double getDiagHeu(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2);
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
    // Normalized OR-moat risk in [0, 1] (no alpha scaling). Used by the
    // inadmissible heuristic so its inflation factor stays dimensionless.
    inline double getRiskNorm(const Eigen::Vector3d &pos) const {
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
        return 1.0 - survival;
    }

    inline double getRiskCost(const Eigen::Vector3d &pos) const {
        return risk_alpha_ * getRiskNorm(pos);
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
    typedef std::shared_ptr<PathSearcher> Ptr;

    PathSearcher(){};
    ~PathSearcher();

    void setLogManager(swarm_formation::LogManager::Ptr log_manager) { log_manager_ = log_manager; }

    void setSDF(const path_planner::sdf::IDistanceField *sdf,
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
    void setFrontEnd(FrontEnd fe) { front_end_ = fe; }
    void setFm2CoarseK(int k) { fm2_coarse_k_ = (k >= 1 ? k : 1); }
    void setFm2Star(bool on) { fm2_star_ = on; }
    void setBypassShortcut(bool b) { bypass_shortcut_ = b; }

    // ----- Visualization dump accessors (read-only, post-search) -----
    const std::vector<float>&  getFm2T() const { return fm2_T_; }
    const std::vector<float>&  getFm2F() const { return fm2_F_; }
    const std::vector<double>& getCoarseG() const { return coarse_g_; }
    Eigen::Vector3i getFm2Dims() const { return {fcnx_, fcny_, fcnz_}; }
    Eigen::Vector3i getCoarseDims() const { return {cnx_, cny_, cnz_}; }
    int getFm2CoarseK() const { return fm2_coarse_k_; }
    int getCoarseK() const { return coarse_k_; }

    // Fine pool gScore (A* per-cell accumulated cost) z-slice dump.
    // Returns a flat (nx * ny) vector of gScore for the given z layer,
    // with INFs left as +inf. Used for visualization only.
    Eigen::Vector3i getPoolSize() const { return POOL_SIZE_; }
    std::vector<double> getFineGScoreSlice(int z) const {
        std::vector<double> out;
        if (z < 0 || z >= POOL_SIZE_(2)) return out;
        out.reserve(static_cast<size_t>(POOL_SIZE_(0)) * POOL_SIZE_(1));
        for (int x = 0; x < POOL_SIZE_(0); ++x)
            for (int y = 0; y < POOL_SIZE_(1); ++y) {
                int flat = (x * POOL_SIZE_(1) + y) * POOL_SIZE_(2) + z;
                out.push_back(pool_[flat].gScore);
            }
        return out;
    }

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

inline double PathSearcher::getHeuAnchor(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    return tie_breaker_ * getDiagHeu(i1, i2);
}

inline double PathSearcher::getHeuInadmis(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    // Coarse risk-aware cost-to-go (see buildCoarseValueField). This
    // encodes the depression structure exactly, so the same heuristic
    // detours around mid-path zones AND cuts through goal-in-zone.
    if (coarse_valid_) {
        const Eigen::Vector3d w = Index2Coord(i1);
        const double c = coarseCostToGo(w);
        if (c >= 0.0) return c;            // valid coarse value
    }
    // Fallback (coarse field unavailable / cell unreachable): inflated
    // euclidean so the inadmis queue still makes progress.
    return tie_breaker_ * smha_w_ * getDiagHeu(i1, i2);
}

inline Eigen::Vector3d PathSearcher::Index2Coord(const Eigen::Vector3i &index) const
{
    return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool PathSearcher::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
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

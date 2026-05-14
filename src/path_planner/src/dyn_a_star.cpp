#include "path_planner/dyn_a_star.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>

using namespace std;
using namespace Eigen;

namespace path_planner { namespace astar {

// Read VmRSS / VmPeak from /proc/self/status. Returns KB. -1 on failure.
static long readVmRssKB() {
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
        if (line.compare(0, 6, "VmRSS:") == 0) {
            long kb = -1;
            sscanf(line.c_str(), "VmRSS: %ld", &kb);
            return kb;
        }
    }
    return -1;
}
static long readVmPeakKB() {
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
        if (line.compare(0, 7, "VmPeak:") == 0) {
            long kb = -1;
            sscanf(line.c_str(), "VmPeak: %ld", &kb);
            return kb;
        }
    }
    return -1;
}

AStar::~AStar() = default;

void AStar::initGridMap(const Eigen::Vector3i &pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;
    nx_ = pool_size(0);
    ny_ = pool_size(1);
    nz_ = pool_size(2);
    const size_t N = static_cast<size_t>(nx_) * ny_ * nz_;

    const long rss_before = readVmRssKB();
    pool_.assign(N, GridNode{});
    const long rss_after = readVmRssKB();
    const long peak = readVmPeakKB();
    if (log_manager_) {
        log_manager_->infof("[A* MEM] pool alloc: N=%zu sizeof=%zu theoretical=%.2f MB | "
                            "VmRSS %ld -> %ld KB (delta=%ld KB = %.2f MB) | VmPeak=%ld KB (%.2f MB)",
                            N, sizeof(GridNode),
                            (double)(N * sizeof(GridNode)) / 1024.0 / 1024.0,
                            rss_before, rss_after, rss_after - rss_before,
                            (double)(rss_after - rss_before) / 1024.0,
                            peak, (double)peak / 1024.0);
    }
    // openSet_ comparator binds to pool_ for fScore lookup.
    openSet_ = std::priority_queue<int, std::vector<int>, NodeComparator>(
        NodeComparator(&pool_));

    // Self-check: flatIdx/flatToIdx round-trip on a sparse sample plus
    // boundary cells. Cheap (sub-ms) and only runs on init.
    {
        auto check = [&](int i, int j, int k) {
            const int f = flatIdx(i, j, k);
            const Eigen::Vector3i back = flatToIdx(f);
            if (back(0) != i || back(1) != j || back(2) != k) {
                if (log_manager_) {
                    log_manager_->errorf("[A* INIT] flatIdx self-check FAILED at (%d,%d,%d) -> %d -> (%d,%d,%d)",
                        i, j, k, f, back(0), back(1), back(2));
                }
                std::abort();
            }
        };
        const int sx = std::max(1, nx_ / 8);
        const int sy = std::max(1, ny_ / 8);
        const int sz = std::max(1, nz_ / 8);
        for (int i = 0; i < nx_; i += sx)
            for (int j = 0; j < ny_; j += sy)
                for (int k = 0; k < nz_; k += sz)
                    check(i, j, k);
        check(0, 0, 0);
        check(nx_ - 1, ny_ - 1, nz_ - 1);
        if (log_manager_) {
            log_manager_->infof("[A* INIT] flatIdx self-check passed (pool=%dx%dx%d, total=%zu)",
                nx_, ny_, nz_, pool_.size());
        }
    }
}

void AStar::resizePool(const Eigen::Vector3i &pool_size)
{
    initGridMap(pool_size);
}

double AStar::getDiagHeu(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    double dx = abs(i1(0) - i2(0));
    double dy = abs(i1(1) - i2(1));
    double dz = abs(i1(2) - i2(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    double dx = abs(i1(0) - i2(0));
    double dy = abs(i1(1) - i2(1));
    double dz = abs(i1(2) - i2(2));
    return dx + dy + dz;
}

double AStar::getEuclHeu(const Eigen::Vector3i &i1, const Eigen::Vector3i &i2)
{
    return (i2 - i1).cast<double>().norm();
}


vector<int> AStar::retrievePath(int current_flat)
{
    vector<int> path;
    if (current_flat < 0) return path;

    int cur = current_flat;
    while (cur >= 0)
    {
        path.push_back(cur);
        cur = pool_[cur].cameFromFlat;
    }
    if (log_manager_ && risk_zones_ && !risk_zones_->empty()) {
        log_manager_->infof("[A* RETRIEVE] path length=%zu (listed goal->start)", path.size());
        int stride = std::max(1, (int)path.size() / 20);
        for (size_t i = 0; i < path.size(); i += stride) {
            const int f = path[i];
            const Eigen::Vector3i idx = flatToIdx(f);
            Eigen::Vector3d w = Index2Coord(idx);
            double tc = getRiskCost(w);
            log_manager_->infof("[A* RETRIEVE] i=%zu idx=(%d,%d,%d) world=(%.2f,%.2f,%.2f) g=%.3f risk_here=%.3f",
                i, idx(0), idx(1), idx(2),
                w.x(), w.y(), w.z(), pool_[f].gScore, tc);
        }
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (log_manager_) {
        log_manager_->debugf("시작/끝점 변환 시도 - Start: (%.2f,%.2f,%.2f), End: (%.2f,%.2f,%.2f)", 
                           start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));
    }
    
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx)) {
        if (log_manager_) {
            log_manager_->error("좌표를 인덱스로 변환 실패");
        }
        return false;
    }

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        if (log_manager_) {
            log_manager_->warnf("시작점이 장애물 내부에 위치 - Idx: (%d,%d,%d), Coord: (%.2f,%.2f,%.2f)", 
                               start_idx(0), start_idx(1), start_idx(2), start_pt(0), start_pt(1), start_pt(2));
        }
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
        if (log_manager_) {
            log_manager_->warnf("시작점 조정 완료 - 새로운 시작점: (%.2f,%.2f,%.2f)", start_pt(0), start_pt(1), start_pt(2));
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "New start point: (%f,%f,%f)", start_pt(0), start_pt(1), start_pt(2));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        if (log_manager_) {
            log_manager_->warnf("도착점이 장애물 내부에 위치 - Coord: (%.2f,%.2f,%.2f)", end_pt(0), end_pt(1), end_pt(2));
        }
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
        if (log_manager_) {
            log_manager_->warnf("도착점 조정 완료 - 새로운 도착점: (%.2f,%.2f,%.2f)", end_pt(0), end_pt(1), end_pt(2));
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "New END point: (%f,%f,%f)", end_pt(0), end_pt(1), end_pt(2));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt, bool use_esdf_check)
{
    auto time_1 = rclcpp::Clock().now();
    ++rounds_;
    
    if (log_manager_) {
        log_manager_->infof("3D A* 검색 시작 - Round: %d, Step size: %.3f, ESDF 사용: %s",
                           rounds_, step_size, use_esdf_check ? "Yes" : "No");
        log_manager_->infof("시작점: (%.2f,%.2f,%.2f), 도착점: (%.2f,%.2f,%.2f)",
                           start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));
        // DEBUG: risk binding at entry.
        size_t tz_n = risk_zones_ ? risk_zones_->size() : 0;
        log_manager_->infof("[A* DBG] risk_zones_ptr=%p size=%zu weight=%.3f",
                           (const void*)risk_zones_, tz_n, risk_weight_);
        if (risk_zones_) {
            for (size_t i = 0; i < risk_zones_->size(); ++i) {
                const auto &tz = (*risk_zones_)[i];
                log_manager_->infof("[A* DBG]  tz[%zu] c=(%.2f,%.2f,%.2f) R=%.2f L=%.2f",
                    i, tz.center.x(), tz.center.y(), tz.center.z(),
                    tz.sensing_range, tz.max_risk_level);
                // Direct probe: what does getRiskCost(center) return? If
                // the zone is really there it should be max_risk_level *
                // risk_weight_.
                double probe = getRiskCost(tz.center);
                log_manager_->infof("[A* DBG]  tz[%zu] probe@center cost=%.3f (expected=%.3f)",
                    i, probe, tz.max_risk_level * risk_weight_);
                // Off-by-one checks: 1m step towards goal from center.
                Eigen::Vector3d off_pos = tz.center + Eigen::Vector3d(1.0, 0.0, 0.0);
                log_manager_->infof("[A* DBG]  tz[%zu] probe@center+1mX cost=%.3f",
                    i, getRiskCost(off_pos));
            }
        }
    }
    // Reset risk-query counters for this search.
    dbg_risk_queries_ = 0;
    dbg_risk_nonzero_ = 0;
    dbg_risk_max_ = 0.0;
    dbg_risk_max_pos_ = Eigen::Vector3d::Zero();

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    if (log_manager_) {
        log_manager_->debugf("검색 중심점: (%.2f,%.2f,%.2f)", center_(0), center_(1), center_(2));
    }
    RCLCPP_INFO(rclcpp::get_logger("astar"), "CENTER: (%f,%f,%f)", center_(0), center_(1), center_(2));

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        if (log_manager_) {
            log_manager_->errorf("3D A* 검색 실패 - 시작/끝점 처리 불가");
            log_manager_->errorf("시작점: (%.2f,%.2f,%.2f), 도착점: (%.2f,%.2f,%.2f)", 
                               start_pt.x(), start_pt.y(), start_pt.z(), end_pt.x(), end_pt.y(), end_pt.z());
            log_manager_->errorf("Pool 크기: (%d,%d,%d), 중심: (%d,%d,%d)", 
                               POOL_SIZE_(0), POOL_SIZE_(1), POOL_SIZE_(2), CENTER_IDX_(0), CENTER_IDX_(1), CENTER_IDX_(2));
        }
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Unable to handle the initial or end point, force return!");
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Start: (%.2f,%.2f,%.2f) End: (%.2f,%.2f,%.2f)", 
                    start_pt.x(), start_pt.y(), start_pt.z(), end_pt.x(), end_pt.y(), end_pt.z());
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Pool size: (%d,%d,%d) Center: (%d,%d,%d)", 
                    POOL_SIZE_(0), POOL_SIZE_(1), POOL_SIZE_(2), CENTER_IDX_(0), CENTER_IDX_(1), CENTER_IDX_(2));
        return false;
    }

    if ( start_pt(0) > -1 && start_pt(0) < 0 )
        cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    int start_flat = flatIdx(start_idx);
    // end_idx is used directly for goal comparison (no need to flatten).

    std::priority_queue<int, std::vector<int>, NodeComparator> empty{NodeComparator(&pool_)};
    openSet_.swap(empty);

    GridNode &startNode = pool_[start_flat];
    startNode.rounds = rounds_;
    startNode.gScore = 0;
    startNode.fScore = getHeu(start_idx, end_idx);
    startNode.state = GridNode::OPENSET;
    startNode.cameFromFlat = -1;
    openSet_.push(start_flat);

    double tentative_gScore;

    int num_iter = 0;
    int current_flat = -1;
    while (!openSet_.empty())
    {
        num_iter++;
        current_flat = openSet_.top();
        openSet_.pop();
        GridNode &current = pool_[current_flat];

        const Eigen::Vector3i current_idx = flatToIdx(current_flat);
        if (current_idx(0) == end_idx(0) && current_idx(1) == end_idx(1) && current_idx(2) == end_idx(2))
        {
            auto time_2 = rclcpp::Clock().now();
            auto elapsed = time_2 - time_1;
            if (log_manager_) {
                log_manager_->infof("3D A* 검색 성공! 반복: %d회, 시간: %.3fms", num_iter, elapsed.seconds()*1000);
                log_manager_->infof("[A* DBG] risk_queries=%zu nonzero=%zu max_cost=%.3f at (%.2f,%.2f,%.2f)",
                    dbg_risk_queries_, dbg_risk_nonzero_, dbg_risk_max_,
                    dbg_risk_max_pos_.x(), dbg_risk_max_pos_.y(), dbg_risk_max_pos_.z());
                log_manager_->infof("[A* DBG] goal fScore=%.3f gScore=%.3f",
                    current.fScore, current.gScore);
            }
            printf("\033[34mA star iter:%d, time:%.3f\033[0m\n", num_iter, elapsed.seconds()*1000);
            gridPath_ = retrievePath(current_flat);
            return true;
        }
        current.state = GridNode::CLOSEDSET;

        static const int neighbor_offsets[26][3] = {
            {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
            {1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
            {1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
            {0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
            {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
            {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
        };
        static const double neighbor_costs_ordered[26] = {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            1.414213562373095, 1.414213562373095, 1.414213562373095, 1.414213562373095,
            1.414213562373095, 1.414213562373095, 1.414213562373095, 1.414213562373095,
            1.414213562373095, 1.414213562373095, 1.414213562373095, 1.414213562373095,
            1.732050807568877, 1.732050807568877, 1.732050807568877, 1.732050807568877,
            1.732050807568877, 1.732050807568877, 1.732050807568877, 1.732050807568877
        };

        for (int i = 0; i < 26; i++)
        {
            int dx = neighbor_offsets[i][0];
            int dy = neighbor_offsets[i][1];
            int dz = neighbor_offsets[i][2];

            Vector3i neighborIdx;
            neighborIdx(0) = current_idx(0) + dx;
            neighborIdx(1) = current_idx(1) + dy;
            neighborIdx(2) = current_idx(2) + dz;

            if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 ||
                neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 ||
                neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
            {
                continue;
            }

            const int neighbor_flat = flatIdx(neighborIdx);
            GridNode &neighbor = pool_[neighbor_flat];

            bool flag_explored = neighbor.rounds == rounds_;

            if (flag_explored && neighbor.state == GridNode::CLOSEDSET)
            {
                continue;
            }

            neighbor.rounds = rounds_;

            {
                const Eigen::Vector3d nw = Index2Coord(neighborIdx);
                // Hard ground / ceiling gate. Always applied, even when
                // search_ignores_obstacles_ is set.
                if (ground_height_ > -0.5 && nw.z() < ground_height_) continue;
                if (virtual_ceil_height_ > -0.5 && nw.z() > virtual_ceil_height_) continue;
            }

            if (!search_ignores_obstacles_) {
                if(use_esdf_check){
                    if (checkOccupancy_esdf(Index2Coord(neighborIdx)))
                        continue;
                } else {
                    if (checkOccupancy(Index2Coord(neighborIdx)))
                        continue;
                }
            }

            double static_cost = neighbor_costs_ordered[i];

            // Risk-aware edge cost: distance * (1 + risk). Multiplicative
            // form keeps shorter paths cheaper inside risk regions.
            Eigen::Vector3d neigh_world = Index2Coord(neighborIdx);
            double risk_cost = getRiskCost(neigh_world);
            tentative_gScore = current.gScore + static_cost * (1.0 + risk_cost);

            if (log_manager_ && risk_zones_ && !risk_zones_->empty()) {
                static thread_local int dbg_expand_count = 0;
                static thread_local int dbg_last_round = -1;
                if (dbg_last_round != rounds_) {
                    dbg_expand_count = 0;
                    dbg_last_round = rounds_;
                }
                bool inside_zone = false;
                for (const auto &tz : *risk_zones_) {
                    if ((neigh_world - tz.center).norm() < tz.sensing_range) {
                        inside_zone = true;
                        break;
                    }
                }
                bool log_it = (dbg_expand_count < 10) ||
                              (inside_zone && dbg_expand_count < 200);
                if (log_it) {
                    log_manager_->infof(
                        "[A* EXPAND] #%d idx=(%d,%d,%d) world=(%.3f,%.3f,%.3f) "
                        "static=%.3f risk=%.3f g=%.3f inside_zone=%d",
                        dbg_expand_count, neighborIdx(0),
                        neighborIdx(1), neighborIdx(2),
                        neigh_world.x(), neigh_world.y(), neigh_world.z(),
                        static_cost, risk_cost, tentative_gScore,
                        inside_zone ? 1 : 0);
                    ++dbg_expand_count;
                }
            }

            if (!flag_explored)
            {
                neighbor.state = GridNode::OPENSET;
                neighbor.cameFromFlat = current_flat;
                neighbor.gScore = tentative_gScore;
                neighbor.fScore = tentative_gScore + getHeu(neighborIdx, end_idx);
                openSet_.push(neighbor_flat);
            }
            else if (tentative_gScore < neighbor.gScore)
            {
                neighbor.cameFromFlat = current_flat;
                neighbor.gScore = tentative_gScore;
                neighbor.fScore = tentative_gScore + getHeu(neighborIdx, end_idx);
                openSet_.push(neighbor_flat);
            }
        }

        auto time_2 = rclcpp::Clock().now();
        auto elapsed = time_2 - time_1;
        // 20 s cap: global (one-shot) planning can afford a long front-end
        // search; upstream 0.5 s was tuned for on-board real-time replan.
        if (elapsed.seconds() > 20.0)
        {
            if (log_manager_) {
                log_manager_->warnf("3D A* 검색 시간 초과 - %.3fms 경과, 반복: %d회", elapsed.seconds()*1000, num_iter);
                log_manager_->errorf("시작점: (%.2f,%.2f,%.2f), 도착점: (%.2f,%.2f,%.2f)",
                                    start_pt.x(), start_pt.y(), start_pt.z(),
                                    end_pt.x(), end_pt.y(), end_pt.z());
            }
            RCLCPP_ERROR(rclcpp::get_logger("astar"),
                        "A* search timeout! Start: (%.2f,%.2f,%.2f), End: (%.2f,%.2f,%.2f), Iter: %d, Time: %.3fs",
                        start_pt.x(), start_pt.y(), start_pt.z(),
                        end_pt.x(), end_pt.y(), end_pt.z(),
                        num_iter, elapsed.seconds());
            return false;
        }
    }

    auto time_2 = rclcpp::Clock().now();
    auto elapsed_total = time_2 - time_1;

    if (log_manager_) {
        log_manager_->warnf("3D A* 검색 실패 - 전체 시간: %.3fms, 반복: %d회", elapsed_total.seconds()*1000, num_iter);
    }
    
    if (elapsed_total.seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("astar"), "Time consume in A star path finding is %.3fs, iter=%d", elapsed_total.seconds(), num_iter);

    return false;
}


vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;
    path.reserve(gridPath_.size());

    for (int flat : gridPath_)
        path.push_back(Index2Coord(flatToIdx(flat)));

    reverse(path.begin(), path.end());
    return path;
}

vector<Vector3d> AStar::astarSearchAndGetSimplePath(const double step_size, Vector3d start_pt, Vector3d end_pt, int drone_id){

    if (log_manager_) {
        log_manager_->infof("드론 %d: 3D 경로 검색 및 단순화 시작", drone_id);
        log_manager_->debugf("시작점: (%.2f,%.2f,%.2f), 도착점: (%.2f,%.2f,%.2f)",
                           start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));
    }

    // 3D A* search with ESDF
    bool search_success = false;
    vector<Vector3d> path;

    if (AstarSearch(step_size, start_pt, end_pt, true)) {
        path = getPath();
        if (path.size() > 1 && (path[0]-start_pt).norm() < step_size * 2.0) {
            if (log_manager_) {
                log_manager_->infof("드론 %d: 3D A* 검색 성공 (ESDF 사용) - 경로 점 개수: %zu", drone_id, path.size());
            }
            RCLCPP_INFO(rclcpp::get_logger("astar"), "3D A* search successful with ESDF");
            search_success = true;
        }
    }

    if (!search_success) {
        if (log_manager_) {
            log_manager_->warnf("드론 %d: 3D A* 검색 실패 (ESDF 사용), ESDF 없이 재시도", drone_id);
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "3D A* search failed with ESDF, retrying without ESDF");

        // 3D A* search without ESDF
        if (AstarSearch(step_size, start_pt, end_pt, false)) {
            path = getPath();
            if (path.size() > 1 && (path[0]-start_pt).norm() < step_size * 2.0) {
                if (log_manager_) {
                    log_manager_->infof("드론 %d: 3D A* 검색 성공 (ESDF 비사용) - 경로 점 개수: %zu", drone_id, path.size());
                }
                RCLCPP_INFO(rclcpp::get_logger("astar"), "3D A* search successful without ESDF");
                search_success = true;
            }
        }
    }

    // Fallback: Z축 상승 후 재시도
    if (!search_success) {
        Vector3d elevated_end = end_pt;
        elevated_end(2) += 5.0;  // 5m 상승

        if (log_manager_) {
            log_manager_->warnf("드론 %d: 3D A* 재시도 실패, 목표점 Z축 상승 시도 (%.2f → %.2fm)",
                               drone_id, end_pt(2), elevated_end(2));
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"),
                    "3D A* failed again, trying with elevated end point Z: %.2f -> %.2f",
                    end_pt(2), elevated_end(2));

        if (AstarSearch(step_size, start_pt, elevated_end, true)) {
            path = getPath();
            if (path.size() > 1 && (path[0]-start_pt).norm() < step_size * 2.0) {
                if (log_manager_) {
                    log_manager_->infof("드론 %d: Z축 상승 후 A* 검색 성공 - 경로 점 개수: %zu", drone_id, path.size());
                }
                RCLCPP_INFO(rclcpp::get_logger("astar"), "3D A* search successful with elevated end point");
                search_success = true;
            }
        }
    }

    if (!search_success) {
        if (log_manager_) {
            log_manager_->errorf("드론 %d: 3D A* 검색 완전 실패 - 직선 경로 반환", drone_id);
        }
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "3D A* search completely failed, returning direct path");

        vector<Vector3d> fallback_path;
        fallback_path.push_back(start_pt);
        fallback_path.push_back(end_pt);
        return fallback_path;
    }

    // Snap the end-points to the exact caller-provided coordinates so the
    // downstream MINCO boundary conditions match. A* cells can be up to
    // sqrt(3)/2 * step_size off in arbitrary directions.
    if (!path.empty()) {
        path.front() = start_pt;
        path.back() = end_pt;
    }

    // ===========================================
    // 3D Path Simplification (based on temp code)
    // ===========================================

    if (log_manager_) {
        double min_z = path[0](2), max_z = path[0](2);
        for (const auto& p : path) {
            min_z = std::min(min_z, p(2));
            max_z = std::max(max_z, p(2));
        }
        log_manager_->infof("드론 %d: Raw 경로 Z 범위: %.3f ~ %.3f (변화량: %.3f)",
                           drone_id, min_z, max_z, max_z - min_z);
    }

    int size = path.size();
    if (size <= 2) {
        if (log_manager_) {
            log_manager_->warnf("드론 %d: 경로가 2점만 가지고 있음 - 단순화 불필요", drone_id);
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "Path only has two points, no simplification needed");
        return path;
    }

    // Risk-aware shortcut (ported from the pre-A* RRT* pipeline).
    // Precompute cumulative edge cost (straight-line distance + Gaussian
    // risk integral) along the raw path. When trying to collapse points
    // i..j into a single straight segment, accept only if the segment is
    // (a) occupancy-free and (b) costs no more than 5 % over the detour
    // cost the A* search actually paid. This way:
    //   - clear corridors: fully shortcut to a straight line
    //   - risk detour    : original avoidance is preserved
    //   - forced breakthrough: detour_cost ≈ shortcut_cost, shortcut allowed
    const double kShortcutMargin = 1.05;
    const int    kRiskSamples  = 4;  // trapezoidal samples per segment

    auto segmentRiskCost = [&](const Vector3d &a, const Vector3d &b) {
        double d = (b - a).norm();
        if (d < 1e-6) return 0.0;
        double sum = 0.0;
        for (int si = 0; si <= kRiskSamples; ++si) {
            double t = (double)si / (double)kRiskSamples;
            Vector3d p = a + t * (b - a);
            double w = (si == 0 || si == kRiskSamples) ? 0.5 : 1.0;
            // (1 + risk) multiplier form matches the RRT* reference.
            sum += w * (1.0 + getRiskCost(p));
        }
        return d * sum / (double)kRiskSamples;
    };

    // Shortcut visibility uses the same obstacle margin as A* search so the
    // two stages stay consistent under debugging (e.g. setting margin very
    // low to probe whether shortcut is producing obstacle-clipping straights).
    auto segmentOccFree = [&](const Vector3d &a, const Vector3d &b) {
        int n = std::max(1, (int)std::ceil((b - a).norm() / 0.5));
        for (int k = 0; k <= n; ++k) {
            double t = (double)k / (double)n;
            Vector3d p = a + t * (b - a);
            if (checkOccupancy_esdf(p)) return false;
        }
        return true;
    };

    // Reject shortcuts that pass through a risk zone with non-trivial
    // risk level. A* already chose to detour around such zones; the 1.05
    // cost-ratio filter below is too permissive (a long straight through a
    // small zone may still fit under 5 %), so we gate it with a hard
    // risk-presence check sampled along the segment. This keeps genuine
    // clear-corridor shortcuts but blocks "shortcut through the restricted zone" ones.
    const double kRiskRejectLevel = 1.0;
    auto segmentRiskFree = [&](const Vector3d &a, const Vector3d &b) {
        if (!risk_zones_ || risk_zones_->empty()) return true;
        int n = std::max(1, (int)std::ceil((b - a).norm() / 0.5));
        for (int k = 0; k <= n; ++k) {
            double t = (double)k / (double)n;
            Vector3d p = a + t * (b - a);
            if (getRiskCost(p) > kRiskRejectLevel) return false;
        }
        return true;
    };

    std::vector<double> cum_cost(path.size(), 0.0);
    for (size_t k = 1; k < path.size(); ++k) {
        cum_cost[k] = cum_cost[k - 1] + segmentRiskCost(path[k - 1], path[k]);
    }

    vector<Vector3d> simple_path;
    simple_path.push_back(path.front());
    size_t i = 0;
    while (i + 1 < path.size()) {
        size_t farthest = i + 1;
        for (size_t j = path.size() - 1; j > i + 1; --j) {
            if (!segmentOccFree(path[i], path[j])) continue;
            if (!segmentRiskFree(path[i], path[j])) continue;
            double shortcut_cost = segmentRiskCost(path[i], path[j]);
            double detour_cost   = cum_cost[j] - cum_cost[i];
            if (shortcut_cost <= detour_cost * kShortcutMargin) {
                farthest = j;
                break;
            }
        }
        simple_path.push_back(path[farthest]);
        i = farthest;
    }
    if (log_manager_) {
        log_manager_->infof("[A* SHORTCUT] raw=%zu → simple=%zu (risk-aware, margin=%.2f)",
            path.size(), simple_path.size(), kShortcutMargin);
    }

    // Remove near points (3D distance)
    bool near_flag;
    do {
        near_flag = false;
        if (simple_path.size() <= 2) {
            break;
        }

        int num_same_check = simple_path.size();
        for (int i = 0; i < num_same_check - 1; i++) {
            double len = (simple_path[i+1] - simple_path[i]).norm();  // Full 3D norm
            if (len < 0.3) {
                simple_path.erase(simple_path.begin() + i + 1);
                near_flag = true;
                break;
            }
        }
    } while (near_flag);

    if (log_manager_) {
        log_manager_->infof("드론 %d: 3D 경로 단순화 완료 - %zu점 -> %zu점",
                           drone_id, path.size(), simple_path.size());

        double min_z = simple_path[0](2), max_z = simple_path[0](2);
        for (const auto& p : simple_path) {
            min_z = std::min(min_z, p(2));
            max_z = std::max(max_z, p(2));
        }
        log_manager_->infof("드론 %d: 단순화된 경로 Z 범위: %.3f ~ %.3f (변화량: %.3f)",
                           drone_id, min_z, max_z, max_z - min_z);
    }

    return simple_path;
}

}} // namespace path_planner::astar

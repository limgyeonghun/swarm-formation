#include "path_planner/dyn_a_star.h"
#include <cmath>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace Eigen;

namespace path_planner { namespace astar {

static void freePool(GridNodePtr ***&map, const Eigen::Vector3i &size) {
    if (!map) return;
    for (int i = 0; i < size(0); i++)
        for (int j = 0; j < size(1); j++)
            for (int k = 0; k < size(2); k++)
                delete map[i][j][k];
    for (int i = 0; i < size(0); i++) {
        for (int j = 0; j < size(1); j++) delete[] map[i][j];
        delete[] map[i];
    }
    delete[] map;
    map = nullptr;
}

AStar::~AStar()
{
    freePool(GridNodeMap_, POOL_SIZE_);
}

void AStar::initGridMap(const Eigen::Vector3i &pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }
}

void AStar::resizePool(const Eigen::Vector3i &pool_size)
{
    freePool(GridNodeMap_, POOL_SIZE_);
    initGridMap(pool_size);
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

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

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}


vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    if (!current) {
        return path;  // Return empty path if current is null
    }

    while (current != NULL)
    {
        path.push_back(current);
        current = current->cameFrom;
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
    }
    
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

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            auto time_2 = rclcpp::Clock().now();
            auto elapsed = time_2 - time_1;
            if (log_manager_) {
                log_manager_->infof("3D A* 검색 성공! 반복: %d회, 시간: %.3fms", num_iter, elapsed.seconds()*1000);
            }
            printf("\033[34mA star iter:%d, time:%.3f\033[0m\n", num_iter, elapsed.seconds()*1000);
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET;

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
            neighborIdx(0) = (current->index)(0) + dx;
            neighborIdx(1) = (current->index)(1) + dy;
            neighborIdx(2) = (current->index)(2) + dz;

            if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || 
                neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || 
                neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
            {
                continue;
            }
            
            neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
            if (!neighborPtr) {
                continue;
            }
            neighborPtr->index = neighborIdx;

            bool flag_explored = neighborPtr->rounds == rounds_;

            if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
            {
                continue;
            }

            neighborPtr->rounds = rounds_;

            if(use_esdf_check){
                if (checkOccupancy_esdf(Index2Coord(neighborPtr->index)))
                    continue;
            } else {
                if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    continue;
            }

            double static_cost = neighbor_costs_ordered[i];

            // Add threat cost if threat zones are enabled
            double threat_cost = getThreatCost(Index2Coord(neighborPtr->index));
            tentative_gScore = current->gScore + static_cost + threat_cost;

            if (!flag_explored)
            {
                neighborPtr->state = GridNode::OPENSET;
                neighborPtr->cameFrom = current;
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                openSet_.push(neighborPtr);
            }
            else if (tentative_gScore < neighborPtr->gScore)
            {
                neighborPtr->cameFrom = current;
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                openSet_.push(neighborPtr);
            }
        }

        auto time_2 = rclcpp::Clock().now();
        auto elapsed = time_2 - time_1;
        // 10 s cap: global (one-shot) planning can afford a long front-end
        // search; the 0.5 s limit in the upstream code was tuned for on-board
        // real-time local replan.
        if (elapsed.seconds() > 10.0)
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

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

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

    vector<Vector3d> simple_path;
    int end_idx = 1;
    Vector3d cut_start = path[0];
    simple_path.push_back(cut_start);

    bool finish = false;
    int safety_counter = 0;
    const int MAX_ITERATIONS = 1000;
    int prev_end_idx = -1;

    while (!finish && safety_counter < MAX_ITERATIONS) {
        safety_counter++;

        // Detect infinite loop
        if (end_idx == prev_end_idx) {
            if (log_manager_) {
                log_manager_->warnf("드론 %d: 단순화에서 정체 감지 (idx=%d) - 강제 진행", drone_id, end_idx);
            }
            if (end_idx < size - 1) {
                simple_path.push_back(path[end_idx]);
                end_idx++;
            } else {
                finish = true;
            }
            continue;
        }
        prev_end_idx = end_idx;

        bool made_progress = false;
        for (int i = end_idx; i < size; i++) {
            bool is_safe = true;
            Vector3d check_pt = path[i];

            // 3D collision checking along the straight line
            int check_num = ceil((check_pt - cut_start).norm() / 0.01);

            for (int j = 0; j <= check_num; j++) {
                double alpha = double(1.0 / check_num) * j;
                Vector3d check_safe_pt = (1 - alpha) * cut_start + alpha * check_pt;

                // Full 3D occupancy check (Z coordinate is naturally interpolated)
                if (checkOccupancy_esdf(check_safe_pt)) {
                    is_safe = false;
                    break;
                }
            }

            if (is_safe && i == (size - 1)) {
                finish = true;
                simple_path.push_back(check_pt);
                made_progress = true;
                break;
            }

            if (is_safe) {
                continue;  // Keep checking farther points
            } else {
                // Found collision - add previous safe point
                made_progress = true;
                if (i == end_idx) {
                    cut_start = path[i];
                    simple_path.push_back(cut_start);
                    end_idx = i + 1;
                } else {
                    cut_start = path[i - 1];
                    simple_path.push_back(cut_start);
                    end_idx = i;
                }
                break;
            }
        }

        if (!made_progress && end_idx >= size - 1) {
            finish = true;
        }
    }

    if (safety_counter >= MAX_ITERATIONS) {
        if (log_manager_) {
            log_manager_->errorf("드론 %d: 경로 단순화 무한 루프 감지 - 원본 경로 반환", drone_id);
        }
        RCLCPP_ERROR(rclcpp::get_logger("astar"),
                     "Drone %d: Infinite loop in path simplification - returning original path", drone_id);
        return path;
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

    // Split long segments (3D distance). For segments longer than the
    // threshold we insert enough equally-spaced points in one pass — the
    // previous "midpoint bisection + retry" loop bailed out at 10 iterations
    // and left ~15m jumps on long diagonals, which then placed the MINCO
    // initial trajectory inside obstacles.
    const double length_threshold = 1.5;
    {
        std::vector<Vector3d> densified;
        densified.reserve(simple_path.size() * 2);
        for (size_t i = 0; i + 1 < simple_path.size(); ++i) {
            densified.push_back(simple_path[i]);
            double leng = (simple_path[i + 1] - simple_path[i]).norm();
            if (leng > length_threshold) {
                int n_splits = static_cast<int>(std::ceil(leng / length_threshold));
                for (int s = 1; s < n_splits; ++s) {
                    double alpha = static_cast<double>(s) / n_splits;
                    densified.push_back(
                        simple_path[i] * (1.0 - alpha) + simple_path[i + 1] * alpha);
                }
            }
        }
        densified.push_back(simple_path.back());
        simple_path = std::move(densified);
    }

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

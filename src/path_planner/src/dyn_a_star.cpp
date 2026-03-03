#include "path_planner/dyn_a_star.h"
#include <cmath>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace Eigen;

void AStar::setLogManager(swarm_formation::LogManager::Ptr log_manager) {
    log_manager_ = log_manager;
    if (log_manager_) {
        log_manager_->info("A* 로그 매니저가 설정되었습니다.");
    }
}

AStar::~AStar()
{
    if (log_manager_) {
        log_manager_->info("A* 소멸자 호출 - 메모리 정리 시작");
    }
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
    if (log_manager_) {
        log_manager_->info("A* 메모리 정리 완료");
    }
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;
    
    if (log_manager_) {
        log_manager_->infof("그리드 맵 초기화 - Pool size: (%d,%d,%d), Center: (%d,%d,%d)", 
                           POOL_SIZE_(0), POOL_SIZE_(1), POOL_SIZE_(2),
                           CENTER_IDX_(0), CENTER_IDX_(1), CENTER_IDX_(2));
    }

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

    grid_map_ = occ_map;
    
    if (log_manager_) {
        log_manager_->info("그리드 맵 초기화 완료");
    }
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

double AStar::getDiagHeu2D(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    double h = 0.0;
    int diag = min(dx, dy);
    dx -= diag;
    dy -= diag;

    h = 1.414213562373095 * diag + (dx + dy);
    return h;
}

double AStar::getManhHeu2D(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    return dx + dy;
}

double AStar::getEuclHeu2D(GridNodePtr node1, GridNodePtr node2)
{
    double dx = node1->index(0) - node2->index(0);
    double dy = node1->index(1) - node2->index(1);

    return dx * dx + dy * dy;
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
        if (elapsed.seconds() > 0.2)
        {
            if (log_manager_) {
                log_manager_->warnf("3D A* 검색 시간 초과 - %.3fms 경과, 반복: %d회", elapsed.seconds()*1000, num_iter);
            }
            RCLCPP_WARN(rclcpp::get_logger("astar"), "Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
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

bool AStar::AstarSearch2D(const double step_size, Vector3d start_pt, Vector3d end_pt, bool use_esdf_check)
{
    auto time_1 = rclcpp::Clock().now();
    ++rounds_;
    
    if (log_manager_) {
        log_manager_->infof("2D A* 검색 시작 - Round: %d, Step size: %.3f, ESDF 사용: %s", 
                           rounds_, step_size, use_esdf_check ? "Yes" : "No");
        log_manager_->infof("시작점: (%.2f,%.2f,%.2f), 도착점: (%.2f,%.2f,%.2f)", 
                           start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));
    }
    
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    if (log_manager_) {
        log_manager_->debugf("2D 검색 중심점: (%.2f,%.2f,%.2f)", center_(0), center_(1), center_(2));
    }
    RCLCPP_INFO(rclcpp::get_logger("astar"), "2D A* CENTER: (%f,%f,%f)", center_(0), center_(1), center_(2));

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        if (log_manager_) {
            log_manager_->error("2D A* 검색 실패 - 시작/끝점 처리 불가");
        }
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Unable to handle the initial or end point in 2D search, force return!");
        return false;
    }

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu2D(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET;
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr);

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1))
        {
            auto time_2 = rclcpp::Clock().now();
            auto elapsed = time_2 - time_1;
            if (log_manager_) {
                log_manager_->infof("2D A* 검색 성공! 반복: %d회, 시간: %.3fms", num_iter, elapsed.seconds()*1000);
            }
            printf("\033[34m2D A star iter:%d, time:%.3f\033[0m\n", num_iter, elapsed.seconds()*1000);
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET;

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                Vector3i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;
                neighborIdx(2) = (current->index)(2);

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || 
                    neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || 
                    neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }
                
                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                if (!neighborPtr) {
                    RCLCPP_ERROR(rclcpp::get_logger("astar"), "Null pointer at GridNodeMap[%d][%d][%d]", 
                                neighborIdx(0), neighborIdx(1), neighborIdx(2));
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
                    if (checkOccupancy_esdf2D(Index2Coord(neighborPtr->index)))
                        continue;
                } else {
                    if (checkOccupancy2D(Index2Coord(neighborPtr->index)))
                        continue;
                }

                double static_cost;
                if (dx != 0 && dy != 0) {
                    static_cost = 1.414213562373095;
                } else {
                    static_cost = 1.0;
                }

                // Add threat cost if threat zones are enabled
                double threat_cost = getThreatCost(Index2Coord(neighborPtr->index));
                tentative_gScore = current->gScore + static_cost + threat_cost;

                if (!flag_explored)
                {
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu2D(neighborPtr, endPtr);
                    openSet_.push(neighborPtr);
                }
                else if (tentative_gScore < neighborPtr->gScore)
                {
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu2D(neighborPtr, endPtr);
                    openSet_.push(neighborPtr);
                }
            }
        
        auto time_2 = rclcpp::Clock().now();
        auto elapsed = time_2 - time_1;
        if (elapsed.seconds() > 0.3)
        {
            if (log_manager_) {
                log_manager_->warnf("2D A* 검색 시간 초과 - %.3fms 경과, 반복: %d회", elapsed.seconds()*1000, num_iter);
            }
            RCLCPP_WARN(rclcpp::get_logger("astar"), "Failed in 2D A star path searching !!! 0.3 seconds time limit exceeded.");
            return false;
        }
    }

    auto time_2 = rclcpp::Clock().now();
    auto elapsed_total = time_2 - time_1;

    if (log_manager_) {
        log_manager_->warnf("2D A* 검색 실패 - 전체 시간: %.3fms, 반복: %d회", elapsed_total.seconds()*1000, num_iter);
    }
    
    if (elapsed_total.seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("astar"), "Time consume in 2D A star path finding is %.3fs, iter=%d", elapsed_total.seconds(), num_iter);

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
    if (AstarSearch(step_size, start_pt, end_pt, true)) {
        vector<Vector3d> path = getPath();
        if (path.size() > 1 && (path[0]-start_pt).norm() < 0.5) {
            if (log_manager_) {
                log_manager_->infof("드론 %d: 3D A* 검색 성공 (ESDF 사용) - 경로 점 개수: %zu", drone_id, path.size());
            }
            RCLCPP_INFO(rclcpp::get_logger("astar"), "3D A* search successful with ESDF");
            return astarSearch2DAndGetSimplePath(step_size, start_pt, end_pt, drone_id, true);
        }
    }

    if (log_manager_) {
        log_manager_->warnf("드론 %d: 3D A* 검색 실패 (ESDF 사용), ESDF 없이 재시도", drone_id);
    }
    RCLCPP_WARN(rclcpp::get_logger("astar"), "3D A* search failed with ESDF, retrying without ESDF");

    // 3D A* search without ESDF
    if (AstarSearch(step_size, start_pt, end_pt, false)) {
        vector<Vector3d> path = getPath();
        if (path.size() > 1 && (path[0]-start_pt).norm() < 0.5) {
            if (log_manager_) {
                log_manager_->infof("드론 %d: 3D A* 검색 성공 (ESDF 비사용) - 경로 점 개수: %zu", drone_id, path.size());
            }
            RCLCPP_INFO(rclcpp::get_logger("astar"), "3D A* search successful without ESDF");
            return astarSearch2DAndGetSimplePath(step_size, start_pt, end_pt, drone_id, false);
        }
    }

    if (log_manager_) {
        log_manager_->errorf("드론 %d: 3D A* 검색 완전 실패 - 직선 경로 반환", drone_id);
    }
    RCLCPP_ERROR(rclcpp::get_logger("astar"), "3D A* search completely failed, returning direct path");

    vector<Vector3d> fallback_path;
    fallback_path.push_back(start_pt);
    fallback_path.push_back(end_pt);
    return fallback_path;
}

vector<Vector3d> AStar::astarSearch2DAndGetSimplePath(const double step_size, Vector3d start_pt, Vector3d end_pt, int drone_id, bool use_esdf_check){
    vector<Vector3d> path = getPath();
    bool is_show_debug = false;

    if (log_manager_) {
        log_manager_->infof("드론 %d: 2D 경로 단순화 시작 (ESDF %s) - 원본 경로 점 개수: %zu",
                           drone_id, use_esdf_check ? "사용" : "비사용", path.size());
    }

    if (path.size() <= 1 || (path[0]-start_pt).norm() > 0.5){
        if (log_manager_) {
            log_manager_->errorf("드론 %d: 2D 경로 단순화에서 잘못된 경로 감지 - 경로 점 개수: %zu", drone_id, path.size());
        }
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Invalid path in 2D simplification");
        vector<Vector3d> fallback_path;
        fallback_path.push_back(start_pt);
        fallback_path.push_back(end_pt);
        return fallback_path;
    }

    vector<Vector3d> simple_path;
    int size = path.size();
    if (size <= 2){
        if (log_manager_) {
            log_manager_->warnf("드론 %d: 2D 경로가 2점만 가지고 있음", drone_id);
        }
        RCLCPP_WARN(rclcpp::get_logger("astar"), "2D path only has two points");
        return path;
    }
        
    int end_idx   = 1;
    Vector3d cut_start = path[0];
    simple_path.push_back(cut_start);

    bool finish = false;
    int safety_counter = 0;
    const int MAX_ITERATIONS = 1000;  // Safety limit to prevent infinite loops
    int prev_end_idx = -1;  // Track progress to detect stuck state

    while (!finish && safety_counter < MAX_ITERATIONS) {
        safety_counter++;

        // Detect infinite loop: end_idx not progressing
        if (end_idx == prev_end_idx) {
            if (log_manager_) {
                log_manager_->warnf("simplifyPath: Stuck at index %d, path may be blocked", end_idx);
            }
            // Force progress by skipping one point
            if (end_idx < size - 1) {
                simple_path.push_back(path[end_idx]);
                end_idx++;
            } else {
                finish = true;  // Reached end
            }
            continue;
        }
        prev_end_idx = end_idx;

        bool made_progress = false;
        for (int i = end_idx; i < size; i++){
            bool is_safe = true;
            Vector3d check_pt = path[i];

            int check_num = ceil((check_pt - cut_start).norm() / 0.01);

            for (int j=0; j<=check_num; j++){
                double alpha = double(1.0 / check_num) * j;
                Vector3d check_safe_pt = (1 - alpha) * cut_start + alpha * check_pt;

                check_safe_pt(2) = start_pt(2);

                // Use the same occupancy check method as A* search
                bool is_occupied = use_esdf_check ? checkOccupancy_esdf2D(check_safe_pt) : checkOccupancy2D(check_safe_pt);
                if (is_occupied){
                    is_safe = false;
                    break;
                }
            }

            if (is_safe && i == (size -1)){
                finish = true;
                simple_path.push_back(check_pt);
                made_progress = true;
                break;
            }

            if (is_safe){
                // Continue checking next points
                continue;
            }

            // Found unsafe point - add previous safe point
            else {
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

        // If loop completed without progress, force finish
        if (!made_progress && end_idx >= size - 1) {
            finish = true;
        }
    }

    // Check if infinite loop was detected
    if (safety_counter >= MAX_ITERATIONS) {
        if (log_manager_) {
            log_manager_->errorf("드론 %d: 경로 단순화 무한 루프 감지! (iterations=%d) - 원본 경로 반환",
                                 drone_id, safety_counter);
        }
        RCLCPP_ERROR(rclcpp::get_logger("astar"),
                     "Drone %d: Infinite loop detected in path simplification (iterations=%d) - returning original path",
                     drone_id, safety_counter);
        return path;  // Return original unsimplified path
    }

    if (is_show_debug){
        cout << "[2D simple A* path] : --------- " << endl;
        int n1 = simple_path.size();
        cout << "2D simple A* path size : " << n1 << endl;
        for (int i=0; i<n1; i++)
            cout << simple_path[i].transpose() << endl;
    }

    bool near_flag;
    do
    {
        near_flag = false;
        if (simple_path.size() <=2){
            near_flag = false;
            break;
        }

        int num_same_check = simple_path.size();
        for (int i=0; i<num_same_check-1; i++){
            double len = sqrt(pow(simple_path[i+1](0) - simple_path[i](0), 2) + 
                             pow(simple_path[i+1](1) - simple_path[i](1), 2));
            if (len < 0.3){
                simple_path.erase(simple_path.begin()+i+1);
                near_flag = true;
                break;
            }
        }
        
    } while (near_flag);

    bool too_long_flag;
    const double length_threshold = 3.0;
    int debug_num = 0;
    do
    {
        debug_num ++;
        too_long_flag = false;
        int num = simple_path.size();
        for (int i=0; i<num-1; i++){
            double leng = sqrt(pow(simple_path[i+1](0) - simple_path[i](0), 2) + 
                              pow(simple_path[i+1](1) - simple_path[i](1), 2));
            if (leng > length_threshold){
                Vector3d insert_point = (simple_path[i+1] + simple_path[i]) / 2;
                simple_path.insert(simple_path.begin()+i+1 ,insert_point);
                too_long_flag = true;
                break;
            }
        }
    } while (too_long_flag && debug_num < 10);

    if (is_show_debug){
        cout << "[final 2D simple path] : --------- " << endl;
        int n3 = simple_path.size();
        cout << "final 2D simple path size : " << n3 << endl;
        for (int i=0; i<n3; i++)
            cout << simple_path[i].transpose() << endl;
    }
    
    if (log_manager_) {
        log_manager_->infof("드론 %d: 2D 경로 단순화 완료 - 최종 경로 점 개수: %zu", drone_id, simple_path.size());
    }
    
    return simple_path;    
}

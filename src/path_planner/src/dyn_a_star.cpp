#include "path_planner/dyn_a_star.h"
#include <cmath>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
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

    grid_map_ = occ_map;
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
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        RCLCPP_INFO(rclcpp::get_logger("astar"), "Start idx %d %d %d", start_idx(0), start_idx(1), start_idx(2));
        RCLCPP_INFO(rclcpp::get_logger("astar"), "Start point %f %f %f", start_pt(0), start_pt(1), start_pt(2));
        RCLCPP_WARN(rclcpp::get_logger("astar"), "Start point is inside an obstacle.");
        RCLCPP_WARN(rclcpp::get_logger("astar"), "Start point: (%f,%f,%f)", start_pt(0), start_pt(1), start_pt(2));
        do
        {
            start_pt = (end_pt - start_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
        RCLCPP_WARN(rclcpp::get_logger("astar"), "New start point: (%f,%f,%f)", start_pt(0), start_pt(1), start_pt(2));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        RCLCPP_WARN(rclcpp::get_logger("astar"), "End point is inside an obstacle.");
        RCLCPP_WARN(rclcpp::get_logger("astar"), "End point: (%f,%f,%f)", end_pt(0), end_pt(1), end_pt(2));
        do
        {
            end_pt = (start_pt - end_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
        RCLCPP_WARN(rclcpp::get_logger("astar"), "New END point: (%f,%f,%f)", end_pt(0), end_pt(1), end_pt(2));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt, bool use_esdf_check)
{
    auto time_1 = rclcpp::Clock().now();
    ++rounds_;
    
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    RCLCPP_INFO(rclcpp::get_logger("astar"), "CENTER: (%f,%f,%f)", center_(0), center_(1), center_(2));

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
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

        // 목표 도달 체크
        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            auto time_2 = rclcpp::Clock().now();
            auto elapsed = time_2 - time_1;
            printf("\033[34mA star iter:%d, time:%.3f\033[0m\n", num_iter, elapsed.seconds()*1000);
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET;

        // 우선순위 기반 이웃 탐색
        static const int neighbor_offsets[26][3] = {
            // 직접 이웃 (6개) - 가장 높은 우선순위
            {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
            // 대각선 이웃 (12개)
            {1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
            {1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
            {0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
            // 3D 대각선 (8개) - 가장 낮은 우선순위
            {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
            {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
        };  
        static const double neighbor_costs_ordered[26] = {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, // 직접 이웃
            1.414213562373095, 1.414213562373095, 1.414213562373095, 1.414213562373095,
            1.414213562373095, 1.414213562373095, 1.414213562373095, 1.414213562373095,
            1.414213562373095, 1.414213562373095, 1.414213562373095, 1.414213562373095, // 2D 대각선
            1.732050807568877, 1.732050807568877, 1.732050807568877, 1.732050807568877,
            1.732050807568877, 1.732050807568877, 1.732050807568877, 1.732050807568877  // 3D 대각선
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

            // 경계 체크
            if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || 
                neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || 
                neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
            {
                continue;
            }
            
            neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
            if (!neighborPtr) {
                continue; // 에러 로그 제거로 성능 향상
            }
            neighborPtr->index = neighborIdx;

            bool flag_explored = neighborPtr->rounds == rounds_;

            if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
            {
                continue;
            }

            neighborPtr->rounds = rounds_;

            // 장애물 체크 최적화 (좌표 변환 최소화)
            if(use_esdf_check){
                if (checkOccupancy_esdf(Index2Coord(neighborPtr->index)))
                    continue;
            } else {
                if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    continue;
            }
            
            // 미리 계산된 비용 사용
            double static_cost = neighbor_costs_ordered[i];
            tentative_gScore = current->gScore + static_cost;

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
                // 주의: priority_queue에서는 업데이트된 노드를 다시 push해야 함
                openSet_.push(neighborPtr);
            }
        }

        auto time_2 = rclcpp::Clock().now();
        auto elapsed = time_2 - time_1;
        if (elapsed.seconds() > 0.2)
        {
            RCLCPP_WARN(rclcpp::get_logger("astar"), "Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    auto time_2 = rclcpp::Clock().now();
    auto elapsed_total = time_2 - time_1;

    if (elapsed_total.seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("astar"), "Time consume in A star path finding is %.3fs, iter=%d", elapsed_total.seconds(), num_iter);

    return false;
}

bool AStar::AstarSearch2D(const double step_size, Vector3d start_pt, Vector3d end_pt, bool use_esdf_check)
{
    auto time_1 = rclcpp::Clock().now();
    ++rounds_;
    
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    RCLCPP_INFO(rclcpp::get_logger("astar"), "2D A* CENTER: (%f,%f,%f)", center_(0), center_(1), center_(2));

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
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
            printf("\033[34m2D A star iter:%d, time:%.3f\033[0m\n", num_iter, elapsed.seconds()*1000);
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET;

        // 2차원 탐색: dx, dy만 사용, dz는 0으로 고정
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                Vector3i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;
                neighborIdx(2) = (current->index)(2); // z축은 현재 레벨 유지

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
                
                // 2D 비용 계산 최적화 (sqrt 계산 최소화)
                double static_cost;
                if (dx != 0 && dy != 0) {
                    static_cost = 1.414213562373095;  // sqrt(2) 상수
                } else {
                    static_cost = 1.0;
                }
                tentative_gScore = current->gScore + static_cost;

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
                }
            }
        
        auto time_2 = rclcpp::Clock().now();
        auto elapsed = time_2 - time_1;
        if (elapsed.seconds() > 0.2)
        {
            RCLCPP_WARN(rclcpp::get_logger("astar"), "Failed in 2D A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    auto time_2 = rclcpp::Clock().now();
    auto elapsed_total = time_2 - time_1;

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
    // 2D 기반 탐색으로 통일 - z축 차이가 크면 시작점 z로 맞춤
    Vector3d adjusted_end_pt = end_pt;
    if (abs(start_pt(2) - end_pt(2)) > 1.0) {
        adjusted_end_pt(2) = start_pt(2);  // z축을 시작점과 동일하게 조정
        RCLCPP_INFO(rclcpp::get_logger("astar"), "Large z difference detected, adjusting end point z from %.2f to %.2f", 
                   end_pt(2), adjusted_end_pt(2));
    }
    
    // 2D 탐색 실행
    if (AstarSearch2D(step_size, start_pt, adjusted_end_pt, true)) {
        vector<Vector3d> path = getPath();
        if (path.size() > 1 && (path[0]-start_pt).norm() < 0.5) {
            RCLCPP_INFO(rclcpp::get_logger("astar"), "2D A* search successful");
            // 2D 경로 후처리로 바로 이동
            return astarSearch2DAndGetSimplePath(step_size, start_pt, adjusted_end_pt, drone_id);
        }
    }
    
    // 2D 탐색 실패시 ESDF 없이 재시도
    RCLCPP_WARN(rclcpp::get_logger("astar"), "2D A* search failed, retrying without ESDF");
    if (AstarSearch2D(step_size, start_pt, adjusted_end_pt, false)) {
        vector<Vector3d> path = getPath();
        if (path.size() > 1 && (path[0]-start_pt).norm() < 0.5) {
            RCLCPP_INFO(rclcpp::get_logger("astar"), "2D A* search successful without ESDF");
            return astarSearch2DAndGetSimplePath(step_size, start_pt, adjusted_end_pt, drone_id);
        }
    }
    
    // 2D 탐색이 완전히 실패한 경우에만 기본 경로 반환
    RCLCPP_ERROR(rclcpp::get_logger("astar"), "2D A* search completely failed, returning direct path");
    vector<Vector3d> fallback_path;
    fallback_path.push_back(start_pt);
    fallback_path.push_back(adjusted_end_pt);
    return fallback_path;    
}

vector<Vector3d> AStar::astarSearch2DAndGetSimplePath(const double step_size, Vector3d start_pt, Vector3d end_pt, int drone_id){
    // 2D A* 탐색 실행 (이미 호출되었으므로 경로만 가져옴)
    vector<Vector3d> path = getPath();
    bool is_show_debug = false;

    // 경로 검증 (이미 상위 함수에서 검증되었으므로 간단히 확인만)
    if (path.size() <= 1 || (path[0]-start_pt).norm() > 0.5){
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Invalid path in 2D simplification");
        vector<Vector3d> fallback_path;
        fallback_path.push_back(start_pt);
        fallback_path.push_back(end_pt);
        return fallback_path;
    }
    
    // 간소화된 경로 생성
    vector<Vector3d> simple_path;
    int size = path.size();
    if (size <= 2){
        RCLCPP_WARN(rclcpp::get_logger("astar"), "2D path only has two points");
        return path;
    }
        
    int end_idx   = 1;
    Vector3d cut_start = path[0];
    simple_path.push_back(cut_start);

    bool finish = false;
    while (!finish) {
        for (int i = end_idx; i < size; i++){
            bool is_safe = true;
            Vector3d check_pt = path[i];
            
            // 2D 경로에서는 더 정밀한 체크 간격 사용
            int check_num = ceil((check_pt - cut_start).norm() / 0.01);
            
            // 충돌 체크 (2D 최적화)
            for (int j=0; j<=check_num; j++){
                double alpha = double(1.0 / check_num) * j;
                Vector3d check_safe_pt = (1 - alpha) * cut_start + alpha * check_pt;
                
                // 2D 경로에서는 z축 높이를 시작점과 동일하게 유지
                check_safe_pt(2) = start_pt(2);
                
                if (checkOccupancy_esdf2D(check_safe_pt)){
                    is_safe = false;
                    break;
                }
            }
            
            if (is_safe && i == (size -1)){
                finish = true;
                simple_path.push_back(check_pt);
            }

            if (is_safe){
                continue;
            }
            // else{
            //     end_idx = i;
            //     cut_start = path[end_idx-1];
            //     simple_path.push_back(cut_start);
            // }
            else {
                if (i == end_idx) {              // (= s+1)
                    // 바로 다음 점도 막히면 '강제 한 칸 전진'
                    cut_start = path[i];         // (= path[s+1])
                    simple_path.push_back(cut_start);
                    end_idx = i + 1;             // 다음 턴은 i+1부터
                } else {
                    // i-1까지는 안전
                    cut_start = path[i - 1];
                    simple_path.push_back(cut_start);
                    end_idx = i;                 // 다음 턴은 i부터
                }
                break; // ★ 충돌을 만났으니 이 for 루프는 즉시 종료하고 while 다음 턴으로
            }
        }
    }

    // 디버그 출력
    if (is_show_debug){
        cout << "[2D simple A* path] : --------- " << endl;
        int n1 = simple_path.size();
        cout << "2D simple A* path size : " << n1 << endl;
        for (int i=0; i<n1; i++)
            cout << simple_path[i].transpose() << endl;
    }

    // 근접한 점들 제거
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
            // 2D 거리만 계산 (x, y만 고려)
            double len = sqrt(pow(simple_path[i+1](0) - simple_path[i](0), 2) + 
                             pow(simple_path[i+1](1) - simple_path[i](1), 2));
            if (len < 0.3){
                simple_path.erase(simple_path.begin()+i+1);
                near_flag = true;
                break;
            }
        }
        
    } while (near_flag);

    // 너무 긴 세그먼트 분할
    bool too_long_flag;
    const double length_threshold = 3.0;
    int debug_num = 0;
    do
    {
        debug_num ++;
        too_long_flag = false;
        int num = simple_path.size();
        for (int i=0; i<num-1; i++){
            // 2D 거리 계산
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

    // 최종 디버그 출력
    if (is_show_debug){
        cout << "[final 2D simple path] : --------- " << endl;
        int n3 = simple_path.size();
        cout << "final 2D simple path size : " << n3 << endl;
        for (int i=0; i<n3; i++)
            cout << simple_path[i].transpose() << endl;
    }
    
    return simple_path;    
}

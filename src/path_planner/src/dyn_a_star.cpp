#include "path_planner/dyn_a_star.h"
#include <algorithm>
#include <chrono>
#include <sys/resource.h>
#include <iostream>

using namespace Eigen;

void AStar::updateMemoryUsage() 
{
    // 현재 프로세스의 메모리 사용량 확인
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    
    // 현재 메모리 사용량 (KB 단위)
    current_memory_usage_ = usage.ru_maxrss;
    
    // 피크 메모리 사용량 업데이트
    if (current_memory_usage_ > peak_memory_usage_) {
        peak_memory_usage_ = current_memory_usage_;
    }
}

size_t AStar::estimateNodeMemoryUsage() const 
{
    // GridNode 크기 + 우선순위 큐 크기 + 경로 크기 예상
    size_t grid_node_size = sizeof(GridNode);
    size_t open_set_size = openSet_.size() * (sizeof(GridNodePtr) + sizeof(size_t)); // 포인터 + 우선순위 큐 오버헤드
    size_t path_size = gridPath_.size() * sizeof(GridNodePtr);
    
    return grid_node_size + open_set_size + path_size;
}

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++) {
        for (int j = 0; j < POOL_SIZE_(1); j++) {
            delete GridNodeMap_[i][j];
        }
        delete[] GridNodeMap_[i];
    }
    delete[] GridNodeMap_;
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector2i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr *[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNode;
        }
    }
    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = std::abs(node1->index(0) - node2->index(0));
    double dy = std::abs(node1->index(1) - node2->index(1));

    // 2D diagonal heuristic
    double h = 0.0;
    int diag = std::min(static_cast<int>(dx), static_cast<int>(dy));
    dx -= diag;
    dy -= diag;

    h = std::sqrt(2.0) * diag + dx + dy;
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = std::abs(node1->index(0) - node2->index(0));
    double dy = std::abs(node1->index(1) - node2->index(1));
    return dx + dy;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

std::vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    std::vector<GridNodePtr> path;
    path.push_back(current);
    while (current->cameFrom != nullptr)
    {
        current = current->cameFrom;
        path.push_back(current);
    }
    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt,
                                                   Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx)
{
    Eigen::Vector3d s_pt = start_pt;
    Eigen::Vector3d e_pt = end_pt;
    
    // First try to convert to index
    if (!Coord2Index(s_pt, start_idx) || !Coord2Index(e_pt, end_idx))
    {
        std::cerr << "Failed to convert points to indices. Start: " << s_pt.transpose() 
                  << ", End: " << e_pt.transpose() << std::endl;
        return false;
    }

    // Check if start point is in obstacle and adjust
    if (checkOccupancy(Index2Coord(start_idx)))
    {
        std::cerr << "Start point is inside an obstacle. Adjusting..." << std::endl;
        int max_attempts = 20;  // Increased from 10 to 20
        int attempts = 0;
        do
        {
            // Try multiple directions to escape obstacle
            Eigen::Vector3d direction;
            if (attempts < 5) {
                // First try moving away from end point
                direction = (s_pt - e_pt).normalized();
            } else if (attempts < 10) {
                // Try moving in perpendicular directions
                Eigen::Vector3d perp = Eigen::Vector3d(-(s_pt - e_pt).y(), (s_pt - e_pt).x(), 0.0).normalized();
                direction = (attempts % 2 == 0) ? perp : -perp;
            } else {
                // Try random directions
                direction = Eigen::Vector3d((attempts % 3 - 1), ((attempts / 3) % 3 - 1), 0.0).normalized();
            }
            
            s_pt = s_pt + direction * step_size_ * (1.0 + attempts * 0.1);  // Increase step size with attempts
            
            if (!Coord2Index(s_pt, start_idx))
            {
                std::cerr << "Failed to adjust start point after " << attempts << " attempts" << std::endl;
                return false;
            }
            attempts++;
        } while (checkOccupancy(Index2Coord(start_idx)) && attempts < max_attempts);
        
        if (attempts >= max_attempts)
        {
            std::cerr << "Failed to find valid start point after " << max_attempts << " attempts" << std::endl;
            return false;
        }
    }

    // Check if end point is in obstacle and adjust
    if (checkOccupancy(Index2Coord(end_idx)))
    {
        std::cerr << "End point is inside an obstacle. Adjusting..." << std::endl;
        int max_attempts = 10;
        int attempts = 0;
        do
        {
            // Move end point away from obstacle
            Eigen::Vector3d direction = (e_pt - s_pt).normalized();
            e_pt = e_pt + direction * step_size_;
            
            if (!Coord2Index(e_pt, end_idx))
            {
                std::cerr << "Failed to adjust end point after " << attempts << " attempts" << std::endl;
                return false;
            }
            attempts++;
        } while (checkOccupancy(Index2Coord(end_idx)) && attempts < max_attempts);
        
        if (attempts >= max_attempts)
        {
            std::cerr << "Failed to find valid end point after " << max_attempts << " attempts" << std::endl;
            return false;
        }
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_esdf_check)
{
    rclcpp::Time time_1 = rclcpp::Clock().now();
    ++rounds_;

    updateMemoryUsage();
    size_t start_memory = current_memory_usage_;
    
    // 메모리 사용량이 너무 높으면 A* 탐색 중단
    const size_t MEMORY_LIMIT = 8000000; // 8GB (KB 단위)
    if (current_memory_usage_ > MEMORY_LIMIT) {
        std::cerr << "[ERROR] Memory usage too high (" << current_memory_usage_ / 1024 
                  << " MB). Aborting A* search to prevent OOM." << std::endl;
        return false;
    }
    
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Eigen::Vector2i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        std::cerr << "Unable to handle the initial or end point, force return!" << std::endl;
        return false;
    }

    // std::cerr << "POOL_SIZE: " << POOL_SIZE_.transpose() << std::endl;
    // std::cerr << "Start idx: " << start_idx.transpose() << " End idx: " << end_idx.transpose() << std::endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)];

    // while (!openSet_.empty()) openSet_.pop();
    openSet_ = decltype(openSet_)();

    GridNodePtr neighborPtr = nullptr;
    GridNodePtr current = nullptr;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET;
    startPtr->cameFrom = nullptr;
    openSet_.push(startPtr);

    endPtr->index = end_idx;

    double tentative_gScore;
    int num_iter = 0;
    const int max_iterations = 500000; // 최대 반복 횟수 제한
    const int max_open_set_size = 100000; // 오픈셋 최대 크기 제한
    const size_t max_memory_increase_mb = 1000; // 최대 메모리 증가량 제한 (MB)
    
    while (!openSet_.empty())
    {
        // 메모리 사용량 제한 확인 (100회 반복마다 체크)
        if (num_iter % 100 == 0) {
            updateMemoryUsage();
            size_t memory_increase = (current_memory_usage_ > start_memory) ? 
                                     (current_memory_usage_ - start_memory) / 1024 : 0; // MB 단위로 변환
            
            if (memory_increase > max_memory_increase_mb) {
                std::cerr << "Failed in A* path searching !!! Memory usage limit exceeded. "
                          << "Current: " << current_memory_usage_ / 1024 << " MB, "
                          << "Increase: " << memory_increase << " MB" << std::endl;
                return false;
            }
        }
        
        // 오픈셋 크기 제한 확인
        if (openSet_.size() > max_open_set_size) {
            std::cerr << "Failed in A* path searching !!! Memory limit exceeded. Open set size: " 
                      << openSet_.size() << std::endl;
            return false;
        }

        // 최대 반복 횟수 제한 확인
        if (num_iter > max_iterations) {
            std::cerr << "Failed in A* path searching !!! Maximum iterations (" 
                      << max_iterations << ") exceeded." << std::endl;
            return false;
        }

        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        if (current->index == endPtr->index)
        {
            rclcpp::Time time_2 = rclcpp::Clock().now();
            rclcpp::Duration elapsed = time_2 - time_1;
            updateMemoryUsage();
            size_t memory_increase = (current_memory_usage_ > start_memory) ? 
                                    (current_memory_usage_ - start_memory) / 1024 : 0;
            
            std::cout << "A* iter:" << num_iter << ", time:" << elapsed.seconds() * 1000 
                      << " ms, memory:" << current_memory_usage_ / 1024 << " MB (+" 
                      << memory_increase << " MB)" << std::endl;
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET;

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                {
                    if (dx == 0 && dy == 0)
                        continue;

                    Eigen::Vector2i neighborIdx;
                    neighborIdx(0) = current->index(0) + dx;
                    neighborIdx(1) = current->index(1) + dy;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 ||
                        neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = (neighborPtr->rounds == rounds_);

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                        continue;

                    neighborPtr->rounds = rounds_;

                    if (use_esdf_check) {
                        if (checkOccupancy_esdf(Index2Coord(neighborPtr->index)))
                            continue;
                    } else {
                        if (checkOccupancy(Index2Coord(neighborPtr->index)))
                            continue;
                    }
                    
                    double static_cost = std::sqrt(dx * dx + dy * dy);
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
                    }
                }
        // Time limit check - only every 100 iterations to reduce overhead
        if (num_iter % 100 == 0) {
            rclcpp::Time time_2 = rclcpp::Clock().now();
            rclcpp::Duration elapsed = time_2 - time_1;
            if (elapsed.seconds() > 0.2)
            {
                std::cerr << "Failed in A* path searching !!! 0.2 seconds time limit exceeded." << std::endl;
                return false;
            }
        }
    }

    rclcpp::Time time_2 = rclcpp::Clock().now();
    rclcpp::Duration elapsed_total = time_2 - time_1;
    updateMemoryUsage();
    size_t memory_increase = (current_memory_usage_ > start_memory) ? 
                           (current_memory_usage_ - start_memory) / 1024 : 0; // MB 단위로 변환
    
    std::cerr << "Failed in A* path finding! Time: " << elapsed_total.seconds() 
              << " s, iter=" << num_iter 
              << ", memory: " << current_memory_usage_ / 1024 << " MB (+" 
              << memory_increase << " MB)" << std::endl;
    return false;
}

std::vector<Eigen::Vector3d> AStar::getPath()
{
    std::vector<Eigen::Vector3d> path;
    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));
    std::reverse(path.begin(), path.end());
    return path;
}

vector<Vector3d> AStar::astarSearchAndGetSimplePath(const double step_size, Vector3d start_pt, Vector3d end_pt){

    rclcpp::Time time_1 = rclcpp::Clock().now();
    updateMemoryUsage();
    size_t start_memory = current_memory_usage_;

    // call astar search and get the path
    bool success = AstarSearch(step_size, start_pt, end_pt, true);
    vector<Vector3d> path = getPath();
    bool is_show_debug = false;

    // 경로 찾기 실패 시 그레이스풀 복구 메커니즘
    if (!success || (!path.empty() && (path[0]-start_pt).norm() > 0.5)) {
        std::cerr << "First A* attempt failed or path start point mismatch. Trying recovery strategies." << std::endl;
        
        // 전략 1: 스텝 사이즈 증가 (더 큰 그리드 사용)
        bool second_attempt = AstarSearch(step_size * 1.5, start_pt, end_pt, false);
        if (second_attempt) {
            std::cerr << "Recovery succeeded with increased step size." << std::endl;
            path = getPath();
        } else {
            // 전략 2: 시작점과 목표점 사이의 중간점을 통해 경로 찾기
            Vector3d mid_pt = (start_pt + end_pt) * 0.5;
            std::cerr << "Trying to find path through midpoint: " << mid_pt.transpose() << std::endl;
            
            bool first_half = AstarSearch(step_size * 1.2, start_pt, mid_pt, false);
            if (first_half) {
                vector<Vector3d> first_path = getPath();
                bool second_half = AstarSearch(step_size * 1.2, mid_pt, end_pt, false);
                if (second_half) {
                    vector<Vector3d> second_path = getPath();
                    // 두 경로 합치기
                    path = first_path;
                    path.insert(path.end(), second_path.begin(), second_path.end());
                    std::cerr << "Successfully found path through midpoint." << std::endl;
                } else {
                    // 전략 3: 더 단순한 직선 경로 시도
                    std::cerr << "Trying simplified direct path..." << std::endl;
                    // 시작점과 끝점을 직접 연결하는 간단한 경로 생성
                    path.clear();
                    path.push_back(start_pt);
                    
                    // 시작점과 끝점 사이에 몇 개의 중간점 추가
                    int num_points = 5;
                    for (int i = 1; i < num_points; i++) {
                        double ratio = static_cast<double>(i) / num_points;
                        path.push_back(start_pt * (1 - ratio) + end_pt * ratio);
                    }
                    
                    path.push_back(end_pt);
                    std::cerr << "Created simplified direct path with " << path.size() << " points." << std::endl;
                }
            } else {
                std::cerr << "All recovery attempts failed. Returning empty path." << std::endl;
                return vector<Vector3d>(); // 빈 경로 반환
            }
        }
    }
    
    updateMemoryUsage();
    size_t memory_increase = (current_memory_usage_ > start_memory) ? 
                           (current_memory_usage_ - start_memory) / 1024 : 0;
    std::cout << "Path planning memory usage: " << current_memory_usage_ / 1024 
              << " MB (+" << memory_increase << " MB)" << std::endl;

    grid_map_->updateESDFLocal(start_pt);

    // generate the simple path
    vector<Vector3d> simple_path;
    int size = path.size();
    if (size <= 2){
        std::cerr << "the path only have two points" << std::endl;
        return path;
    }

    int end_idx   = 1;
    Vector3d cut_start = path[0];
    simple_path.push_back(cut_start);

    bool finish = false;
    static long long total_esdf_calls = 0;
    long long esdf_calls_before = total_esdf_calls;

    while (!finish) {
        for (int i = end_idx; i < size; i++){
            bool is_safe = true;
            Vector3d check_pt = path[i];
            int check_num = std::max(1, (int)std::ceil((check_pt - cut_start).norm() / 0.1));

            // check collision
            for (int j=0; j<=check_num; j++){
                double alpha = double(1.0 / check_num) * j;
                Vector3d check_safe_pt = (1 - alpha) * cut_start + alpha * check_pt;
                total_esdf_calls++;
                if (checkOccupancy_esdf(check_safe_pt)){
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
            } else{
                end_idx = i;
                cut_start = path[end_idx-1];
                simple_path.push_back(cut_start);
            }
        }
    }

    // rclcpp::Time time_3 = rclcpp::Clock().now();
    // rclcpp::Duration elapsed_total2 = time_3 - time_2;
    // std::cerr << "STEP2 " << elapsed_total2.seconds() <<  std::endl;
    // std::cerr << "[DBG] simple_path.size=" << simple_path.size()
    //           << ", ESDF calls in STEP2=" << (total_esdf_calls - esdf_calls_before)
    //           << ", ESDF calls total=" << total_esdf_calls << "\n";

    // debug
    if (is_show_debug){
        cout << "[simple A* path] : --------- " << endl;
        int n1 = simple_path.size();
        cout << "simple A* path size : " << n1 << endl;
        for (int i=0; i<n1; i++)
            cout << simple_path[i].transpose() << endl;
    }

    // check the near points and delete it
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
            double len = (simple_path[i+1] - simple_path[i]).norm();
            if (len < 0.3){
                simple_path.erase(simple_path.begin()+i+1);
                near_flag = true;
                break;
            }
        }

    } while (near_flag);

    // rclcpp::Time time_4 = rclcpp::Clock().now();
    // rclcpp::Duration elapsed_total3 = time_4 - time_3;
    // std::cerr << "STEP3 " << elapsed_total3.seconds() << std::endl;

    // debug
    if (is_show_debug){
        cout << "[delete simple path] : --------- " << endl;
        int n2 = simple_path.size();
        cout << "delete simple path size : " << n2 << endl;
        for (int i=0; i<n2; i++)
            cout << simple_path[i].transpose() << endl;
    }

    // check the path and add a point if two of them are too far away
    bool too_long_flag;
    const double length_threshold = 3;
    int debug_num = 0;
    do
    {
        debug_num ++;
        too_long_flag = false;
        int num = simple_path.size();
        for (int i=0; i<num-1; i++){
            double leng = (simple_path[i+1] - simple_path[i]).norm();
            if (leng > length_threshold){
                Vector3d insert_point = (simple_path[i+1] + simple_path[i]) / 2;
                simple_path.insert(simple_path.begin()+i+1 ,insert_point);
                too_long_flag = true;
                break;
            }
        }
    } while (too_long_flag && debug_num < 10);

    // rclcpp::Time time_5 = rclcpp::Clock().now();
    // rclcpp::Duration elapsed_total4 = time_5 - time_4;
    // std::cerr << "STEP4 " << elapsed_total4.seconds() <<  std::endl;

    // debug
    if (is_show_debug){
        cout << "[final simple path] : --------- " << endl;
        int n3 = simple_path.size();
        cout << "final simple path size : " << n3 << endl;
        for (int i=0; i<n3; i++)
            cout << simple_path[i].transpose() << endl;
    }

    return simple_path;
}
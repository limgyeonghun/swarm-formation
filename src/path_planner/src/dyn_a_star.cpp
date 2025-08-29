#include "path_planner/dyn_a_star.h"
#include <algorithm>
#include <chrono>

using namespace Eigen;

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
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        if (current->index == endPtr->index)
        {
            rclcpp::Time time_2 = rclcpp::Clock().now();
            rclcpp::Duration elapsed = time_2 - time_1;
            std::cout << "A* iter:" << num_iter << ", time:" << elapsed.seconds() * 1000 << " ms" << std::endl;
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
    if (elapsed_total.seconds() > 0.1)
        std::cerr << "Time consumed in A* path finding is " << elapsed_total.seconds() << " s, iter=" << num_iter << std::endl;
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

    // call astar search and get the path
    AstarSearch(step_size, start_pt, end_pt, true);
    vector<Vector3d> path = getPath();
    bool is_show_debug = false;

    // double res = grid_map_ ? grid_map_->getResolution() : -1.0;
    // double total_len = 0.0;
    // for (int i = 0; i + 1 < (int)path.size(); ++i)
    //     total_len += (path[i+1] - path[i]).norm();

    // std::cerr << "[DBG] POOL_SIZE: " << POOL_SIZE_(0) << " " << POOL_SIZE_(1) << " " << POOL_SIZE_(2) << "\n";
    // std::cerr << "[DBG] center: " << center_.transpose() << "\n";
    // std::cerr << "[DBG] step_size(arg)=" << step_size << ", res=" << res << "\n";
    // std::cerr << "[DBG] raw path.size=" << path.size() << ", total_len=" << total_len << " m\n";

    // rclcpp::Time time_2 = rclcpp::Clock().now();
    // rclcpp::Duration elapsed_total1 = time_2 - time_1;
    // std::cerr << "STEP1 " << elapsed_total1.seconds() <<  std::endl;

    // I don't know why, but only try A* again
    if (!path.empty() && (path[0]-start_pt).norm() > 0.5){
        std::cerr << "I don't know why, but only try A* again" << std::endl;
        AstarSearch(step_size, start_pt, end_pt, false);
        path = getPath();
    }

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
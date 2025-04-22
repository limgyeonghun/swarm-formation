#include "path_planner/dyn_a_star.h"
#include <algorithm>
#include <chrono>

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];

    for (int i = 0; i < POOL_SIZE_(0); i++) {
        for (int j = 0; j < POOL_SIZE_(1); j++) {
            delete[] GridNodeMap_[i][j];
        }
        delete[] GridNodeMap_[i];
    }
    delete[] GridNodeMap_;
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
    double dx = std::abs(node1->index(0) - node2->index(0));
    double dy = std::abs(node1->index(1) - node2->index(1));
    double dz = std::abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = std::min({static_cast<int>(dx), static_cast<int>(dy), static_cast<int>(dz)});
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * std::sqrt(3.0) * diag + std::sqrt(2.0) * std::min(dy, dz) + 1.0 * std::abs(dy - dz);
    }
    else if (dy == 0)
    {
        h = 1.0 * std::sqrt(3.0) * diag + std::sqrt(2.0) * std::min(dx, dz) + 1.0 * std::abs(dx - dz);
    }
    else if (dz == 0)
    {
        h = 1.0 * std::sqrt(3.0) * diag + std::sqrt(2.0) * std::min(dx, dy) + 1.0 * std::abs(dx - dy);
    }
    else {
        h = std::sqrt(3.0) * diag + std::sqrt(2.0) * (std::max({dx, dy, dz}) - diag);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = std::abs(node1->index(0) - node2->index(0));
    double dy = std::abs(node1->index(1) - node2->index(1));
    double dz = std::abs(node1->index(2) - node2->index(2));
    return dx + dy + dz;
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
                                                   Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
{
    Eigen::Vector3d s_pt = start_pt;
    Eigen::Vector3d e_pt = end_pt;
    if (!Coord2Index(s_pt, start_idx) || !Coord2Index(e_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        std::cerr << "Start point is inside an obstacle." << std::endl;
        do
        {
            s_pt = (s_pt - e_pt).normalized() * step_size_ + s_pt;
            if (!Coord2Index(s_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        std::cerr << "End point is inside an obstacle." << std::endl;
        do
        {
            e_pt = (e_pt - s_pt).normalized() * step_size_ + e_pt;
            if (!Coord2Index(e_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_esdf_check)
{
    auto time_1 = std::chrono::steady_clock::now();
    ++rounds_;
    
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Eigen::Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        std::cerr << "Unable to handle the initial or end point, force return!" << std::endl;
        return false;
    }

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    while (!openSet_.empty()) openSet_.pop();

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
            auto time_2 = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = time_2 - time_1;
            std::cout << "A* iter:" << num_iter << ", time:" << elapsed.count()*1000 << " ms" << std::endl;
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET;

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Eigen::Vector3i neighborIdx;
                    neighborIdx(0) = current->index(0) + dx;
                    neighborIdx(1) = current->index(1) + dy;
                    neighborIdx(2) = current->index(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 ||
                        neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 ||
                        neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
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
                    
                    double static_cost = std::sqrt(dx * dx + dy * dy + dz * dz);
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
        auto time_2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = time_2 - time_1;
        if (elapsed.count() > 0.2)
        {
            std::cerr << "Failed in A* path searching !!! 0.2 seconds time limit exceeded." << std::endl;
            return false;
        }
    }

    auto time_2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_total = time_2 - time_1;
    if (elapsed_total.count() > 0.1)
        std::cerr << "Time consumed in A* path finding is " << elapsed_total.count() << " s, iter=" << num_iter << std::endl;
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

std::vector<Eigen::Vector3d> AStar::astarSearchAndGetSimplePath(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    AstarSearch(step_size, start_pt, end_pt, true);
    std::vector<Eigen::Vector3d> path = getPath();
    // bool is_show_debug = false;

    if ((path[0] - start_pt).norm() > 0.5) {
        std::cerr << "I don't know why, but only try A* again" << std::endl;
        AstarSearch(step_size, start_pt, end_pt, false);
        path = getPath();
    }
    
    std::vector<Eigen::Vector3d> simple_path;
    int size = path.size();
    if (size <= 2) {
        std::cerr << "The path only has two points" << std::endl;
        return path;
    }
        
    int end_idx = 1;
    Eigen::Vector3d cut_start = path[0];
    simple_path.push_back(cut_start);
    
    bool finish = false;
    while (!finish) {
        for (int i = end_idx; i < size; i++) {
            bool is_safe = true;
            Eigen::Vector3d check_pt = path[i];
            int check_num = std::ceil((check_pt - cut_start).norm() / 0.01);
            for (int j = 0; j <= check_num; j++) {
                double alpha = (1.0 / check_num) * j;
                Eigen::Vector3d check_safe_pt = (1 - alpha) * cut_start + alpha * check_pt;
                if (checkOccupancy_esdf(check_safe_pt)) {
                    is_safe = false;
                    break;
                }
            }
            
            if (is_safe && i == (size - 1)) {
                finish = true;
                simple_path.push_back(check_pt);
            }
            if (is_safe) {
                continue;
            } else {
                end_idx = i;
                cut_start = path[end_idx - 1];
                simple_path.push_back(cut_start);
            }
        }
    }

    bool near_flag;
    do
    {
        near_flag = false;
        if (simple_path.size() <= 2)
            break;
        int num_same_check = simple_path.size();
        for (int i = 0; i < num_same_check - 1; i++) {
            double len = (simple_path[i + 1] - simple_path[i]).norm();
            if (len < 0.3) {
                simple_path.erase(simple_path.begin() + i + 1);
                near_flag = true;
                break;
            }
        }
    } while (near_flag);

    bool too_long_flag;
    const double length_threshold = 3;
    int debug_num = 0;
    do
    {
        debug_num++;
        too_long_flag = false;
        int num = simple_path.size();
        for (int i = 0; i < num - 1; i++) {
            double leng = (simple_path[i + 1] - simple_path[i]).norm();
            if (leng > length_threshold) {
                Eigen::Vector3d insert_point = (simple_path[i + 1] + simple_path[i]) / 2;
                simple_path.insert(simple_path.begin() + i + 1, insert_point);
                too_long_flag = true;
                break;
            }
        }
    } while (too_long_flag && debug_num < 10);

    return simple_path;
}

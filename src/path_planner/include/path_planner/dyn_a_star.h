#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <cmath>
#include <memory>

#include "path_planner/grid_map.h"

constexpr double inf = 1 >> 20;  

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
    enum enum_state state = UNDEFINED;
    Eigen::Vector2i index;  // Changed to 2D for rover
    double gScore{inf}, fScore{inf};
    GridNodePtr cameFrom{nullptr};
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
    GridMap::Ptr grid_map_;

    double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
    double getManhHeu(GridNodePtr node1, GridNodePtr node2);
    double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
    inline double getHeu(GridNodePtr node1, GridNodePtr node2);

    bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt,
                                                 Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx);

    inline Eigen::Vector3d Index2Coord(const Eigen::Vector2i &index) const;
    inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector2i &idx) const;

    inline bool checkOccupancy(const Eigen::Vector3d &pos) { return (bool)grid_map_->getInflateOccupancy(pos); }
    inline bool checkOccupancy_esdf(const Eigen::Vector3d &pos) {
        double dist;
        grid_map_->evaluateEDT(pos, dist);
        return dist < 0.2; // 0.2m dist
    }

    std::vector<GridNodePtr> retrievePath(GridNodePtr current);

    double step_size_, inv_step_size_;
    Eigen::Vector3d center_;
    Eigen::Vector2i CENTER_IDX_, POOL_SIZE_;  // Changed to 2D
    const double tie_breaker_ = 1.0 + 1.0 / 10000;

    std::vector<GridNodePtr> gridPath_;

    GridNodePtr **GridNodeMap_;  // Changed to 2D array
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

    int rounds_{0};
    
    // 메모리 사용량 모니터링 변수
    size_t peak_memory_usage_{0};
    size_t current_memory_usage_{0};
    
    // 메모리 사용량 추적 함수
    void updateMemoryUsage();
    size_t estimateNodeMemoryUsage() const;

public:
    typedef std::shared_ptr<AStar> Ptr;

    AStar(){};
    ~AStar();

    void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector2i pool_size);  // Changed to 2D

    bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_esdf_check);

    std::vector<Eigen::Vector3d> getPath();

    std::vector<Eigen::Vector3d> astarSearchAndGetSimplePath(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    
    // 메모리 사용량 정보 반환 함수
    size_t getPeakMemoryUsage() const { return peak_memory_usage_; }
    size_t getCurrentMemoryUsage() const { return current_memory_usage_; }
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector2i &index) const
{
    Eigen::Vector3d coord;
    coord(0) = ((index(0) - CENTER_IDX_(0)) * step_size_) + center_(0);
    coord(1) = ((index(1) - CENTER_IDX_(1)) * step_size_) + center_(1);
    coord(2) = center_(2);  // Keep z at center level for 2D
    return coord;
}

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector2i &idx) const
{
    // Convert 3D point to 2D index for rover
    double x_offset = pt(0) - center_(0);
    double y_offset = pt(1) - center_(1);
    
    idx(0) = static_cast<int>(x_offset * inv_step_size_ + 0.5) + CENTER_IDX_(0);
    idx(1) = static_cast<int>(y_offset * inv_step_size_ + 0.5) + CENTER_IDX_(1);
    
    // Check bounds
    if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) ||
        idx(1) < 0 || idx(1) >= POOL_SIZE_(1))
    {
        std::cerr << "Ran out of pool, index=" << idx(0) << " " << idx(1) 
                  << ", pool_size=" << POOL_SIZE_.transpose() 
                  << ", center_idx=" << CENTER_IDX_.transpose() << std::endl;
        return false;
    }
    return true;
}

#endif

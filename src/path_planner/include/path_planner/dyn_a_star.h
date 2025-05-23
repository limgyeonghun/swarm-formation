#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <Eigen/Eigen>
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
    Eigen::Vector3i index;
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
                                                 Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

    inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
    inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

    inline bool checkOccupancy(const Eigen::Vector3d &pos) { return (bool)grid_map_->getInflateOccupancy(pos); }
    inline bool checkOccupancy_esdf(const Eigen::Vector3d &pos) {
        double dist;
    grid_map_->evaluateEDT(pos, dist);
    // std::cout << "Checking pos: " << pos.transpose() << ", dist: " << dist << std::endl; // debug
    return dist < 0.2; // 0.2m dist
    }

    std::vector<GridNodePtr> retrievePath(GridNodePtr current);

    double step_size_, inv_step_size_;
    Eigen::Vector3d center_;
    Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
    const double tie_breaker_ = 1.0 + 1.0 / 10000;

    std::vector<GridNodePtr> gridPath_;

    GridNodePtr ***GridNodeMap_;
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

    int rounds_{0};

public:
    typedef std::shared_ptr<AStar> Ptr;

    AStar(){};
    ~AStar();

    void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

    bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_esdf_check);

    std::vector<Eigen::Vector3d> getPath();

    std::vector<Eigen::Vector3d> astarSearchAndGetSimplePath(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
    return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
}

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
    idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;
    if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) ||
        idx(1) < 0 || idx(1) >= POOL_SIZE_(1) ||
        idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
    {
        std::cerr << "Ran out of pool, index=" << idx(0) << " " << idx(1) << " " << idx(2) << std::endl;
        return false;
    }
    return true;
}

#endif

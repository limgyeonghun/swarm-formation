#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>
#include "path_planner/grid_map.h"
#include "../../common/log_manager.hpp"
#include <queue>

constexpr double inf = 1e20; 

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

    int rounds{0}; // Distinguish every call
    enum enum_state state
    {
        UNDEFINED
    };
    Eigen::Vector3i index;

    double gScore{inf}, fScore{inf};
    GridNodePtr cameFrom{NULL};
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
    swarm_formation::LogManager::Ptr log_manager_;

    inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

    double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
    double getDiagHeu2D(GridNodePtr node1, GridNodePtr node2);
    double getManhHeu(GridNodePtr node1, GridNodePtr node2);
    double getManhHeu2D(GridNodePtr node1, GridNodePtr node2);
    double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
    double getEuclHeu2D(GridNodePtr node1, GridNodePtr node2);
    inline double getHeu(GridNodePtr node1, GridNodePtr node2);
    inline double getHeu2D(GridNodePtr node1, GridNodePtr node2);

    bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

    inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
    inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

    //bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );
    
    inline bool checkOccupancy(const Eigen::Vector3d &pos) { 
        int occ = grid_map_->getInflateOccupancy(pos);
        return (occ > 0);  // -1(out of map) is false, 1(obstacle) is true, 0(free space) is false
    }
    inline bool checkOccupancy2D(const Eigen::Vector3d &pos) { 
        int occ = grid_map_->getInflateOccupancy2D(pos);
        return (occ > 0);
    }
    inline bool checkOccupancy_esdf(const Eigen::Vector3d &pos){
        const double dist = 0.2;
        if (grid_map_->getDistance(pos) < dist ) 
            return true;
        else
            return false;
    }
    inline bool checkOccupancy_esdf2D(const Eigen::Vector3d &pos){
        const double dist = 0.15;
        if (grid_map_->getDistance(pos) < dist ) 
            return true;
        else
            return false;
    }
    
    // inline bool checkOccupancy(const Eigen::Vector3d &pos) { 
    //     const double dist = 0.2;
    //     if (grid_map_->getDistance(pos) < dist ) 
    //         return true;
    //     else
    //         return false;
    // }

    std::vector<GridNodePtr> retrievePath(GridNodePtr current);

    double step_size_, inv_step_size_;
    Eigen::Vector3d center_;
    Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
    const double tie_breaker_ = 1.0 + 1.0 / 10000;
    const int max_iterations_ = 50000;

    std::vector<GridNodePtr> gridPath_;

    GridNodePtr ***GridNodeMap_;
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;
    int rounds_{0};

public:
    typedef std::shared_ptr<AStar> Ptr;

    AStar(){};
    ~AStar();
    
    void setLogManager(swarm_formation::LogManager::Ptr log_manager);

    void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

    bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_esdf_check);

    bool AstarSearch2D(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_esdf_check);

    std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> astarSearchAndGetSimplePath(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, int drone_id);
    std::vector<Eigen::Vector3d> astarSearch2DAndGetSimplePath(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, int drone_id);
    
    Eigen::Vector3d getOrigin() const { return grid_map_->getOrigin(); }
    Eigen::Vector3d getMapSize() const { return grid_map_->getMapSize(); }
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    return tie_breaker_ * getDiagHeu(node1, node2);
}

inline double AStar::getHeu2D(GridNodePtr node1, GridNodePtr node2)
{
    return tie_breaker_ * getDiagHeu2D(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
    return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
    idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

    if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
    {
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
        RCLCPP_ERROR(rclcpp::get_logger("astar"), "Ran out of pool, pt=(%f,%f,%f)", pt(0), pt(1), pt(2));
        return false;
    }

    return true;
};

#endif

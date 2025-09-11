#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <thread>
#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <path_planner/dyn_a_star.h>
#include <path_planner/grid_map.h>
#include <swarm_graph/swarm_graph.hpp>
#include "lbfgs.hpp"
#include "plan_container.hpp"
#include "poly_traj_utils.hpp"
#include "munkres_algorithm.hpp"

namespace ego_planner
{
  enum FORMATION_TYPE
  {
    NONE_FORMATION = 0,
    REGULAR_HEXAGON = 1,
    REGULAR_SQUARE = 2,
    TEST_FORMATION = 3
  };

  class ConstrainPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;

    void resize_cp(const int size_set)
    {
      cp_size = size_set;
      points.resize(3, size_set);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class PolyTrajOptimizer
  {
  private:
    double dbg_cost_formation_{0.0};
    GridMap::Ptr grid_map_;
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{nullptr};
    ConstrainPoints cps_;
    SwarmGraph::Ptr swarm_graph_;
    std::ofstream log_file_;
    std::string log_file_path_;

    int drone_id_;
    int cps_num_prePiece_;
    int variable_num_;
    int piece_num_;
    int iter_num_;
    double min_ellip_dist2_;

    std::string result_fn_;
    std::fstream result_file_;

    double collision_check_time_end_ = 0.0;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;


    double wei_obs_;
    double wei_swarm_;
    double wei_feas_;
    double wei_sqrvar_;
    double wei_time_;
    double wei_formation_;

    double obs_clearance_;
    double swarm_clearance_;
    double max_vel_, max_acc_;

    int formation_size_ = 4;  // Default to 4 drones
    bool use_formation_ = true;
    bool is_other_assigning_ = false;
    uint64_t seq_ = 0;
    double debug_similarity_ = 0.0;

    double t_now_;
    bool enable_obstacles_;
    bool enable_debug_logs_;  // Debug logging control

    rclcpp::Node::SharedPtr node_;

  public:
    PolyTrajOptimizer() {}
    // ~PolyTrajOptimizer() { }
    ~PolyTrajOptimizer() { closeLogFile(); }

    void setParam(const rclcpp::Node::SharedPtr &node);
    void initLogFile(const std::string &path);
    void closeLogFile();
    void setEnvironment(const GridMap::Ptr &map);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);
    void setFormation(const std::vector<Eigen::Vector3d>& formation_positions, int formation_size);

    inline ConstrainPoints getControlPoints() { return cps_; }
    inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr(void) { return &jerkOpt_; }
    inline int get_cps_num_prePiece_() { return cps_num_prePiece_; };
    inline double getSwarmClearance(void) { return swarm_clearance_; }
    double getCollisionCheckTimeEnd() { return collision_check_time_end_; }

    bool OptimizeTrajectory_lbfgs(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                  const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                  Eigen::MatrixXd &optimal_points, const bool use_formation);

    void astarWithMinTraj(const Eigen::MatrixXd &iniState,
                          const Eigen::MatrixXd &finState,
                          std::vector<Eigen::Vector3d> &simple_path,
                          Eigen::MatrixXd &ctl_points,
                          poly_traj::MinJerkOpt &frontendMJ);

    void showFormationInformation(bool is_show, Eigen::Vector3d pos);
    void setDesiredFormation(int type);
    bool getFormationPos(std::vector<Eigen::Vector3d> &swarm_graph_pos, Eigen::Vector3d pos);

  private:
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);
    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);

    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    bool swarmGraphGradCostP(const int i_dp,
                             const double t,
                             const Eigen::Vector3d &p,
                             const Eigen::Vector3d &v,
                             Eigen::Vector3d &gradp,
                             double &gradt,
                             double &grad_prev_t,
                             double &costp);

    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    bool checkCollision(void);

  public:
    typedef std::unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace ego_planner
#endif

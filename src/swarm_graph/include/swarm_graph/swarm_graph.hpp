#ifndef _SWARM_GRAPH_H_
#define _SWARM_GRAPH_H_

#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp> // ROS2 header

class SwarmGraph
{

private:
    std::vector<Eigen::Vector3d> nodes;
    std::vector<Eigen::Vector3d> nodes_des;
    std::vector<Eigen::Vector3d> nodes_des_init;

    std::vector<Eigen::Vector3d> agent_grad;

    bool have_desired;

    Eigen::MatrixXd A;
    Eigen::VectorXd D;
    Eigen::MatrixXd Lhat;

    Eigen::MatrixXd A_des;
    Eigen::VectorXd D_des;
    Eigen::MatrixXd Lhat_des;

    Eigen::MatrixXd DLhat;

public:
    SwarmGraph();
    ~SwarmGraph() {}

    bool updateGraph(const std::vector<Eigen::Vector3d> &swarm);

    bool setDesiredForm(const std::vector<Eigen::Vector3d> &swarm_des);

    bool calcMatrices(const std::vector<Eigen::Vector3d> &swarm,
                      Eigen::MatrixXd &Adj, Eigen::VectorXd &Deg,
                      Eigen::MatrixXd &SNL);

    double calcDist2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);

    bool calcFNorm2(double &cost);

    bool calcFGrad(Eigen::Vector3d &gradp, int idx);

    Eigen::Vector3d getGrad(int id);
    bool getGrad(std::vector<Eigen::Vector3d> &swarm_grad);

    std::vector<Eigen::Vector3d> getDesNodesInit() { return nodes_des_init; }
    std::vector<Eigen::Vector3d> getDesNodesCur() { return nodes_des; }

    typedef std::unique_ptr<SwarmGraph> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif

#ifndef _SWARM_GRAPH_H_
#define _SWARM_GRAPH_H_

#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>  // ROS2 header

class SwarmGraph {
    
private:
    std::vector<Eigen::Vector3d> nodes;         // swarm_graph의 정점 위치
    std::vector<Eigen::Vector3d> nodes_des;       // 할당 후 변경될 수 있는 desired graph
    std::vector<Eigen::Vector3d> nodes_des_init;  // 최초 설정된 desired graph (한 번 정해지면 변경 없음)

    std::vector<Eigen::Vector3d> agent_grad;      // swarm_graph의 gradient

    bool have_desired;

    Eigen::MatrixXd A;      // 인접 행렬
    Eigen::VectorXd D;      // Degree 행렬
    Eigen::MatrixXd Lhat;   // 대칭 정규화 Laplacian

    Eigen::MatrixXd A_des;  // Desired 인접 행렬 
    Eigen::VectorXd D_des;  // Desired Degree 행렬
    Eigen::MatrixXd Lhat_des;  // Desired SNL 
    
    Eigen::MatrixXd DLhat;  // SNL의 차이

public:
    SwarmGraph();
    ~SwarmGraph(){}

    // 노드, feature matrix 및 desired matrix 업데이트
    bool updateGraph(const std::vector<Eigen::Vector3d> &swarm); 

    // 원하는 swarm 노드 설정
    bool setDesiredForm(const std::vector<Eigen::Vector3d> &swarm_des);

    // 그래프 feature matrix 계산
    bool calcMatrices(const std::vector<Eigen::Vector3d> &swarm,
                      Eigen::MatrixXd &Adj, Eigen::VectorXd &Deg,
                      Eigen::MatrixXd &SNL);

    // 두 벡터 사이의 유클리드 거리의 제곱 계산
    double calcDist2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);                   

    // SNL 행렬의 F-노름 제곱 차이 계산
    bool calcFNorm2(double &cost); 

    // 위치에 대한 gradient 계산
    bool calcFGrad(Eigen::Vector3d &gradp, int idx);

    // idx번째 gradient 반환
    Eigen::Vector3d getGrad(int id);
    bool getGrad(std::vector<Eigen::Vector3d> &swarm_grad);

    // 헬퍼 함수들
    std::vector<Eigen::Vector3d> getDesNodesInit() { return nodes_des_init; }
    std::vector<Eigen::Vector3d> getDesNodesCur() { return nodes_des; }

    typedef std::unique_ptr<SwarmGraph> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif

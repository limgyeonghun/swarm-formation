#include <swarm_graph/swarm_graph.hpp>

SwarmGraph::SwarmGraph() {
    have_desired = false;
}

bool SwarmGraph::updateGraph(const std::vector<Eigen::Vector3d> &swarm) { 

    nodes = swarm;
    
    if (nodes.size() != nodes_des.size()) {
        std::cout << "swarm size : " << nodes.size() << std::endl;
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Size of swarm formation vector is incorrect.");
        return false;
    }

    calcMatrices(nodes, A, D, Lhat);

    if (have_desired) {
        DLhat = Lhat - Lhat_des;

        Eigen::Vector3d gradp;
        agent_grad.clear();
        for (int idx = 0; idx < nodes.size(); idx++) {
            calcFGrad(gradp, idx);
            agent_grad.push_back(gradp);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Please enter the desired formation!!");
    }
    
    return true;
}

bool SwarmGraph::setDesiredForm(const std::vector<Eigen::Vector3d> &swarm_des) {  

    if (!have_desired)
        nodes_des_init = swarm_des;

    nodes_des = swarm_des;
    calcMatrices(swarm_des, A_des, D_des, Lhat_des);
    have_desired = true;
    return have_desired;
}

bool SwarmGraph::calcMatrices(const std::vector<Eigen::Vector3d> &swarm,
                              Eigen::MatrixXd &Adj, Eigen::VectorXd &Deg,
                              Eigen::MatrixXd &SNL) {   

    Adj = Eigen::MatrixXd::Zero(swarm.size(), swarm.size());
    Deg = Eigen::VectorXd::Zero(swarm.size());
    SNL = Eigen::MatrixXd::Zero(swarm.size(), swarm.size());


    for (int i = 0; i < swarm.size(); i++) {
        for (int j = 0; j < swarm.size(); j++) {
            Adj(i, j) = calcDist2(swarm[i], swarm[j]);
            Deg(i) += Adj(i, j);
        }
    }

    for (int i = 0; i < swarm.size(); i++) {
        for (int j = 0; j < swarm.size(); j++) {
            if (i == j) {
                SNL(i, j) = 1;
            } else {
                SNL(i, j) = -Adj(i, j) * std::pow(Deg(i), -0.5) * std::pow(Deg(j), -0.5);
            }
        }
    }
    
    return true;
}

double SwarmGraph::calcDist2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
    return (v1 - v2).cwiseAbs2().sum();
}

bool SwarmGraph::calcFNorm2(double &cost) {

    if (have_desired) {
        cost = DLhat.cwiseAbs2().sum();
        return true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Invalid desired formation.");
        return false;
    }
}

bool SwarmGraph::calcFGrad(Eigen::Vector3d &gradp, int idx) {

    if (have_desired) {
        int iter = 0;
        int N = nodes.size();

        Eigen::VectorXd dfde = Eigen::VectorXd::Zero(N - 1);
        Eigen::MatrixXd dedp(N - 1, 3);

        double b0 = 0;
        for (int k = 0; k < N; k++) {
            b0 += A(idx, k) * DLhat(idx, k) / std::sqrt(D(k));
        }
        b0 = 2 * std::pow(D(idx), -1.5) * b0;

        for (int i = 0; i < N; i++) {
            if (i != idx) {
                for (int k = 0; k < N; k++) {
                    dfde(iter) += A(i, k) * DLhat(i, k) / std::sqrt(D(k));
                }
                dfde(iter) = 2 * std::pow(D(i), -1.5) * dfde(iter) + b0 + 4 * (-1 / (std::sqrt(D(i)) * std::sqrt(D(idx)))) * DLhat(i, idx);
                iter++;
            }
        }

        iter = 0;
        for (int i = 0; i < N; i++) {
            if (i != idx) {
                for (int k = 0; k < 3; k++) {
                    dedp(iter, k) = nodes[idx](k) - nodes[i](k);
                }
                iter++;
            }
        }

        if ((dfde.transpose() * dedp).norm() > 1e-7) {
            gradp = (dfde.transpose() * dedp).normalized();
        } else {
            gradp = Eigen::Vector3d::Zero();
        }
        return true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "Invalid desired formation.");
        return false;
    }
}

Eigen::Vector3d SwarmGraph::getGrad(int id) {
    if (have_desired && id < nodes_des.size()) {
        return agent_grad[id];
    } else {
        Eigen::Vector3d grad = Eigen::Vector3d::Zero();
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "id is error !!! or desired graph has not been setup !!!");
        return grad;
    }
}

bool SwarmGraph::getGrad(std::vector<Eigen::Vector3d> &swarm_grad) {
    if (have_desired) {
        swarm_grad = agent_grad;
        return true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("SwarmGraph"), "desired graph has not been setup !!!");
        return false;
    }
}

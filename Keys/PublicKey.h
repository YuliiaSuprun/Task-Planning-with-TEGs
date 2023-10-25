#pragma once

#include <Eigen/Dense>

class PublicKey {
public:
    virtual Eigen::VectorXd project(const Eigen::VectorXd& state, double p) const = 0;
    virtual Eigen::VectorXd reconstruct(const Eigen::VectorXd& abstract_state, double p) const = 0;
    virtual Eigen::VectorXd get_private_key(const Eigen::VectorXd& state) const = 0;
    virtual Eigen::VectorXd get_private_key(const Eigen::VectorXd& states, const Eigen::VectorXd& abstract_states = Eigen::VectorXd()) const = 0;
};

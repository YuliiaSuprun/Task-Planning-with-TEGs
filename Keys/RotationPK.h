#pragma once

#include "PublicKey.h"
#include <Eigen/Dense>
#include <vector>

class RotationPK : public PublicKey {
public:
    Eigen::VectorXd project(const Eigen::VectorXd& state, double p) const override;
    Eigen::VectorXd reconstruct(const Eigen::VectorXd& abstract_state, double p) const override;
    Eigen::VectorXd get_private_key(const Eigen::VectorXd& states, const Eigen::VectorXd& abstract_states = Eigen::VectorXd()) const override;
    bool is_valid_private_key(double p, const Eigen::VectorXd& states, const Eigen::VectorXd& abstract_states = Eigen::VectorXd()) const;
    
private:
    Eigen::MatrixXd get_rotation_matrix(double p) const;
    double find_rotation_angle(const Eigen::VectorXd& state1, const Eigen::VectorXd& state2) const;
};

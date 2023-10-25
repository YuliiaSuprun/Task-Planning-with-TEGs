#include "RotationPK.h"
#include <vector>

Eigen::VectorXd RotationPK::project(const Eigen::VectorXd& state, double p) const {
    Eigen::MatrixXd rotation_matrix = get_rotation_matrix(-p);
    return rotation_matrix * state;
}

Eigen::VectorXd RotationPK::reconstruct(const Eigen::VectorXd& abstract_state, double p) const {
    Eigen::MatrixXd rotation_matrix = get_rotation_matrix(p);
    return rotation_matrix * abstract_state;
}

Eigen::VectorXd RotationPK::get_private_key(const Eigen::VectorXd& states, const Eigen::VectorXd& abstract_states) const {
    if (abstract_states.size() != 0) {
        double p = find_rotation_angle(states, abstract_states);
        for (int i = 1; i < states.size(); ++i) {
            if (p != find_rotation_angle(states[i], abstract_states[i])) {
                // Returning an empty vector to indicate None
                return Eigen::VectorXd();
            }
        }
    } else {
        // Returning a vector of values [-1, 0, 1, 2]
        Eigen::VectorXd result(4);
        result << -1, 0, 1, 2;
        return result;
    }
}

bool RotationPK::is_valid_private_key(double p, const Eigen::VectorXd& states, const Eigen::VectorXd& abstract_states) const {
    if (abstract_states.size() != 0) {
        for (int i = 0; i < states.size(); ++i) {
            if (p != find_rotation_angle(states[i], abstract_states[i])) {
                return false;
            }
        }
        return true;
    } else {
        return (p == -1) || (p == 0) || (p == 1) || (p == 2);
    }
}

Eigen::MatrixXd RotationPK::get_rotation_matrix(double p) const {
    Eigen::MatrixXd mat(2, 2);
    if(p == -1) {
        mat << 0, 1, -1, 0;
    } else if(p == 0) {
        mat << 1, 0, 0, 1;
    } else if(p == 1) {
        mat << 0, -1, 1, 0;
    } else if(p == 2 || p == -2) {
        mat << -1, 0, 0, -1;
    } else {
        // Equivalent to returning None in Python
        mat = Eigen::MatrixXd();  // Setting matrix to be empty
    }
    return mat;
}

double RotationPK::find_rotation_angle(const Eigen::VectorXd& state1, const Eigen::VectorXd& state2) const {
    for (double p = -2; p <= 2; ++p) {
        if (get_rotation_matrix(p) * state1 == state2) {
            return p;
        }
    }
    return std::nan("");  // Equivalent to returning None in Python
}

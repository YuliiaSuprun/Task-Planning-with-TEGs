#include "PublicKey.h"
#include <Eigen/Dense>

class RotationPK {
public:
    static Eigen::MatrixXd get_rotation_matrix(double theta) {
        Eigen::MatrixXd rotation_matrix(2, 2);
        rotation_matrix << cos(theta), -sin(theta),
                           sin(theta),  cos(theta);
        return rotation_matrix;
    }
};

bool is_integer_num(double num) {
    return std::floor(num) == num;
}

bool is_natural_num(double num) {
    return is_integer_num(num) && num > 0;
}

class ComboPK : public PublicKey {
public:
    Eigen::VectorXd project(const Eigen::VectorXd& state, double p) const override {
        double theta = p[0], alpha = p[1], beta = p[2], x = p[3], y = p[4];

        Eigen::MatrixXd rotation_matrix = RotationPK::get_rotation_matrix(-theta);
        Eigen::MatrixXd scaling_matrix = Eigen::MatrixXd::Identity(2, 2);
        scaling_matrix(0, 0) = 1/beta;
        scaling_matrix(1, 1) = 1/alpha;

        Eigen::VectorXd translation_vec(2);
        translation_vec << y, x;

        Eigen::VectorXd result = state - translation_vec;
        result = scaling_matrix * result;
        result = rotation_matrix * result;

        // Ensure the result vector has integer values
        for (int i = 0; i < result.size(); ++i) {
            result[i] = static_cast<int>(result[i]);
        }

        return result;
    }

    Eigen::VectorXd reconstruct(const Eigen::VectorXd& abstract_state, double p) const override {
        double theta = p[0], alpha = p[1], beta = p[2], x = p[3], y = p[4];

        Eigen::MatrixXd rotation_matrix = RotationPK::get_rotation_matrix(theta);
        Eigen::MatrixXd scaling_matrix = Eigen::MatrixXd::Identity(2, 2);
        scaling_matrix(0, 0) = beta;
        scaling_matrix(1, 1) = alpha;

        Eigen::VectorXd translation_vec(2);
        translation_vec << y, x;

        Eigen::VectorXd result = rotation_matrix * abstract_state;
        result = scaling_matrix * result;
        result = result + translation_vec;

        // Ensure the result vector has integer values
        for (int i = 0; i < result.size(); ++i) {
            result[i] = static_cast<int>(result[i]);
        }

        return result;
    }

    // ... Continue with the rest of the methods ...
};

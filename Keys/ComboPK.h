#include <vector>
#include "PublicKey.h"
#include "RotationPK.h"
#include <Eigen/Dense>
class ComboPK : public PublicKey {
public:
    Eigen::VectorXd project(const Eigen::VectorXd& state, double p) const override;
    Eigen::VectorXd reconstruct(const Eigen::VectorXd& abstract_state, double p) const override;
    Eigen::VectorXd get_private_key(const Eigen::VectorXd& states, const Eigen::VectorXd& abstract_states = Eigen::VectorXd()) const override;
    bool is_valid_private_key(double p, const Eigen::VectorXd& states, const Eigen::VectorXd& abstract_states = Eigen::VectorXd()) const;
};

#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include <boost/format.hpp>
#include <iostream>

static Eigen::IOFormat HeavyFmt;

class Tools {
public:

    /**
     * A helper method to calculate RMSE.
     */
    static Eigen::Vector4d CalculateRMSE(const std::vector<Eigen::Vector4d> &estimations,
                                         const std::vector<Eigen::Vector4d> &ground_truth);


    /**
     * A helper method to calculate Jacobians.
     */
    static Eigen::Matrix<double, 3, 4> CalculateJacobian(const Eigen::VectorXd& x_state);

    static Eigen::Vector3d ConvertCartesianToPolar(const Eigen::Vector4d &state) {
        const double p_norm = state.topRows<2>().norm();
        return {
            p_norm,
            std::atan2(state[1], state[0]),
            static_cast<double>(state.topRows<2>().transpose() * state.bottomRows<2>()) / p_norm
        };
    }

    static Eigen::Vector4d ConvertPolarToCartesian(const Eigen::Vector3d &measurement) {
        const double rho = measurement[0];
        const double phi = measurement[1];
        const double rho_dot = measurement[2];
        return {
            std::cos(phi) * rho,
            std::sin(phi) * rho,
            std::cos(phi) * rho_dot,
            std::sin(phi) * rho_dot
        };
    }
};


#endif  // TOOLS_H_

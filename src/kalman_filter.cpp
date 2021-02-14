#include "kalman_filter.h"
#include "tools.h"
#include "logger.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

void KalmanFilter::Predict(double dt) {

    // Update F
    F_(0, 2) = F_(1, 3) = dt;
    // Using linear motion model for all measurements

    // Update Q
    static constexpr double noise_ax = 9.0,
                            noise_ay = 9.0;

    const double dt4 = std::pow(dt, 4) / 4.0,
                 dt3 = std::pow(dt, 3) / 2.0,
                 dt2 = std::pow(dt, 2);

    Q_ <<
        dt4 * noise_ax , 0.0            , dt3 * noise_ax , 0.0             ,
        0.0            , dt4 * noise_ay , 0.0            , dt3 * noise_ay  ,
        dt3 * noise_ax , 0.0            , dt2 * noise_ax , 0.0             ,
        0.0            , dt3 * noise_ay , 0.0            , dt2 * noise_ay  ;

    // Predict next state and covariance
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
    BOOST_LOG_TRIVIAL(debug) << "Predict: x=" << x_.transpose().format(HeavyFmt);
//    BOOST_LOG_TRIVIAL(debug) << "   P=\n" << P_.format(HeavyFmt);
}

void KalmanFilter::Update(const Eigen::Vector2d &z) {
    const Eigen::Vector2d y = z - H_ * x_;
    const Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
    const Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = P_ - K * H_ * P_;
}

void KalmanFilter::UpdateEKF(const Eigen::Vector3d &z) {
    Eigen::Matrix<double, 3, 4> Hj = Tools::CalculateJacobian(x_);
    Eigen::Vector3d z_pred = Tools::ConvertCartesianToPolar(x_);

    // NOTE: Handle a case where the measured angle changes from M_PI to -M_PI.
    // FIXME: Is there better way to flip the angle?
    if (z_pred[1] < -M_PI * 0.9 and z[1] > M_PI * 0.9) {
        z_pred[1] = 2 * M_PI - std::abs(z_pred[1]);
    }

    Eigen::Vector3d y = z - z_pred;
    Eigen::Matrix3d S = Hj * P_ * Hj.transpose() + R_;
    Eigen::Matrix<double, 4, 3> K = P_ * Hj.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = P_ - K * Hj * P_;
}

#include "FusionEKF.h"
#include <chrono>
#include "Eigen/Dense"
#include "tools.h"
#include "logger.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // Initialize H_laser
    H_laser_ = Eigen::Matrix<double, 2, 4>::Zero();
    H_laser_(0, 0) = 1.0;
    H_laser_(1, 1) = 1.0;

    //measurement covariance matrix - laser
    R_laser_ = Eigen::Matrix2d();
    R_laser_ <<
        0.0225, 0,
        0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ = Eigen::Matrix3d();
    R_radar_ <<
        0.09 ,      0 , 0,
        0    , 0.0009 , 0,
        0    ,      0 , 0.09;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}


void FusionEKF::EnsureInitialization(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_) {
        BOOST_LOG_TRIVIAL(info) << "EKF initialization";
        ekf_.F_ = Eigen::Matrix4d::Identity();
        ekf_.Q_ = Eigen::Matrix4d::Zero();
        ekf_.x_ = Eigen::Vector4d::Ones();
        ekf_.P_ = Eigen::Matrix4d::Identity();
        ekf_.P_(1, 2) = 1000.0;
        ekf_.P_(2, 3) = 1000.0;

        switch (measurement_pack.sensor_type_) {
            case MeasurementPackage::RADAR:
                ekf_.x_ = Tools::ConvertPolarToCartesian(measurement_pack.raw_measurements_);
                break;
            case MeasurementPackage::LASER:
                ekf_.x_[0] = measurement_pack.raw_measurements_[0];
                ekf_.x_[1] = measurement_pack.raw_measurements_[1];
                break;
            default:
                throw std::runtime_error("Not supported sensor type");
        }
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
    }
}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    BOOST_LOG_TRIVIAL(debug) << (boost::format("Received sensor update, sensor type= %s, sensor measurement=%s")
                                 % measurement_pack.sensor_type_ % measurement_pack.raw_measurements_.transpose()
                                     .format(HeavyFmt)).str();
    /**
     * Initialization
     */
    EnsureInitialization(measurement_pack);

    const double delta_t = static_cast<double>(measurement_pack.timestamp_ - previous_timestamp_) / 1e6;

    /**
     * Prediction
     */

    ekf_.Predict(delta_t);

    /**
     * Update
     */

    switch (measurement_pack.sensor_type_) {
        case MeasurementPackage::RADAR:
            ekf_.R_ = R_radar_;
            ekf_.UpdateEKF(measurement_pack.raw_measurements_.topRows<3>());
            break;
        case MeasurementPackage::LASER:
            ekf_.R_ = R_laser_;
            ekf_.H_ = H_laser_;
            ekf_.Update(measurement_pack.raw_measurements_.topRows<2>());
            break;
        default: break;
    }

    // Update timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    BOOST_LOG_TRIVIAL(info) << "x=\n" << ekf_.x_.transpose().format(HeavyFmt);
    BOOST_LOG_TRIVIAL(info) << "P=\n" << ekf_.P_.format(HeavyFmt);
}

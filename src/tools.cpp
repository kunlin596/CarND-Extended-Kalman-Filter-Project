#include "tools.h"
#include "logger.h"
#include <iostream>

#include <boost/assert.hpp>
#include <boost/format.hpp>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

static const Eigen::IOFormat HeavyFormat = Eigen::IOFormat(Eigen::StreamPrecision, 1, ", ", ";\n", "[", "]", "[", "]");

static double Cross2d(const Eigen::Vector2d &vec1, const Eigen::Vector2d &vec2) {
    return vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

Eigen::Vector4d Tools::CalculateRMSE(const vector<Eigen::Vector4d> &estimations,
                                     const vector<Eigen::Vector4d> &ground_truth) {
    BOOST_ASSERT(estimations.size() == ground_truth.size());
    Eigen::Vector4d rmse;

    const size_t data_size = estimations.size();
    if (data_size < 1) {
        BOOST_LOG_TRIVIAL(error) << (boost::format("Got invalid size, estimations.size()=%zu, ground_truth.size()=%zu.")
                                     % estimations.size() % ground_truth.size()).str();
        return rmse;
    }

    Eigen::Map<const Eigen::Array4Xd> estimations_map(reinterpret_cast<const double*>(estimations.data()),
                                                      4, data_size);
    Eigen::Map<const Eigen::Array4Xd> ground_truth_map(reinterpret_cast<const double*>(ground_truth.data()),
                                                       4, data_size);
    rmse = ((estimations_map - ground_truth_map).square().rowwise().sum() / data_size).sqrt();
    BOOST_LOG_TRIVIAL(info) << (boost::format("rmse=%s") % rmse.transpose().format(HeavyFormat)).str();
    return rmse;
}

Eigen::Matrix<double, 3, 4> Tools::CalculateJacobian(const VectorXd& x_state) {
   auto J = Eigen::Matrix<double, 3, 4>();
   const double &px = x_state[0];
   const double &py = x_state[1];
   const double &vx = x_state[2];
   const double &vy = x_state[3];
   const double p_norm = x_state.topRows<2>().norm();
   const double p_norm2 = p_norm * p_norm;

   if (std::abs(p_norm) < 1e-6) {
       return J;
   }

   J <<
       // row1
       px / p_norm, py / p_norm, 0.0, 0.0,
       // row2
       -py / p_norm2, px / p_norm2, 0.0, 0.0,
       // row3
       py * (Cross2d(x_state.bottomRows<2>(), x_state.topRows<2>())) / (p_norm * p_norm2),
       px * (Cross2d(x_state.topRows<2>(), x_state.bottomRows<2>())) / (p_norm * p_norm2),
       px / p_norm, py / p_norm;

    return J;
}

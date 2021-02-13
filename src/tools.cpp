#include "tools.h"
#include <iostream>

#include <boost/assert.hpp>
#include <boost/format.hpp>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

static Eigen::IOFormat HeavyFormat = Eigen::IOFormat(Eigen::FullPrecision, 1, ", ", ";\n", "[", "]", "[", "]");

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    BOOST_ASSERT(estimations.size() == ground_truth.size());
    VectorXd rmse;

    const size_t data_size = estimations.size();
    if (data_size < 1) {
        printf("Got invalid size, estimations.size()=%zu, ground_truth.size()=%zu.",
               estimations.size(), ground_truth.size());
        return rmse;
    }

    const size_t state_size = estimations[0].size();
    if (state_size < 1) {
        printf("Got invalid size, state_size=%zu", state_size);
        return rmse;
    }
    Eigen::Map<const Eigen::ArrayXXd> estimations_map(reinterpret_cast<const double*>(estimations.data()), data_size, state_size);
    Eigen::Map<const Eigen::ArrayXXd> ground_truth_map(reinterpret_cast<const double*>(ground_truth.data()), data_size, state_size);
    rmse = ((estimations_map - ground_truth_map).matrix().colwise().squaredNorm().array() / static_cast<const float>(state_size)).sqrt();
    BOOST_ASSERT(rmse.rows() == state_size);
    LOG("rmse=%s", rmse.transpose().format(HeavyFormat));
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   return MatrixXd();
}

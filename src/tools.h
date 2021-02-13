#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

static Eigen::IOFormat HeavyFmt;

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LOG(fmt, ...) std::cout << (boost::format(std::string("%s:%s - ") + fmt) % __FILENAME__ % __LINE__ % __VA_ARGS__).str() << std::endl;

class Tools {
public:
    /**
     * Constructor.
     */
    Tools();

    /**
     * Destructor.
     */
    virtual ~Tools();

    /**
     * A helper method to calculate RMSE.
     */
    Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                  const std::vector<Eigen::VectorXd> &ground_truth);

    /**
     * A helper method to calculate Jacobians.
     */
    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);


};


#endif  // TOOLS_H_

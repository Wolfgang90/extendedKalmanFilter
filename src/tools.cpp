#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
// Create RMSE Calculation
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
// Create Jacobian calculation
}

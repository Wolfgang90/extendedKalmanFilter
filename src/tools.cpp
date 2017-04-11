#include <iostream>
#include <cmath>
#include "tools.h" 

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculatePolarMappingHx(const VectorXd &x_state) {
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  VectorXd h_x(3);
  
  // define precalculated temporary values
  double tmp_1 = sqrt(px*px + py*py); 

  // prevent division by zero
  if(tmp_1 == 0) {
    cout << "Avoided division by 0 while mapping polar to cartesian coordinates" << endl;
    tmp_1 = 0.0000000001;
  }
  
  //Create mapping vector
  h_x << tmp_1,
         atan2(py, px),
         (px*py + py*vy)/tmp_1;

  return h_x;
}

VectorXd Tools::CalculateCartesianMappings(const VectorXd& x_meas) {
  double rho = x_meas(0);
  double phi = x_meas(1);
  double rho_dot = x_meas(2);

  float px = sqrt(rho*rho) / (1+ tan(phi) * tan(phi));
  float py = tan(phi) * px;
  float vx = rho_dot * cos(phi);
  float vy = rho_dot * sin(phi);

  VectorXd cartesian(4);
  cartesian << px, py, vx, vy;
  
  return cartesian;
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    cout << "Stopped during RMSE calculation; invalid estimation or ground_truth data" << endl;
    return rmse;
  }
  
  // accumulate square residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    
    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the mean
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c2*c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2),              (py/c2),               0,     0,
        -(py/c1),             (px/c1),               0,     0,
        py*(vx*py - vy*px)/c3, px*(px*vy -py*vx)/c3, px/c2, py/c2;
  
  return Hj;
}

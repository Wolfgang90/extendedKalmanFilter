#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict(){
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  cout << "z_pred: " << z_pred << endl;
  cout << "z" << z << endl;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  cout << "Ht" << Ht << endl;
  cout << "R_ Dim: " << R_.rows() << " x "  << R_.cols() << endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  cout << "S" << S << endl;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  cout << "PHt" << PHt << endl;
  MatrixXd K = PHt * Si;
  cout << "K" << K << endl;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Define h-function to be used instead of H-matrix
  //Update state by using Kalman Filter equations
  //calculate the new Jacobian Hj  
  // use non-linear measurement function to project
  
  // Define h-function to be used instead of H-matrix
  VectorXd hx = tools.CalculatePolarMappingHx(x_);
  MatrixXd Hj = tools.CalculateJacobian(x_);
  
  VectorXd y = z - hx;
  MatrixXd Hjt = Hj.transpose();
  
  MatrixXd S = Hj * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Hjt;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}

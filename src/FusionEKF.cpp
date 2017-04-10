#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false

  previous_timestamp_ = 0;

  // Initialize measurement vector
  ekf_.x_ = VectorXd(4);
  
  // Initialize measurement covariance matrix - laser
  R_laser_ = MatrixXd(2,2);
  R_laser_ << 0.0225, 0
              0,      0.0225;
  
  // Initialize measurement covariance matrix - radar
  R_radar_ = MatrixXd(3,3);
  R_radar_ << 0.09,      0, 0
              0,    0.0009, 0
              0,         0, 0.09

  // Initialize measurement matrix 
  H_laser_ = MatrixXd(2,4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ = MatrixXd(3,4);

  // Initialize state covariance matrix
  efk_.P_ = MatrixXd(4,4);
  ekf_.P_ = 1, 0,    0, 0,
            0, 1,    0, 0,
            0, 0, 1000, 0,
            0, 0,    0, 1000,

  // Initialize transition matrix with dt = 1
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ = 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // set the acceleration noise components
  noise_ax = 9;
  noise_ax = 9;

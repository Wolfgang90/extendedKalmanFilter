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
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initialize measurement vector
  x_ = VectorXd(4);
  
  // Initialize measurement covariance matrix - laser
  R_laser_ = MatrixXd(2,2);
  R_laser_ << 0.0225, 0,
              0,      0.0225;
  
  // Initialize measurement covariance matrix - radar
  R_radar_ = MatrixXd(3,3);
  R_radar_ << 0.09,      0, 0,
              0,    0.0009, 0,
              0,         0, 0.09;

  // Initialize measurement matrix 
  H_ = MatrixXd(2,4);
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  Hj_ = MatrixXd(3,4);

  // Initialize state covariance matrix
  P_ = MatrixXd(4,4);
  P_ << 1, 0,    0, 0,
        0, 1,    0, 0,
        0, 0, 1000, 0,
        0, 0,    0, 1000;

  // Initialize transition matrix with dt = 1
  F_ = MatrixXd(4,4);
  F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // Initialize process covariance matrix
  Q_ = MatrixXd(4,4);
  
  // set the acceleration noise components
  noise_ax = 9;
  noise_ax = 9;
  }

  /**
  * Destructor.
  */
  FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &meas_package) {

  //Initialization
  if (!is_initialized_){
    // initial measurement
    cout << "EKF: " <<endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      x_ << tools.CalculateCartesianMappings(meas_package.raw_measurements_);


      ekf_.Init(x_, P_, F_, H_, R_radar_, Q_); 
             
    } else if(meas_package.sensor_type_ == MeasurementPackage::LASER){
      x_ << meas_package.raw_measurements_[0],
            meas_package.raw_measurements_[1],
            0,
            0;
            
      ekf_.Init(x_, P_, F_, H_, R_laser_, Q_);
    }

    previous_timestamp_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    
    cout << "FusionEKF initialized" << endl;
    return;
  }
  


  //Prediction
  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; //dt in sec
  previous_timestamp_ = meas_package.timestamp_;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Update process covariance matrix Q
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  ekf_.Q_ << dt_4/4*noise_ax, 0,               dt_3/2*noise_ax, 0,
             0,               dt_4/4*noise_ay, 0,               dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0,               dt_2*noise_ax,   0,
             0,               dt_3/2*noise_ay, 0,               dt_2*noise_ay;

  ekf_.Predict();

 
  
  //Update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(meas_package.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.Update(meas_package.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  
}

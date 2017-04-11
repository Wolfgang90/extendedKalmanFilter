#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filer update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous_timestamp_;
  long previous_timestamp_;
  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::VectorXd x_;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd Hj_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd Q_;
  float noise_ax;
  float noise_ay;
};

#endif /* FusionEKF_H_ */

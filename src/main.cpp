#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "Eigen/Dense"
#include "FusionEKF.h"


using namespace std;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

void check_arguments(int argc, char *argv[]){

  // Initialize reply string if input is invalid
  stringstream ss;
  ss << "Please enter the following command line arguments: " << argv[0] << " path/to/input-file.txt output-file.txt";
  string command_input = ss.str();

  // Initialize tracker for argument validity
  bool args_valid = false;

  if (argc <= 1) {
    cerr << command_input << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << command_input << endl;
  } else if (argc == 3) {
    args_valid = true;
  } else if (argc > 3) {
    cerr << "Too many arguments provided.\n" << command_input << endl;
  }

  if (!args_valid) {
    exit(EXIT_FAILURE);
  }
}


void check_files(ifstream &in_file, string &in_name,
                 ofstream &out_file, string &out_name) {
  //Validate whether input file is open
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  //Validate whether output file is open
  if (!in_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}


int main(int argc, char *argv[]){
  cout << "Program started successfully" << endl;
  
  check_arguments(argc, argv);

  // Initialize input file
  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  // Initialize output file
  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  // Analyze whether files are read- and writeable
  check_files(in_file_, in_file_name_, out_file_, out_file_name_); 

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  while (getline(in_file_, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT


      // read measurements at this timestamp 
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
        // RADAR MEASUREMENT

        // read measurements at this timestamp
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        float ro;
        float theta;
        float ro_dot;
        iss >> ro;
        iss >> theta;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro, theta, ro_dot;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
        measurement_pack_list.push_back(meas_package);
    }
    
    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }
  cout << "Data read in successfully" << endl;

  // Create a Fusion EKF instance
  FusionEKF fusionEKF;
  cout << "Fusion EKF instance initialized" << endl;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);
    
    // output the estimation
    out_file_ << fusionEKF.ekf_.x_(0) << "\t";
    out_file_ << fusionEKF.ekf_.x_(1) << "\t";
    out_file_ << fusionEKF.ekf_.x_(2) << "\t";
    out_file_ << fusionEKF.ekf_.x_(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // output the estimation
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if(measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t";
      out_file_ << ro * sin(phi) << "\t";
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\t";

    estimations.push_back(fusionEKF.ekf_.x_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }
  
  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations,ground_truth) << endl;

  // close files
  if(out_file_.is_open()) {
    out_file_.close();
  }

  if(in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}

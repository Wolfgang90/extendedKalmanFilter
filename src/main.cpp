#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "Eigen/Dense"


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

  sting line;

  while (getline(in_file_, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    } 
  

  }

  return 0;
}

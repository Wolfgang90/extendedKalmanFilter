#include <iostream>
#include <sstream>

using namespace std;

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

int main(int argc, char *argv[]){
  cout << "Program started successfully" << endl;
  
  check_arguments(argc, argv);

  //Load data from input file


  return 0;
}

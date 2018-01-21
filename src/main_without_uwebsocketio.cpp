#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main()
{
  // Create a Kalman Filter instance
  UKF ukf;

  // Used to compute the RMSE later
  Tools tools;
  
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  MeasurementPackage meas_package;
  long long timestamp;
  string sensor_type;
  float px;
  float py;
  float ro;
  float phi;
  float ro_dot;
  float x_gt;
  float y_gt;
  float vx_gt;
  float vy_gt;
  double p_x;
  double p_y;
  double v;
  double v1;
  double v2;
  double yaw;
  double yaw_rate;
  float px_meas;
  float py_meas;

  string in_file_name_ = "../obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(in_file_name_.c_str(),std::ifstream::in);
	
	if (in_file.fail()) {
		cout << "main() - File open() failed!!!" << endl;
	}
	else if (!in_file.is_open()) {
		cout << "main() - Cannot open input file: " << in_file_name_ << endl;
	}
  else {
    string line;
    int line_number = 1;
    
    ofstream out_file;
    out_file.open("estimates.txt");
    // Header
    out_file << "px\t" << "py\t" << "v\t" << "yaw\t" << "yaw_rate\t" << "vx\t" << "vy\t" << "px_meas\t" << "py_meas\t" << "x_gt\t" << "y_gt\t" << "vx_gt\t" << "vy_gt\t" << "rmse_px\t" << "rmse_py\t" << "rmse_vx\t" << "rmse_vy\t"  << "nis_lidar\t"  << "nis_radar" << endl;
    
		while (getline(in_file, line)) {
      istringstream iss(line);
      iss >> sensor_type;

      if (sensor_type.compare("L") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);
        iss >> px;
        iss >> py;
        meas_package.raw_measurements_ << px, py;
        px_meas = px;
        py_meas = py;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
      } 
      else if (sensor_type.compare("R") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        iss >> ro;
        iss >> phi;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro,phi,ro_dot;
        px_meas = ro * cos(phi);
        py_meas = ro * sin(phi);
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
      }
      
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      VectorXd gt_values(4);
      gt_values(0) = x_gt;
      gt_values(1) = y_gt; 
      gt_values(2) = vx_gt;
      gt_values(3) = vy_gt;
      ground_truth.push_back(gt_values);
        
      // Call ProcessMeasurment(meas_package) for Kalman filter
      ukf.ProcessMeasurement(meas_package);    	  

      // Push the current estimated x,y position from the Kalman filter's state vector
      VectorXd estimate(4);

      p_x = ukf.x_(0);
      p_y = ukf.x_(1);
      v  = ukf.x_(2);
      yaw = ukf.x_(3);
      yaw_rate = ukf.x_(4);
      
      v1 = cos(yaw)*v;
      v2 = sin(yaw)*v;

      estimate(0) = p_x;
      estimate(1) = p_y;
      estimate(2) = v1;
      estimate(3) = v2;
      
      estimations.push_back(estimate);

      VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

      //  px, py, vx, and vy RMSE should be less than or equal to the values:
      // [.09, .10, .40, .30]
      cout << "Result for observation: " << line_number << endl;
      cout << "estimate_px: " << p_x << endl;
      cout << "estimate_py: " << p_y << endl;
      cout << "estimate_v: " << v << endl;
      cout << "estimate_yaw: " << yaw << endl;
      cout << "estimate_yaw_rate: " << yaw_rate << endl;
      cout << "estimate_vx: " << v1 << endl;
      cout << "estimate_vy: " << v2 << endl;
      cout << "measured_px: " << px_meas << endl;
      cout << "measured_py: " << py_meas << endl;
      cout << "groundtruth_px: " << x_gt << endl;
      cout << "groundtruth_py: " << y_gt << endl;
      cout << "groundtruth_vx: " << vx_gt << endl;
      cout << "groundtruth_vy: " << vy_gt << endl;
      cout << "rmse_px: " << RMSE(0) << endl;
      cout << "rmse_py: " << RMSE(1) << endl;
      cout << "rmse_vx: " << RMSE(2) << endl;
      cout << "rmse_vy: " << RMSE(3) << endl;
      cout << "nis_lidar: " << ukf.nis_lidar_ << endl;
      cout << "nis_radar: " << ukf.nis_radar_ << endl << endl;
      
      out_file << p_x << "\t" << p_y << "\t" << v << "\t" << yaw << "\t" << yaw_rate << "\t" << v1 << "\t" << v2 << "\t" << px_meas << "\t" << py_meas << "\t" << x_gt << "\t" << y_gt << "\t" << vx_gt << "\t" << vy_gt << "\t" << RMSE(0) << "\t" << RMSE(1) << "\t" << RMSE(2) << "\t" << RMSE(3) << "\t"  << ukf.nis_lidar_ << "\t"  << ukf.nis_radar_ << endl;
     
      line_number += 1;
    }
    if(out_file.is_open()) {
      out_file.close();
    }
  }

  if(in_file.is_open()) {
		in_file.close();
	}
  
	return 0;
}
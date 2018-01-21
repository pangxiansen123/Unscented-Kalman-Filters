#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
		cout << "Tools::CalculateRMSE() - Error - Either estimation vector size is 0 OR estimation and ground truth vectors have different sizes!!!" << endl;
		return rmse;
	}

	VectorXd squared_sum(4);
	squared_sum << 0,0,0,0;
	VectorXd squared_diff(4);
	VectorXd squared(4);
  
	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {
		squared_diff = estimations[i] - ground_truth[i];
		squared = squared_diff.array() * squared_diff.array();
		squared_sum += squared;
	}

	VectorXd mean(4);
	mean = squared_sum / estimations.size();
	rmse = mean.array().sqrt();
	return rmse;
}
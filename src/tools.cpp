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

	// Check validity of inputs:
	//  -estimation vector size should not be zero
	//  -estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	// Accumulate squared residuals
	for (unsigned int i=0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i];

		// Coefficient-wise multiplication
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	// Mean + square root
	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;
}

double Tools::NormalizeAngle(double angle) {
  return angle < -M_PI
    ? M_PI + std::fmod(angle - -M_PI, M_PI - -M_PI)
    : std::fmod(angle - -M_PI, M_PI - -M_PI) + -M_PI;
}

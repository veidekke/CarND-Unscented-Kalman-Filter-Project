#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  * @param {vector<VectorXd>} &estimations The estimated values.
  * @param {vector<VectorXd>} &ground_truth The ground truth values.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to normalize an angle.
  * @param {double} angle The angle to be normalized.
  */
  double NormalizeAngle(double angle);

};

#endif /* TOOLS_H_ */

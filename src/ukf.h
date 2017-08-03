#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* Initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* If this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* If this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* State covariance matrix
  MatrixXd P_;

  ///* Predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* Predicted sigma points matrix
  MatrixXd Xsig_aug_;

  ///* Matrix for sigma points in measurement space - radar
  MatrixXd Zsig_radar_;

  ///* Matrix for sigma points in measurement space - lidar
  MatrixXd Zsig_lidar_;

  ///* Time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Measurement dimension - radar
  int n_z_radar_;

  ///* Measurement dimension - lidar
  int n_z_lidar_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * Process a single measurement.
   * @param {MeasurementPackage} measurement_pack The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage measurement_pack);

  /**
   * Update state & state covariance matrix using lidar or radar measurement.
   * @param {MeasurementPackage} measurement_pack The measurement at k+1
   * @param {VectorXd} z_pred Vector containg mean predicted measurement
   * @param {MatrixXd} S Matrix containing predicted measurement covariance
   * @param {bool} is_radar Is measurement radar (or lidar)?
   */
  void Update(MeasurementPackage measurement_pack, VectorXd z_pred, MatrixXd S, bool is_radar);

  /**
   * Augment the sigma points, write result directly into Xsig_aug_.
   */
  void AugmentedSigmaPoints();

  /**
   * Predict sigma points, write result directly into Xsig_pred_.
   * @param {double} delta_t the change in time (in seconds) between the last
   * measurement and this one.
   */
  void SigmaPointPrediction(double delta_t);

  /**
   * Predict mean and covariance.
   */
  void PredictMeanAndCovariance();

  /**
   * Predict radar or lidar measurement.
   * @param {VectorXd*} z_out Vector containing mean predicted measurement.
   * @param {MatrixXd*} S_out Measurement covariance matrix.
   * @param {bool} is_radar Is measurement radar (or lidar)?
   */
  void PredictMeasurement(VectorXd* z_out, MatrixXd* S_out, bool is_radar);
};

#endif /* UKF_H */

#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // If this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // If this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented dimension
  n_aug_ = 7;

  // Number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  // Measurement dimension - radar
  n_z_radar_ = 3;

  // Measurement dimension - lidar
  n_z_lidar_ = 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Augmented sigma points matrix
  Xsig_aug_ = MatrixXd(n_aug_, n_sig_);

  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // Matrix for sigma points in measurement space - radar
  Zsig_radar_ = MatrixXd(n_z_radar_, n_sig_);

  // Matrix for sigma points in measurement space - lidar
  Zsig_lidar_ = MatrixXd(n_z_lidar_, n_sig_);

  // Weights
  weights_ = VectorXd(n_sig_);

  // Initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0);

  // Initial covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Used for angle normalization
  Tools tools;

  // Measurement noise covariance matrix - radar
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  // Measurement noise covariance matrix - lidar
  R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);
  R_lidar_ << std_laspx_ * std_laspx_, 0 ,
              0, std_laspy_ * std_laspy_;

  // Set weights
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
}

UKF::~UKF() {}

/**
 * Process a single measurement.
 * @param {MeasurementPackage} measurement_pack The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // Use first measurement to initialize state
    // (convert from polar to cartesian coordinates in case of radar)
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // ro * cos(theta)
      x_(0) = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);

      // ro * sin(theta)
      x_(1) = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_(0) = measurement_pack.raw_measurements_[0];
      x_(1) = measurement_pack.raw_measurements_[1];
    }

    time_us_ = measurement_pack.timestamp_;

    // Done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Preprocessing
   ****************************************************************************/

  // Time elapsed between current and previous measurements (in s)
  double delta_t = (measurement_pack.timestamp_ - time_us_) / 1000000.0;
	time_us_ = measurement_pack.timestamp_;

  AugmentedSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();

  /*****************************************************************************
   *  Prediction & Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    VectorXd z_out = VectorXd(n_z_radar_);
    MatrixXd S_out = MatrixXd(n_z_radar_, n_z_radar_);
    PredictMeasurement(&z_out, &S_out, true);
    Update(measurement_pack, z_out, S_out, true);

  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    VectorXd z_out = VectorXd(n_z_lidar_);
    MatrixXd S_out = MatrixXd(n_z_lidar_, n_z_lidar_);
    PredictMeasurement(&z_out, &S_out, false);
    Update(measurement_pack, z_out, S_out, false);
  }

  // Print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

/**
 * Augment the sigma points, write result directly into Xsig_aug_.
 */
void UKF::AugmentedSigmaPoints() {

  // Create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // Create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // Create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // Create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
}

/**
 * Predict sigma points, write result directly into Xsig_pred_.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::SigmaPointPrediction(double delta_t) {
  // Predict sigma points
  for (int i = 0; i < n_sig_; i++)
  {
    // Extract values for better readability
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    // Predicted state values
    double px_p, py_p;

    // Avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v / yawd * ( sin(yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // Add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // Write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
}

/**
 * Predict mean and covariance.
 */
void UKF::PredictMeanAndCovariance() {
  // Create vector for predicted state
  //VectorXd x = VectorXd(n_x_);

  // Create covariance matrix for prediction
  //MatrixXd P = MatrixXd(n_x_, n_x_);

  // Predicted state mean
  x_ = Xsig_pred_ * weights_;

  // Predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  // Iterate over sigma points

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization
    x_diff(3) = tools.NormalizeAngle(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Predict radar or lidar measurement.
 * @param {VectorXd*} z_out Vector containing mean predicted measurement.
 * @param {MatrixXd*} S_out Measurement covariance matrix.
 * @param {bool} is_radar Is measurement radar (or lidar)?
 */
void UKF::PredictMeasurement(VectorXd* z_out, MatrixXd* S_out, bool is_radar) {
  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) {  // 2n+1 sigma points

    // Extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v  = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // Measurement model
    if (is_radar) {
      // Avoid domain errors and division by zero
      if (p_y == 0 && p_x == 0) {
        p_y = 0.01;
        p_x = 0.01;
      }
      double rho = sqrt(p_x * p_x + p_y * p_y);
      Zsig_radar_(0, i) = rho;
      Zsig_radar_(1, i) = atan2(p_y, p_x);  // phi
      Zsig_radar_(2, i) = (p_x * v1 + p_y * v2 ) / rho;  // rho_dot
    } else {
      Zsig_lidar_(0, i) = p_x;
      Zsig_lidar_(1, i) = p_y;
    }
  }

  // Mean predicted measurement
  VectorXd z_pred;
  if (is_radar) {
    z_pred = VectorXd(n_z_radar_);
  } else {
    z_pred = VectorXd(n_z_lidar_);
  }
  z_pred.fill(0.0);

  for (int i = 0; i < n_sig_; i++) {
    if (is_radar) {
      z_pred = z_pred + weights_(i) * Zsig_radar_.col(i);
    } else {
      z_pred = z_pred + weights_(i) * Zsig_lidar_.col(i);
    }
  }

  // Measurement covariance matrix S
  MatrixXd S;
  if (is_radar) {
    S = MatrixXd(n_z_radar_ ,n_z_radar_);
  } else {
    S = MatrixXd(n_z_lidar_ ,n_z_lidar_);
  }
  S.fill(0.0);

  for (int i = 0; i < n_sig_; i++) {  // 2n+1 simga points
    // Residual
    VectorXd z_diff;
    if (is_radar) {
      z_diff = Zsig_radar_.col(i) - z_pred;
    } else {
      z_diff = Zsig_lidar_.col(i) - z_pred;
    }

    // Angle normalization
    z_diff(1) = tools.NormalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  if (is_radar) {
    S = S + R_radar_;
  } else {
    S = S + R_lidar_;
  }

  *z_out = z_pred;
  *S_out = S;
}

/**
 * Update state & state covariance matrix using lidar or radar measurement.
 * @param {MeasurementPackage} measurement_pack The measurement at k+1
 * @param {VectorXd} z_pred Vector containg mean predicted measurement
 * @param {MatrixXd} S Matrix containing predicted measurement covariance
 * @param {bool} is_radar Is measurement radar (or lidar)?
 */
void UKF::Update(MeasurementPackage measurement_pack, VectorXd z_pred, MatrixXd S, bool is_radar) {
  // Incoming measurement
  VectorXd z;
  if (is_radar) {
    z = VectorXd(3);
    z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2];
  } else {
    z = VectorXd(2);
    z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
  }

  // Create matrix for cross correlation Tc
  MatrixXd Tc;
  if (is_radar) {
    Tc = MatrixXd(n_x_, n_z_radar_);

  } else {
    Tc = MatrixXd(n_x_, n_z_lidar_);
  }

  // Calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  // 2n+1 sigma points

    // Residual
    VectorXd z_diff;
    if (is_radar) {
      z_diff = Zsig_radar_.col(i) - z_pred;
    } else {
      z_diff = Zsig_lidar_.col(i) - z_pred;
    }

    // Angle normalization
    z_diff(1) = tools.NormalizeAngle(z_diff(1));

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization
    x_diff(3) = tools.NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // Residual
  VectorXd z_diff = z - z_pred;

  // Angle normalization
  z_diff(1) = tools.NormalizeAngle(z_diff(1));


  // Update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}

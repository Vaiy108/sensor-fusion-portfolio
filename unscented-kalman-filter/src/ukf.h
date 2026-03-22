#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructs an Unscented Kalman Filter for fusing lidar and radar
   * measurements.
   */
  UKF();

  /**
   * Destroys the UKF instance.
   */
  virtual ~UKF();

  /**
   * Processes a new sensor measurement.
   *
   * On the first measurement, the filter initializes its state. On subsequent
   * measurements, it performs prediction followed by the appropriate sensor
   * update step.
   *
   * @param meas_package The latest radar or lidar measurement.
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Predicts the state mean and covariance forward in time using the unscented
   * transform and the CTRV motion model.
   *
   * @param delta_t Time elapsed since the previous measurement in seconds.
   */
  void Prediction(double delta_t);

  /**
   * Updates the state using a lidar measurement.
   *
   * Lidar directly observes position in Cartesian coordinates, so this step
   * uses a linear measurement model.
   *
   * @param meas_package The incoming lidar measurement.
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state using a radar measurement.
   *
   * Radar observes range, bearing, and range rate, so this step performs a
   * nonlinear measurement update in radar measurement space.
   *
   * @param meas_package The incoming radar measurement.
   */
  void UpdateRadar(MeasurementPackage meas_package);

  // Indicates whether the filter has received its first measurement.
  bool is_initialized_;

  // Enables or disables lidar updates after initialization.
  bool use_laser_;

  // Enables or disables radar updates after initialization.
  bool use_radar_;

  // State vector [px, py, v, yaw, yaw_rate].
  Eigen::VectorXd x_;

  // State covariance matrix.
  Eigen::MatrixXd P_;

  // Predicted sigma points matrix.
  Eigen::MatrixXd Xsig_pred_;

  // Timestamp of the most recent processed measurement in microseconds.
  long long time_us_;

  // Process noise standard deviation for longitudinal acceleration (m/s^2).
  double std_a_;

  // Process noise standard deviation for yaw acceleration (rad/s^2).
  double std_yawdd_;

  // Lidar measurement noise standard deviation for position x (m).
  double std_laspx_;

  // Lidar measurement noise standard deviation for position y (m).
  double std_laspy_;

  // Radar measurement noise standard deviation for range (m).
  double std_radr_;

  // Radar measurement noise standard deviation for bearing (rad).
  double std_radphi_;

  // Radar measurement noise standard deviation for range rate (m/s).
  double std_radrd_;

  // Weights used for the unscented transform sigma points.
  Eigen::VectorXd weights_;

  // Dimension of the state vector.
  int n_x_;

  // Dimension of the augmented state vector.
  int n_aug_;

  // Sigma-point spreading parameter.
  double lambda_;
};

#endif  // UKF_H

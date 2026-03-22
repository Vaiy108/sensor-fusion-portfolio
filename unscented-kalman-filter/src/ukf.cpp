#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#include "Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes the Unscented Kalman Filter (UKF) with process and measurement
 * noise parameters, state dimensions, and sigma-point weights.
 */
UKF::UKF() {
  // Enable both sensors by default.
  use_laser_ = true;
  use_radar_ = true;

  is_initialized_ = false;
  time_us_ = 0;

  // State dimension: [px, py, v, yaw, yaw_rate]
  n_x_ = 5;

  // Augmented state dimension includes process noise terms.
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  P_ = MatrixXd(n_x_, n_x_);
  P_.setIdentity();

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Process noise standard deviation for longitudinal acceleration (m/s^2)
  std_a_ = 1.5;

  // Process noise standard deviation for yaw acceleration (rad/s^2)
  std_yawdd_ = 0.5;

  /**
   * Measurement noise values are defined by the sensor specifications.
   */

  // Lidar measurement noise standard deviation in px (m)
  std_laspx_ = 0.15;

  // Lidar measurement noise standard deviation in py (m)
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation in range (m)
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation in bearing (rad)
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation in range rate (m/s)
  std_radrd_ = 0.3;

  // Compute sigma-point weights used in the unscented transform.
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // Initialize the filter with the first incoming measurement.
  if (!is_initialized_) {
    x_ << 0, 0, 0, 0, 0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      const double rho = meas_package.raw_measurements_(0);
      const double phi = meas_package.raw_measurements_(1);
      const double rho_dot = meas_package.raw_measurements_(2);

      const double px = rho * std::cos(phi);
      const double py = rho * std::sin(phi);

      // Estimate initial speed from the radar range-rate measurement.
      const double vx = rho_dot * std::cos(phi);
      const double vy = rho_dot * std::sin(phi);
      const double v = std::sqrt(vx * vx + vy * vy);

      x_ << px, py, v, 0, 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      const double px = meas_package.raw_measurements_(0);
      const double py = meas_package.raw_measurements_(1);

      // Lidar provides position only, so velocity and yaw terms start at zero.
      x_ << px, py, 0, 0, 0;
    }

    // Avoid exact zeros in position to reduce the risk of numerical issues.
    if (std::fabs(x_(0)) < 0.001) {
      x_(0) = 0.001;
    }
    if (std::fabs(x_(1)) < 0.001) {
      x_(1) = 0.001;
    }

    P_.setIdentity();

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  const double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER &&
             use_laser_) {
    UpdateLidar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  // Create the augmented mean vector.
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0.0;
  x_aug(n_x_ + 1) = 0.0;

  // Create the augmented covariance matrix.
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  // Compute the square-root matrix for sigma-point generation.
  MatrixXd L = P_aug.llt().matrixL();

  // Generate augmented sigma points.
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;

  const double scaling_factor = std::sqrt(lambda_ + n_aug_);
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + scaling_factor * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - scaling_factor * L.col(i);
  }

  // Predict each sigma point forward using the CTRV motion model.
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    const double p_x = Xsig_aug(0, i);
    const double p_y = Xsig_aug(1, i);
    const double v = Xsig_aug(2, i);
    const double yaw = Xsig_aug(3, i);
    const double yawd = Xsig_aug(4, i);
    const double nu_a = Xsig_aug(5, i);
    const double nu_yawdd = Xsig_aug(6, i);

    double px_p;
    double py_p;

    if (std::fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (std::sin(yaw + yawd * delta_t) - std::sin(yaw));
      py_p = p_y + v / yawd * (std::cos(yaw) - std::cos(yaw + yawd * delta_t));
    } else {
      // Use straight-line motion when yaw rate is close to zero.
      px_p = p_x + v * delta_t * std::cos(yaw);
      py_p = p_y + v * delta_t * std::sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // Add process noise.
    px_p += 0.5 * nu_a * delta_t * delta_t * std::cos(yaw);
    py_p += 0.5 * nu_a * delta_t * delta_t * std::sin(yaw);
    v_p += nu_a * delta_t;

    yaw_p += 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p += nu_yawdd * delta_t;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // Predict the new state mean from the weighted sigma points.
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // Predict the new state covariance.
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Normalize yaw angle to keep it within [-pi, pi].
    while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // Lidar directly measures position, so a standard linear Kalman update is
  // sufficient here.
  const int n_z = 2;

  MatrixXd H = MatrixXd(n_z, n_x_);
  H.fill(0.0);
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;

  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;

  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_pred = H * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H) * P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // Radar measures range, bearing, and range rate, so the sigma points must be
  // projected into nonlinear measurement space.
  const int n_z = 3;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    const double p_x = Xsig_pred_(0, i);
    const double p_y = Xsig_pred_(1, i);
    const double v = Xsig_pred_(2, i);
    const double yaw = Xsig_pred_(3, i);

    const double v1 = std::cos(yaw) * v;
    const double v2 = std::sin(yaw) * v;

    double rho = std::sqrt(p_x * p_x + p_y * p_y);
    if (rho < 0.001) {
      rho = 0.001;
    }

    Zsig(0, i) = rho;
    Zsig(1, i) = std::atan2(p_y, p_x);
    Zsig(2, i) = (p_x * v1 + p_y * v2) / rho;
  }

  // Predict radar measurement mean.
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // Predict radar measurement covariance.
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Normalize radar bearing angle.
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
  S += R;

  // Compute cross-correlation between state space and measurement space.
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Compute Kalman gain.
  MatrixXd K = Tc * S.inverse();

  // Compute measurement residual.
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z - z_pred;
  while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

  // Update state mean and covariance.
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  // Optional consistency metric:
  // double NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
}

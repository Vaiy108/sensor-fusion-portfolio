#include "ekf.h"

#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kPi = 3.14159265358979323846;
}

/**
 * Initializes the Extended Kalman Filter with a constant-velocity state model,
 * sensor noise matrices, and process noise values.
 */
EKF::EKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // State vector: [px, py, vx, vy]
  x_ = VectorXd(4);
  x_.fill(0.0);

  // Initial state covariance matrix:
  // position is relatively well observed at initialization,
  // velocity is much more uncertain.
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  // State transition matrix
  F_ = MatrixXd::Identity(4, 4);

  // Process covariance matrix
  Q_ = MatrixXd(4, 4);
  Q_.fill(0.0);

  // Lidar measurement model: directly observes px and py.
  H_lidar_ = MatrixXd(2, 4);
  H_lidar_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;

  // Lidar measurement noise covariance
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << 0.0225, 0.0,
              0.0, 0.0225;

  // Radar measurement noise covariance
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09,   0.0,    0.0,
              0.0,    0.0009, 0.0,
              0.0,    0.0,    0.09;

  // Process noise
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;
}

EKF::~EKF() {}

void EKF::ProcessMeasurement(const MeasurementPackage& measurement) {
  if (!is_initialized_) {
    Initialize(measurement);
    previous_timestamp_ = measurement.timestamp_;
    is_initialized_ = true;
    return;
  }

  const double dt =
      (measurement.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement.timestamp_;

  Predict(dt);

  if (measurement.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(measurement.raw_measurements_);
  } else if (measurement.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(measurement.raw_measurements_);
  }
}

void EKF::Initialize(const MeasurementPackage& measurement) {
  x_ << 0.0, 0.0, 0.0, 0.0;

  if (measurement.sensor_type_ == MeasurementPackage::LASER) {
    const double px = measurement.raw_measurements_(0);
    const double py = measurement.raw_measurements_(1);

    x_ << px, py, 0.0, 0.0;
  } else if (measurement.sensor_type_ == MeasurementPackage::RADAR) {
    const double rho = measurement.raw_measurements_(0);
    const double phi = measurement.raw_measurements_(1);
    const double rho_dot = measurement.raw_measurements_(2);

    const double px = rho * std::cos(phi);
    const double py = rho * std::sin(phi);
    const double vx = rho_dot * std::cos(phi);
    const double vy = rho_dot * std::sin(phi);

    x_ << px, py, vx, vy;
  }

  // void initialization too close to zero position to protect radar math.
  if (std::fabs(x_(0)) < kEpsilon) {
    x_(0) = kEpsilon;
  }
  if (std::fabs(x_(1)) < kEpsilon) {
    x_(1) = kEpsilon;
  }
}

void EKF::Predict(double dt) {
  // Update the state transition matrix for elapsed time.
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;

  // Process covariance matrix for constant-velocity motion model.
  Q_ << dt4 / 4.0 * noise_ax_, 0.0,                   dt3 / 2.0 * noise_ax_, 0.0,
        0.0,                   dt4 / 4.0 * noise_ay_, 0.0,                   dt3 / 2.0 * noise_ay_,
        dt3 / 2.0 * noise_ax_, 0.0,                   dt2 * noise_ax_,       0.0,
        0.0,                   dt3 / 2.0 * noise_ay_, 0.0,                   dt2 * noise_ay_;

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void EKF::UpdateLidar(const VectorXd& z) {
  const VectorXd z_pred = H_lidar_ * x_;
  const VectorXd y = z - z_pred;

  const MatrixXd Ht = H_lidar_.transpose();
  const MatrixXd S = H_lidar_ * P_ * Ht + R_lidar_;
  const MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + K * y;

  const MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_lidar_) * P_;
}

void EKF::UpdateRadar(const VectorXd& z) {
  const VectorXd h_x = RadarMeasurementFunction(x_);
  VectorXd y = z - h_x;

  NormalizeAngle(y(1));

  const MatrixXd Hj = CalculateJacobian(x_);
  const MatrixXd Hjt = Hj.transpose();
  const MatrixXd S = Hj * P_ * Hjt + R_radar_;
  const MatrixXd K = P_ * Hjt * S.inverse();

  x_ = x_ + K * y;

  const MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * Hj) * P_;
}

VectorXd EKF::RadarMeasurementFunction(const VectorXd& x_state) const {
  VectorXd h(3);

  const double px = x_state(0);
  const double py = x_state(1);
  const double vx = x_state(2);
  const double vy = x_state(3);

  const double c1 = px * px + py * py;
  const double rho = std::sqrt(std::max(c1, kEpsilon));
  const double phi = std::atan2(py, px);
  const double rho_dot = (px * vx + py * vy) / rho;

  h << rho, phi, rho_dot;
  return h;
}

MatrixXd EKF::CalculateJacobian(const VectorXd& x_state) const {
  MatrixXd Hj(3, 4);

  const double px = x_state(0);
  const double py = x_state(1);
  const double vx = x_state(2);
  const double vy = x_state(3);

  const double c1 = px * px + py * py;

  if (std::fabs(c1) < kEpsilon) {
    std::cerr << "Warning: Jacobian calculation skipped due to near-zero position."
              << std::endl;
    Hj.setZero();
    return Hj;
  }

  const double c2 = std::sqrt(c1);
  const double c3 = c1 * c2;

  Hj << px / c2, py / c2, 0.0, 0.0,
       -py / c1, px / c1, 0.0, 0.0,
       py * (vx * py - vy * px) / c3,
       px * (vy * px - vx * py) / c3,
       px / c2,
       py / c2;

  return Hj;
}

void EKF::NormalizeAngle(double& angle) const {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
}
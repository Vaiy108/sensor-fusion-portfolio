#ifndef EKF_H
#define EKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class EKF {
 public:
  EKF();
  virtual ~EKF();

  void ProcessMeasurement(const MeasurementPackage& measurement);

  // Exposed for reporting/debugging in main.
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

 private:
  void Initialize(const MeasurementPackage& measurement);
  void Predict(double dt);
  void UpdateLidar(const Eigen::VectorXd& z);
  void UpdateRadar(const Eigen::VectorXd& z);

  Eigen::VectorXd RadarMeasurementFunction(
      const Eigen::VectorXd& x_state) const;
  Eigen::MatrixXd CalculateJacobian(
      const Eigen::VectorXd& x_state) const;
  void NormalizeAngle(double& angle) const;

  bool is_initialized_;
  long long previous_timestamp_;

  Eigen::MatrixXd F_;
  Eigen::MatrixXd Q_;

  Eigen::MatrixXd H_lidar_;
  Eigen::MatrixXd R_lidar_;
  Eigen::MatrixXd R_radar_;

  double noise_ax_;
  double noise_ay_;
};

#endif  // EKF_H
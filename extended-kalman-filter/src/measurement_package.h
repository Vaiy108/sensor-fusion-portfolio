#ifndef MEASUREMENT_PACKAGE_H
#define MEASUREMENT_PACKAGE_H

#include "Eigen/Dense"

class MeasurementPackage {
 public:
  enum SensorType {
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
  long long timestamp_;
};

#endif  // MEASUREMENT_PACKAGE_H
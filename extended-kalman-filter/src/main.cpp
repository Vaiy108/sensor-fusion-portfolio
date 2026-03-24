#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "ekf.h"
#include "measurement_package.h"
#include "tools.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: ./ekf_demo <input_data_file>" << std::endl;
    return 1;
  }

  const std::string input_file_name = argv[1];
  std::ifstream in_file(input_file_name);

  if (!in_file.is_open()) {
    std::cerr << "Failed to open input file: " << input_file_name << std::endl;
    return 1;
  }

  std::cout << "Starting Extended Kalman Filter..." << std::endl;

  EKF ekf;
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;

  std::string line;
  while (std::getline(in_file, line)) {
    if (line.empty()) {
      continue;
    }

    std::istringstream iss(line);
    char sensor_type;
    iss >> sensor_type;

    MeasurementPackage measurement;
    Eigen::VectorXd gt_values(4);
    long long timestamp = 0;

    if (sensor_type == 'L') {
      double px = 0.0;
      double py = 0.0;
      double gt_px = 0.0;
      double gt_py = 0.0;
      double gt_vx = 0.0;
      double gt_vy = 0.0;

      measurement.sensor_type_ = MeasurementPackage::LASER;
      measurement.raw_measurements_ = Eigen::VectorXd(2);

      iss >> px >> py >> timestamp >> gt_px >> gt_py >> gt_vx >> gt_vy;

      measurement.raw_measurements_ << px, py;
      measurement.timestamp_ = timestamp;

      gt_values << gt_px, gt_py, gt_vx, gt_vy;
    } else if (sensor_type == 'R') {
      double rho = 0.0;
      double phi = 0.0;
      double rho_dot = 0.0;
      double gt_px = 0.0;
      double gt_py = 0.0;
      double gt_vx = 0.0;
      double gt_vy = 0.0;

      measurement.sensor_type_ = MeasurementPackage::RADAR;
      measurement.raw_measurements_ = Eigen::VectorXd(3);

      iss >> rho >> phi >> rho_dot >> timestamp
          >> gt_px >> gt_py >> gt_vx >> gt_vy;

      measurement.raw_measurements_ << rho, phi, rho_dot;
      measurement.timestamp_ = timestamp;

      gt_values << gt_px, gt_py, gt_vx, gt_vy;
    } else {
      std::cerr << "Unknown sensor type in line: " << line << std::endl;
      continue;
    }

    ekf.ProcessMeasurement(measurement);

    estimations.push_back(ekf.x_);
    ground_truth.push_back(gt_values);

    Eigen::VectorXd rmse = Tools::CalculateRMSE(estimations, ground_truth);

    std::cout << "Estimated state x:\n" << ekf.x_ << "\n";
    std::cout << "RMSE: " << rmse.transpose() << "\n\n";
  }

  in_file.close();

  Eigen::VectorXd final_rmse =
      Tools::CalculateRMSE(estimations, ground_truth);

  std::cout << "Final RMSE: " << final_rmse.transpose() << std::endl;

  return 0;
}
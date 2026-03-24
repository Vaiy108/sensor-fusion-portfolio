#include "tools.h"

#include <iostream>

using Eigen::VectorXd;
using std::vector;

VectorXd Tools::CalculateRMSE(
    const vector<VectorXd>& estimations,
    const vector<VectorXd>& ground_truth) {
  VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  if (estimations.empty()) {
    std::cerr << "RMSE calculation error: estimation vector is empty."
              << std::endl;
    return rmse;
  }

  if (estimations.size() != ground_truth.size()) {
    std::cerr << "RMSE calculation error: estimation and ground truth sizes do "
                 "not match."
              << std::endl;
    return rmse;
  }

  for (size_t i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse /= static_cast<double>(estimations.size());
  rmse = rmse.array().sqrt();

  return rmse;
}
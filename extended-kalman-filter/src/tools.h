#ifndef TOOLS_H
#define TOOLS_H

#include "Eigen/Dense"
#include <vector>

class Tools {
 public:
  static Eigen::VectorXd CalculateRMSE(
      const std::vector<Eigen::VectorXd>& estimations,
      const std::vector<Eigen::VectorXd>& ground_truth);
};

#endif  // TOOLS_H
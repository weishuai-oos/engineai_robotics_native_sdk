#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace runner::supermimic {

struct Reference {
  std::vector<Eigen::Matrix3d> rotations;
  Eigen::MatrixXf goals;
  int frames() const { return static_cast<int>(rotations.size()); }
};

Reference LoadReference(const std::string& path);

}  // namespace runner::supermimic

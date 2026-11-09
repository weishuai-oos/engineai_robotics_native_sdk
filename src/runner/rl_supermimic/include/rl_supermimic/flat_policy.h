#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>

namespace MNN { class Interpreter; class Session; class Tensor; }

namespace runner::supermimic {

class FlatPolicy {
 public:
  explicit FlatPolicy(const std::string& path);
  ~FlatPolicy();
  FlatPolicy(const FlatPolicy&) = delete;
  FlatPolicy& operator=(const FlatPolicy&) = delete;
  bool Infer(const Eigen::VectorXf& input, Eigen::VectorXd* action);
 private:
  MNN::Interpreter* net_ = nullptr;
  MNN::Session* session_ = nullptr;
  MNN::Tensor* input_ = nullptr;
  MNN::Tensor* output_ = nullptr;
};

}  // namespace runner::supermimic

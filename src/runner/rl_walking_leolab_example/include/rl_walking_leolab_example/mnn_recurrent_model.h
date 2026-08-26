#pragma once

#include <MNN/Interpreter.hpp>
#include <eigen3/Eigen/Core>

#include <string>

namespace math {

// Small MNN adapter for the deployed CusRL recurrent actor tensor contract:
//   observation + memory_in.0 + memory_in.1
//       -> action + memory_out.0 + memory_out.1
//
// The precompiled core MNNModel intentionally remains unchanged so existing
// feed-forward runners keep their ABI. This adapter is local to the
// leo_lab-compatible runner and owns the recurrent state at the runner layer.
class MNNRecurrentModel {
 public:
  MNNRecurrentModel(const std::string& model_path,
                    const std::string& observation_input_name,
                    const std::string& hidden_input_name,
                    const std::string& cell_input_name,
                    const std::string& action_output_name,
                    const std::string& hidden_output_name,
                    const std::string& cell_output_name);
  ~MNNRecurrentModel();

  MNNRecurrentModel(const MNNRecurrentModel&) = delete;
  MNNRecurrentModel& operator=(const MNNRecurrentModel&) = delete;

  bool IsValid() const { return valid_; }
  int ObservationSize() const;
  int HiddenSize() const;
  int CellSize() const;
  int ActionSize() const;

  bool Inference(const Eigen::VectorXf& observation,
                 const Eigen::VectorXf& hidden_in,
                 const Eigen::VectorXf& cell_in,
                 Eigen::VectorXf* action,
                 Eigen::VectorXf* hidden_out,
                 Eigen::VectorXf* cell_out);

 private:
  MNN::Interpreter* net_ = nullptr;
  MNN::Session* session_ = nullptr;
  MNN::Tensor* observation_tensor_ = nullptr;
  MNN::Tensor* hidden_input_tensor_ = nullptr;
  MNN::Tensor* cell_input_tensor_ = nullptr;
  MNN::Tensor* action_tensor_ = nullptr;
  MNN::Tensor* hidden_output_tensor_ = nullptr;
  MNN::Tensor* cell_output_tensor_ = nullptr;
  bool valid_ = false;
};

}  // namespace math

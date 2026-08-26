#include "rl_walking_leolab_example/mnn_recurrent_model.h"

#include <MNN/MNNForwardType.h>

#include <algorithm>
#include <type_traits>
#include <utility>

#include <glog/logging.h>

namespace math {
namespace {

template <typename CopyOperation>
bool RunTensorCopy(CopyOperation&& copy_operation) {
  if constexpr (std::is_void_v<std::invoke_result_t<CopyOperation>>) {
    std::forward<CopyOperation>(copy_operation)();
    return true;
  } else {
    return static_cast<bool>(std::forward<CopyOperation>(copy_operation)());
  }
}

template <typename TensorMap>
MNN::Tensor* ResolveTensor(const TensorMap& tensors,
                           const std::string& requested_name,
                           const char* tensor_kind) {
  if (requested_name.empty()) {
    LOG(ERROR) << "[MNNRecurrentModel] Configured " << tensor_kind << " tensor name is empty";
    return nullptr;
  }

  const auto exact = tensors.find(requested_name);
  if (exact == tensors.end()) {
    LOG(ERROR) << "[MNNRecurrentModel] Configured " << tensor_kind << " tensor '" << requested_name
               << "' was not found in the MNN model";
    return nullptr;
  }
  return exact->second;
}

bool CopyIntoTensor(MNN::Tensor* tensor, const Eigen::VectorXf& values) {
  if (!tensor || tensor->elementSize() != static_cast<size_t>(values.size())) return false;
  MNN::Tensor host_tensor(tensor, MNN::Tensor::CAFFE);
  float* host_data = host_tensor.host<float>();
  if (!host_data) return false;
  std::copy(values.data(), values.data() + values.size(), host_data);
  return RunTensorCopy([&]() { return tensor->copyFromHostTensor(&host_tensor); });
}

bool CopyFromTensor(MNN::Tensor* tensor, Eigen::VectorXf* values) {
  if (!tensor || !values) return false;
  MNN::Tensor host_tensor(tensor, MNN::Tensor::CAFFE);
  if (!RunTensorCopy([&]() { return tensor->copyToHostTensor(&host_tensor); })) return false;
  const float* host_data = host_tensor.host<float>();
  if (!host_data) return false;
  values->resize(static_cast<int>(tensor->elementSize()));
  std::copy(host_data, host_data + values->size(), values->data());
  return values->allFinite();
}

}  // namespace

MNNRecurrentModel::MNNRecurrentModel(const std::string& model_path,
                                     const std::string& observation_input_name,
                                     const std::string& hidden_input_name,
                                     const std::string& cell_input_name,
                                     const std::string& action_output_name,
                                     const std::string& hidden_output_name,
                                     const std::string& cell_output_name) {
  net_ = MNN::Interpreter::createFromFile(model_path.c_str());
  if (!net_) {
    LOG(ERROR) << "[MNNRecurrentModel] Failed to load policy: " << model_path;
    return;
  }

  MNN::ScheduleConfig schedule_config;
  schedule_config.type = MNN_FORWARD_CPU;
  schedule_config.numThread = 1;
  session_ = net_->createSession(schedule_config);
  if (!session_) {
    LOG(ERROR) << "[MNNRecurrentModel] Failed to create MNN session for: " << model_path;
    return;
  }

  const auto& inputs = net_->getSessionInputAll(session_);
  const auto& outputs = net_->getSessionOutputAll(session_);
  observation_tensor_ = ResolveTensor(inputs, observation_input_name, "observation input");
  hidden_input_tensor_ = ResolveTensor(inputs, hidden_input_name, "hidden-state input");
  cell_input_tensor_ = ResolveTensor(inputs, cell_input_name, "cell-state input");
  action_tensor_ = ResolveTensor(outputs, action_output_name, "action output");
  hidden_output_tensor_ = ResolveTensor(outputs, hidden_output_name, "hidden-state output");
  cell_output_tensor_ = ResolveTensor(outputs, cell_output_name, "cell-state output");

  valid_ = observation_tensor_ && hidden_input_tensor_ && cell_input_tensor_ && action_tensor_ &&
           hidden_output_tensor_ && cell_output_tensor_;
  if (!valid_) {
    LOG(ERROR) << "[MNNRecurrentModel] Recurrent tensor contract is incomplete for: " << model_path
               << ". Inputs=" << inputs.size() << ", outputs=" << outputs.size();
  }
}

MNNRecurrentModel::~MNNRecurrentModel() {
  if (net_) {
    if (session_) net_->releaseSession(session_);
    MNN::Interpreter::destroy(net_);
  }
}

int MNNRecurrentModel::ObservationSize() const {
  return observation_tensor_ ? static_cast<int>(observation_tensor_->elementSize()) : 0;
}

int MNNRecurrentModel::HiddenSize() const {
  return hidden_input_tensor_ ? static_cast<int>(hidden_input_tensor_->elementSize()) : 0;
}

int MNNRecurrentModel::CellSize() const {
  return cell_input_tensor_ ? static_cast<int>(cell_input_tensor_->elementSize()) : 0;
}

int MNNRecurrentModel::ActionSize() const {
  return action_tensor_ ? static_cast<int>(action_tensor_->elementSize()) : 0;
}

bool MNNRecurrentModel::Inference(const Eigen::VectorXf& observation,
                                  const Eigen::VectorXf& hidden_in,
                                  const Eigen::VectorXf& cell_in,
                                  Eigen::VectorXf* action,
                                  Eigen::VectorXf* hidden_out,
                                  Eigen::VectorXf* cell_out) {
  if (!valid_ || !action || !hidden_out || !cell_out) return false;
  if (!CopyIntoTensor(observation_tensor_, observation) || !CopyIntoTensor(hidden_input_tensor_, hidden_in) ||
      !CopyIntoTensor(cell_input_tensor_, cell_in)) {
    return false;
  }
  if (net_->runSession(session_) != MNN::NO_ERROR) return false;
  return CopyFromTensor(action_tensor_, action) && CopyFromTensor(hidden_output_tensor_, hidden_out) &&
         CopyFromTensor(cell_output_tensor_, cell_out);
}

}  // namespace math

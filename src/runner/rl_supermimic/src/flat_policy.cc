#include "rl_supermimic/flat_policy.h"

#include <MNN/Interpreter.hpp>
#include <MNN/MNNForwardType.h>
#include <MNN/Tensor.hpp>
#include <algorithm>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "rl_supermimic/policy_contract.h"

namespace runner::supermimic {
namespace {
template <class Copy>
bool TensorCopy(Copy&& copy) {
  if constexpr (std::is_void_v<std::invoke_result_t<Copy>>) {
    std::forward<Copy>(copy)();
    return true;
  } else return static_cast<bool>(std::forward<Copy>(copy)());
}

bool FloatTensor(MNN::Tensor* tensor, int width) {
  if (!tensor || tensor->shape() != std::vector<int>({1, width})) return false;
  const auto type = tensor->getType();
  return type.code == halide_type_float && type.bits == 32 && type.lanes == 1;
}
}  // namespace

FlatPolicy::FlatPolicy(const std::string& path) {
  try {
    net_ = MNN::Interpreter::createFromFile(path.c_str());
    if (!net_) throw std::runtime_error("Cannot load SuperMimic MNN model: " + path);
    MNN::ScheduleConfig config;
    config.type = MNN_FORWARD_CPU;
    config.numThread = 1;
    MNN::BackendConfig backend;
    backend.precision = MNN::BackendConfig::Precision_High;
    config.backendConfig = &backend;
    session_ = net_->createSession(config);
    if (!session_) throw std::runtime_error("Cannot create SuperMimic MNN session");
    const auto& inputs = net_->getSessionInputAll(session_);
    const auto& outputs = net_->getSessionOutputAll(session_);
    if (inputs.size() != 1 || outputs.size() != 1)
      throw std::runtime_error("SuperMimic needs one input and one output");
    input_ = inputs.begin()->second;
    output_ = outputs.begin()->second;
    if (!FloatTensor(input_, kInput) || !FloatTensor(output_, kActions))
      throw std::runtime_error("SuperMimic MNN contract must be float32 [1,1520] -> [1,25]");
  } catch (...) {
    if (net_) {
      if (session_) net_->releaseSession(session_);
      MNN::Interpreter::destroy(net_);
      net_ = nullptr;
    }
    throw;
  }
}

FlatPolicy::~FlatPolicy() {
  if (net_) {
    if (session_) net_->releaseSession(session_);
    MNN::Interpreter::destroy(net_);
  }
}

bool FlatPolicy::Infer(const Eigen::VectorXf& input, Eigen::VectorXd* action) {
  if (!action || input.size() != kInput || !input.allFinite()) return false;
  MNN::Tensor input_host(input_, MNN::Tensor::CAFFE);
  if (!input_host.host<float>()) return false;
  std::copy(input.data(), input.data() + input.size(), input_host.host<float>());
  if (!TensorCopy([&]() { return input_->copyFromHostTensor(&input_host); }) ||
      net_->runSession(session_) != MNN::NO_ERROR) return false;
  MNN::Tensor output_host(output_, MNN::Tensor::CAFFE);
  if (!TensorCopy([&]() { return output_->copyToHostTensor(&output_host); }) || !output_host.host<float>())
    return false;
  *action = Eigen::Map<const Eigen::VectorXf>(output_host.host<float>(), kActions).cast<double>();
  // Some exports return raw actions; clipped exports enforce their own bounds
  // inside the model. Preserve either output for targets and action history.
  return action->allFinite();
}

}  // namespace runner::supermimic

#include "pd_stand_param/pd_stand_param.h"
#include <cmath>
#include <string>

#include "parameter/parameter_loader.h"
namespace data {

void PdStandParam::Check() {
  const auto model_scope = FindScope("model");
  if (model_scope.empty()) {
    throw std::runtime_error("model parameter scope not found");
  }
  const auto model_node = common::GetGlobalConfigTree().FindNode(model_scope);
  const auto& limbs = model_node["limbs"];

  if (!std::isfinite(duration) || duration <= 0.0) {
    throw std::runtime_error("pd stand duration must be positive");
  }
  if (initial_joint_position_bias_threshold &&
      (!std::isfinite(*initial_joint_position_bias_threshold) || *initial_joint_position_bias_threshold < 0.0)) {
    throw std::runtime_error("initial joint position bias threshold must be non-negative");
  }

  const size_t limb_count = limbs.size();
  if (desired_joint_position.size() != limb_count || stiffness.size() != limb_count || damping.size() != limb_count) {
    throw std::runtime_error("pd stand parameter limb groups do not match model");
  }

  const auto validate_group = [&](const std::vector<Eigen::VectorXd>& group, const char* name) {
    for (size_t limb = 0; limb < limb_count; ++limb) {
      const size_t joint_count = limbs[limb]["joints"].size();
      if (group[limb].size() != joint_count) {
        throw std::runtime_error(std::string("pd stand ") + name + " joint group does not match model");
      }
      if (!group[limb].allFinite()) {
        throw std::runtime_error(std::string("pd stand ") + name + " contains non-finite values");
      }
    }
  };
  validate_group(desired_joint_position, "desired_joint_position");
  validate_group(stiffness, "stiffness");
  validate_group(damping, "damping");
}

}  // namespace data

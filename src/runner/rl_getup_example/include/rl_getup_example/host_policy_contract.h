#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace runner::rl_getup_contract {

struct ObservationScales {
  double angular_velocity = 1.0;
  double dof_position = 1.0;
  double dof_velocity = 1.0;
};

inline void RequireSize(std::size_t actual, std::size_t expected, const char* name) {
  if (actual != expected) {
    throw std::invalid_argument(std::string(name) + " size mismatch: " + std::to_string(actual) +
                                " != " + std::to_string(expected));
  }
}

inline double Clip(double value, double limit) { return std::clamp(value, -limit, limit); }

inline std::vector<double> BuildStepObservation(const std::array<double, 3>& base_ang_vel,
                                                const std::array<double, 3>& projected_gravity,
                                                const std::vector<double>& joint_position,
                                                const std::vector<double>& joint_velocity,
                                                const std::vector<double>& previous_action,
                                                double action_rescale,
                                                const ObservationScales& scales) {
  const std::size_t num_actions = joint_position.size();
  RequireSize(joint_velocity.size(), num_actions, "joint_velocity");
  RequireSize(previous_action.size(), num_actions, "previous_action");

  std::vector<double> observation;
  observation.reserve(7 + 3 * num_actions);
  for (double value : base_ang_vel) {
    observation.push_back(value * scales.angular_velocity);
  }
  for (double value : projected_gravity) {
    observation.push_back(value);
  }
  for (double value : joint_position) {
    observation.push_back(value * scales.dof_position);
  }
  for (double value : joint_velocity) {
    observation.push_back(value * scales.dof_velocity);
  }
  observation.insert(observation.end(), previous_action.begin(), previous_action.end());
  observation.push_back(action_rescale);
  return observation;
}

inline void UpdateObservationHistory(std::vector<double>& history,
                                     int one_step_dim,
                                     int history_steps,
                                     const std::vector<double>& current_observation,
                                     double observation_clip,
                                     bool shift_existing_history) {
  if (one_step_dim <= 0 || history_steps <= 0) {
    throw std::invalid_argument("observation history dimensions must be positive");
  }
  const std::size_t step_dim = static_cast<std::size_t>(one_step_dim);
  const std::size_t steps = static_cast<std::size_t>(history_steps);
  RequireSize(current_observation.size(), step_dim, "current_observation");
  RequireSize(history.size(), step_dim * steps, "history");

  if (shift_existing_history && steps > 1) {
    for (std::size_t column = 0; column + 1 < steps; ++column) {
      for (std::size_t row = 0; row < step_dim; ++row) {
        history[column * step_dim + row] = history[(column + 1) * step_dim + row];
      }
    }
  }

  const std::size_t newest_column_start = (steps - 1) * step_dim;
  for (std::size_t row = 0; row < step_dim; ++row) {
    history[newest_column_start + row] = current_observation[row];
  }
  for (double& value : history) {
    value = Clip(value, observation_clip);
  }
}

inline std::vector<double> ComputeRelativeJointTargets(const std::vector<double>& joint_position,
                                                       const std::vector<double>& raw_action,
                                                       const std::vector<double>& action_scale,
                                                       double action_rescale) {
  const std::size_t num_actions = joint_position.size();
  RequireSize(raw_action.size(), num_actions, "raw_action");
  RequireSize(action_scale.size(), num_actions, "action_scale");

  std::vector<double> target(num_actions);
  for (std::size_t i = 0; i < num_actions; ++i) {
    target[i] = joint_position[i] + raw_action[i] * action_scale[i] * action_rescale;
  }
  return target;
}

}  // namespace runner::rl_getup_contract

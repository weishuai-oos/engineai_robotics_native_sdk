/**
 * @file rl_dance_example_runner.cc
 * @brief Implementation of the RL-based whole-body tracking (WBT) dance Runner.
 *
 * This Runner deploys a reinforcement-learning policy that tracks a pre-recorded
 * reference dance trajectory (loaded from a .npz file). Unlike the walking Runner
 * which uses live gamepad commands, this Runner replays a fixed motion sequence
 * so the robot can perform choreographed dance moves.
 *
 * Core pipeline each control cycle:
 *   1. Compute observations via a registry-based observation system (wbt_obs)
 *   2. Run the MLP policy network inference to get joint actions
 *   3. Map actions to target joint positions and send motor commands
 *
 * Key differences from the walking Runner:
 *   - Uses a **reference trajectory** (.npz) instead of real-time gamepad input
 *   - Observation assembly is **registry-driven** — each observation component is
 *     registered by name and retrieved dynamically via wbt_obs::GetObservation()
 *   - Supports **per-component history buffers** with configurable history lengths
 *   - Performs **yaw alignment** on the first frame to align the reference trajectory
 *     with the robot's actual heading at startup
 *
 * Notes for secondary developers:
 *   - To add new observation types, register them in wbt_obs_registry and add the
 *     name to the `observation_names` parameter list in the YAML config.
 *   - The reference trajectory .npz file must contain keys: "joint_pos", "joint_vel",
 *     "body_quat_w" as float arrays.
 *   - Joint name ordering in `joint_names` must match the policy training configuration.
 */

#include "rl_dance_example/rl_dance_example_runner.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "math/interpolation.h"
#include "math/rotation_matrix.h"
#include "rl_dance_example/wbt_obs_registry.h"

namespace runner {

namespace {

bool IsUpperBodyJointName(const std::string& joint_name) {
  return joint_name.find("SHOULDER") != std::string::npos ||
         joint_name.find("ELBOW") != std::string::npos ||
         joint_name.find("HEAD") != std::string::npos;
}

bool ValidateFloatTrajectoryArray(const cnpy::NpyArray& array, const char* key) {
  if (array.word_size != sizeof(float)) {
    LOG(ERROR) << "[WbtRunner::Enter] Trajectory key '" << key << "' must contain float32 data; word_size="
               << array.word_size;
    return false;
  }
  if (array.fortran_order) {
    LOG(ERROR) << "[WbtRunner::Enter] Trajectory key '" << key
               << "' uses Fortran order, which this Runner does not support";
    return false;
  }
  if (array.shape.size() != 2 && array.shape.size() != 3) {
    LOG(ERROR) << "[WbtRunner::Enter] Trajectory key '" << key << "' must be a 2D or 3D array; dimensions="
               << array.shape.size();
    return false;
  }
  if (!array.data_holder || array.num_vals == 0) {
    LOG(ERROR) << "[WbtRunner::Enter] Trajectory key '" << key << "' is empty";
    return false;
  }
  for (const size_t dimension : array.shape) {
    if (dimension == 0) {
      LOG(ERROR) << "[WbtRunner::Enter] Trajectory key '" << key << "' has a zero-sized dimension";
      return false;
    }
  }
  if (array.num_bytes() != array.num_vals * array.word_size) {
    LOG(ERROR) << "[WbtRunner::Enter] Trajectory key '" << key << "' has an inconsistent data buffer";
    return false;
  }
  return true;
}

}  // namespace

// ============================================================================
// Runner Lifecycle Methods
// ============================================================================

/**
 * @brief Sets up the runtime context before this Runner is scheduled.
 *
 * Disables the classic parser's parallel motion control, as the RL policy
 * has exclusive control over all active joints.
 */
void RlDanceExampleRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(false); }

/**
 * @brief Tears down the runtime context. Currently no cleanup needed.
 */
void RlDanceExampleRunner::TeardownContext() {}

/**
 * @brief Initialization upon entering this Runner. Allocates all resources.
 *
 * Performs the following steps:
 *   1. Load/reload parameters (supports param_tag_ hot-switching)
 *   2. Set up joint PD gains and build policy-to-deploy joint index mapping
 *   3. Load the MLP policy network (.mnn model)
 *   4. Initialize observation and history buffers
 *   5. Load the reference dance trajectory from a .npz file
 *   6. Pre-fill the constant portion of the observation context
 *
 * @return true if initialization succeeds, false on parameter or model load failure.
 */
bool RlDanceExampleRunner::Enter() {
  // --- Step 1: Parameter loading ---
  // Reload parameters if a tag has been set (supports runtime config switching)
  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::RlDanceExampleParam>(param_tag_);
  }
  if (!param_) {
    LOG(ERROR) << "[WbtRunner::Enter] Failed to create WbtParam";
    return false;
  }

  const std::string trajectory_end_behavior = param_->trajectory_end_behavior.value_or("exit");
  if (trajectory_end_behavior != "exit" && trajectory_end_behavior != "hold") {
    LOG(ERROR) << "[WbtRunner::Enter] trajectory_end_behavior must be 'exit' or 'hold', got: "
               << trajectory_end_behavior;
    return false;
  }
  exit_on_trajectory_end_ = trajectory_end_behavior == "exit";

  // --- Step 2: Joint PD gains and index mapping ---
  // Initialize full-body joint arrays (all joints, not just policy-controlled ones)
  joint_kp_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  joint_kd_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  default_joint_q_ = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Zero(model_param_->num_total_joints));

  // Build the mapping from policy action indices to full-body joint indices.
  // policy2deploy_joint_idx_[i] gives the full-body joint index for the i-th policy output.
  policy2deploy_joint_idx_ = std::make_shared<Eigen::VectorXi>(Eigen::VectorXi::Zero(param_->num_actions));
  for (size_t i = 0; i < param_->joint_names.size(); ++i) {
    int deploy_idx = model_param_->joint_id_in_total_limb.at(param_->joint_names[i]);
    (*policy2deploy_joint_idx_)(static_cast<int>(i)) = deploy_idx;
  }

  // Apply PD gains and default positions only to the policy-controlled joints
  joint_kp_(*policy2deploy_joint_idx_) = param_->joint_stiffness;
  joint_kd_(*policy2deploy_joint_idx_) = param_->joint_damping;
  (*default_joint_q_)(*policy2deploy_joint_idx_) = param_->default_joint_pos;
  action_scale_ = param_->action_scale;

  // --- Step 3: Load the MLP policy network ---
  std::string policy_path =
      common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), param_->policy_file);
  mlp_net_ = std::make_unique<math::MNNModel>(policy_path);
  if (!mlp_net_) {
    LOG(ERROR) << "[WbtRunner::Enter] Failed to load policy model";
    return false;
  }

  // Compute the total observation dimension (sum of all observation components × their history lengths)
  int total_obs_dim = ComputeTotalObservationDim();
  if (total_obs_dim <= 0) {
    LOG(ERROR) << "[WbtRunner::Enter] Invalid total observation dimension: " << total_obs_dim;
    return false;
  }
  mlp_net_observation_vec.setZero(total_obs_dim);
  mlp_net_action_ = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Zero(param_->num_actions));

  // --- Step 4: Initialize observation history buffers ---
  // Each observation component has its own sliding-window history buffer
  initHistoryBuffers();

  // --- Step 5: Load reference dance trajectory ---
  // The .npz file contains pre-recorded motion data with keys:
  //   "joint_pos"    — reference joint positions [num_frames x num_actions]
  //   "joint_vel"    — reference joint velocities [num_frames x num_actions]
  //   "body_quat_w"  — reference body quaternion (w, x, y, z) [num_frames x 4]
  std::string traj_path =
      common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), param_->trajectory_file_npz);
  if (!LoadAndValidateTrajectory(traj_path) || !ValidatePolicyContract()) {
    return false;
  }

  // --- Step 6: Reset runtime state ---
  is_first_time_ = true;
  policy_step = 0;
  trajectory_hold_active_ = false;
  startup_interpolation_time_ = 0.0;
  GetMutableOutput().Reset();

  // Pre-fill observation context fields that remain constant throughout execution
  fillObsContextConstantPart();

  if (!InitializeStartupInterpolation()) {
    return false;
  }

  LOG(INFO) << "[WbtRunner::Enter] Done, obs_dim=" << total_obs_dim << ", actions=" << param_->num_actions
            << ", frames=" << ref_joint_pos_all_->rows();
  return true;
}

/**
 * @brief Pre-fills the observation context with references that do not change
 *        between control cycles.
 *
 * The ObsContext struct is shared with all observation computation functions
 * registered in the wbt_obs system. This method sets the "constant" fields
 * (data store reference, trajectory data, joint mappings, etc.) so that only
 * the per-cycle fields (like policy_step) need updating in the main loop.
 */
void RlDanceExampleRunner::fillObsContextConstantPart() {
  obs_ctx_.data_store = data_store_;
  obs_ctx_.ref_joint_pos_all = ref_joint_pos_all_;
  obs_ctx_.ref_joint_vel_all = ref_joint_vel_all_;
  obs_ctx_.ref_body_quat_w_all = ref_body_quat_w_all_;
  obs_ctx_.num_actions = param_->num_actions;
  obs_ctx_.default_joint_q = default_joint_q_;
  obs_ctx_.policy2deploy_joint_idx = policy2deploy_joint_idx_;
  obs_ctx_.actions = mlp_net_action_;
}

// ============================================================================
// Main Control Loop
// ============================================================================

/**
 * @brief Main loop called once per control cycle.
 *
 * Executes the perception → decision → action pipeline and advances the
 * trajectory frame counter. The frame counter is clamped to max_policy_step,
 * so the robot holds the final pose once the trajectory is fully played.
 */
void RlDanceExampleRunner::Run() {
  const double control_period = GetControlPeriod();

  if (trajectory_hold_active_) {
    SendMotorCommand();
    return;
  }

  if (!startup_policy_started_) {
    const double startup_phase =
        std::min(startup_interpolation_time_ + control_period, lower_body_startup_interpolation_duration_);

    q_des_ = startup_full_q_init_;
    qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
    tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
    ApplyLowerBodyStartupInterpolation(startup_phase);
    ApplyUpperBodyStartupInterpolation(startup_phase);
    SendMotorCommand();

    startup_interpolation_time_ = startup_phase;
    if (startup_interpolation_time_ >= lower_body_startup_interpolation_duration_) {
      startup_policy_started_ = true;
      lower_body_policy_blend_time_ = 0.0;
      lower_body_policy_blend_start_q_ = lower_body_q_target_;
      is_first_time_ = true;
      policy_step = 0;
      mlp_net_action_->setZero();
    }
    return;
  }

  CalculateObservation();   // Assemble observation from registered components
  CalculateMotorCommand();  // Run policy inference and compute target positions

  if (lower_body_policy_blend_duration_ > 0.0 &&
      lower_body_policy_blend_time_ < lower_body_policy_blend_duration_) {
    lower_body_policy_blend_time_ =
        std::min(lower_body_policy_blend_time_ + control_period, lower_body_policy_blend_duration_);
    ApplyLowerBodyPolicyBlend(lower_body_policy_blend_time_);
  }

  if (startup_interpolation_time_ < upper_body_startup_total_duration_) {
    startup_interpolation_time_ =
        std::min(startup_interpolation_time_ + control_period, upper_body_startup_total_duration_);
    ApplyUpperBodyStartupInterpolation(startup_interpolation_time_);
  }

  SendMotorCommand();       // Send PD commands to motors

  // Advance trajectory frame and loop back to the start.
  // policy_step = (policy_step >= max_policy_step) ? 0 : policy_step + 1;
  policy_step = std::min(policy_step + 1, max_policy_step);
}

// ============================================================================
// Observation Assembly
// ============================================================================

/**
 * @brief Assembles the full observation vector from registry-based observation components.
 *
 * Unlike the walking Runner which manually concatenates sensor data, this Runner
 * uses a dynamic registry system (wbt_obs). Each observation component is:
 *   1. Looked up by name from the `observation_names` config list
 *   2. Computed by its registered function via wbt_obs::GetObservation()
 *   3. Maintained in its own sliding-window history buffer
 *   4. Flattened (column-major) into the final observation vector
 *
 * On the first frame:
 *   - Yaw alignment is performed to match the reference trajectory heading
 *   - All history buffers are pre-filled with the current observation
 *     (avoids feeding zero-initialized history to the policy)
 *
 * @note The observation composition and ordering are fully determined by the YAML
 *       config parameter `observation_names`. Adding or reordering entries requires
 *       retraining the policy model.
 */
void RlDanceExampleRunner::CalculateObservation() {
  // On the very first frame, align the yaw angle between the reference trajectory
  // and the robot's actual heading direction
  if (is_first_time_) {
    updateFirstFrameYawAlignment();
  }

  // Update the per-cycle observation context field
  obs_ctx_.policy_step = policy_step;

  int output_offset = 0;
  for (size_t i = 0; i < param_->observation_names.size(); ++i) {
    const std::string& obs_name = param_->observation_names[i];

    // Compute a single-step observation for this component via the registry
    Eigen::VectorXd single = wbt_obs::GetObservation(obs_name, obs_ctx_);

    // Update the sliding-window history buffer for this component
    Eigen::MatrixXd& buf = observation_history_buffers_[i];
    const int hist_len = static_cast<int>(buf.cols());
    if (is_first_time_) {
      // First frame: replicate the current observation across all history steps
      buf.colwise() = single;
    } else {
      // Subsequent frames: shift buffer left by one, insert newest at rightmost column
      if (hist_len > 1) {
        buf.leftCols(hist_len - 1) = buf.rightCols(hist_len - 1).eval();
      }
      buf.rightCols(1) = single;
    }

    // Flatten this component's history buffer (column-major) into the observation vector
    mlp_net_observation_vec.segment(output_offset, buf.size()) =
        Eigen::Map<const Eigen::VectorXd>(buf.data(), buf.size());
    output_offset += buf.size();
  }

  if (is_first_time_) {
    is_first_time_ = false;
  }
}

/**
 * @brief Computes yaw alignment on the first frame.
 *
 * Extracts the yaw angle from both:
 *   - The robot's current IMU orientation (actual heading)
 *   - The reference trajectory's first-frame body orientation
 *
 * These yaw rotations are stored and used by observation functions to transform
 * reference trajectory data into the robot's local coordinate frame. This ensures
 * the dance motion starts in the direction the robot is actually facing, regardless
 * of its initial heading.
 */
void RlDanceExampleRunner::updateFirstFrameYawAlignment() {
  // Get the robot's current orientation from IMU
  Eigen::Matrix3d R_local = math::RotationMatrixd(data_store_->imu_info.Get()->quaternion).matrix();

  // Get the reference trajectory's first-frame body orientation (quaternion: w, x, y, z)
  Eigen::Quaterniond ref_anchor_ori_quat_w(
      (*ref_body_quat_w_all_)(policy_step, 0), (*ref_body_quat_w_all_)(policy_step, 1),
      (*ref_body_quat_w_all_)(policy_step, 2), (*ref_body_quat_w_all_)(policy_step, 3));
  Eigen::Matrix3d ref_anchor_ori_rot_w = math::RotationMatrixd(ref_anchor_ori_quat_w).matrix();

  // Extract yaw angles (rotation about Z-axis) from both orientations
  double ref_yaw = std::atan2(ref_anchor_ori_rot_w(1, 0), ref_anchor_ori_rot_w(0, 0));
  double body_yaw = std::atan2(R_local(1, 0), R_local(0, 0));

  // Store pure yaw rotation matrices for coordinate frame alignment in observations
  ref_init_yaw_rot_ = Eigen::AngleAxisd(ref_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  body_init_yaw_rot_ = Eigen::AngleAxisd(body_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Share with the observation context so observation functions can use them
  obs_ctx_.ref_init_yaw_rot = ref_init_yaw_rot_;
  obs_ctx_.body_init_yaw_rot = body_init_yaw_rot_;
}

// ============================================================================
// Policy Inference and Motor Command
// ============================================================================

/**
 * @brief Runs the MLP policy network inference and computes target joint positions.
 *
 * Pipeline:
 *   1. Read current joint positions and velocities (for state feedback)
 *   2. Forward the assembled observation vector through the MNN model
 *   3. Map the action output to target joint positions:
 *      q_des[active_joints] = ref_joint_pos + action * action_scale
 *
 * @note This matches the tracking training task where the policy outputs a residual
 *       around the reference trajectory joint positions, not an absolute joint target
 *       around the default pose.
 */
void RlDanceExampleRunner::CalculateMotorCommand() {
  // Read current joint state (used internally by some observation functions
  // but NOT directly used in action computation here)
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd_real_);

  // Run MLP forward inference (float precision, cast back to double)
  *mlp_net_action_ = (mlp_net_->Inference(mlp_net_observation_vec.cast<float>())).cast<double>();

  // Map action to target joint positions:
  //   q_des = ref_joint_pos + action * action_scale (for policy-controlled joints only)

  q_des_ = *default_joint_q_;
  if (param_->resident_control) {
    const int ref_step = std::min(policy_step, max_policy_step);
    const Eigen::VectorXd ref_joint_pos = ref_joint_pos_all_->row(ref_step);
    const Eigen::VectorXd scaled_action = mlp_net_action_->cwiseProduct(action_scale_);
    q_des_(*policy2deploy_joint_idx_) = ref_joint_pos + scaled_action;
  } else {
    q_des_(*policy2deploy_joint_idx_) += mlp_net_action_->cwiseProduct(action_scale_);
  }
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);

  if (IsTrajectoryFinished()) {
    if (exit_on_trajectory_end_) {
      SetRunnerState(RunnerState::kTryExit);
    } else {
      trajectory_hold_active_ = true;
      LOG(INFO) << "Trajectory finished; holding the final motor command until an operator transition.";
    }
  }
}

bool RlDanceExampleRunner::InitializeStartupInterpolation() {
  lower_body_startup_interpolation_duration_ =
      param_->lower_body_startup_interpolation_duration.value_or(0.0);
  upper_body_startup_interpolation_duration_ =
      param_->upper_body_startup_interpolation_duration.value_or(
          param_->startup_interpolation_duration.value_or(0.0));
  lower_body_policy_blend_duration_ = param_->lower_body_policy_blend_duration.value_or(0.0);
  upper_body_startup_total_duration_ =
      lower_body_startup_interpolation_duration_ + upper_body_startup_interpolation_duration_;
  startup_interpolation_time_ = 0.0;
  lower_body_policy_blend_time_ = 0.0;
  startup_policy_started_ = lower_body_startup_interpolation_duration_ <= 0.0;
  upper_body_interpolation_target_step_ = 0;

  if (lower_body_startup_interpolation_duration_ < 0.0 ||
      upper_body_startup_interpolation_duration_ < 0.0 ||
      lower_body_policy_blend_duration_ < 0.0) {
    LOG(ERROR) << "Startup interpolation durations must be non-negative. lower_body="
               << lower_body_startup_interpolation_duration_
               << ", upper_body=" << upper_body_startup_interpolation_duration_
               << ", lower_body_blend=" << lower_body_policy_blend_duration_;
    return false;
  }
  if ((lower_body_startup_interpolation_duration_ > 0.0 ||
       upper_body_startup_interpolation_duration_ > 0.0 ||
       lower_body_policy_blend_duration_ > 0.0) &&
      runner_period_ <= 0.0) {
    LOG(WARNING) << "Runner period is not set. Startup interpolation falls back to 0.02s control period.";
  }
  if (!ref_joint_pos_all_ || ref_joint_pos_all_->rows() == 0 ||
      ref_joint_pos_all_->cols() != param_->num_actions) {
    LOG(ERROR) << "Invalid reference joint position trajectory for startup interpolation.";
    return false;
  }

  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  startup_full_q_init_ = q_real_;
  upper_body_interpolation_target_step_ =
      std::min(static_cast<int>(std::ceil(upper_body_startup_interpolation_duration_ / GetControlPeriod())),
               max_policy_step);

  std::vector<int> lower_body_action_idx;
  std::vector<int> lower_body_joint_idx;
  std::vector<int> upper_body_action_idx;
  std::vector<int> upper_body_joint_idx;
  for (int action_idx = 0; action_idx < param_->num_actions; ++action_idx) {
    if (action_idx >= static_cast<int>(param_->joint_names.size())) {
      LOG(ERROR) << "joint_names size is smaller than num_actions.";
      return false;
    }
    const int joint_idx = (*policy2deploy_joint_idx_)(action_idx);
    if (IsUpperBodyJointName(param_->joint_names[action_idx])) {
      upper_body_action_idx.push_back(action_idx);
      upper_body_joint_idx.push_back(joint_idx);
    } else {
      lower_body_action_idx.push_back(action_idx);
      lower_body_joint_idx.push_back(joint_idx);
    }
  }

  if ((lower_body_startup_interpolation_duration_ > 0.0 ||
       lower_body_policy_blend_duration_ > 0.0) &&
      lower_body_action_idx.empty()) {
    LOG(ERROR) << "No lower-body/torso joints found for startup interpolation.";
    return false;
  }
  if (upper_body_startup_interpolation_duration_ > 0.0 && upper_body_action_idx.empty()) {
    LOG(ERROR) << "No upper-body joints found for startup interpolation.";
    return false;
  }

  const int num_lower_body_joints = static_cast<int>(lower_body_action_idx.size());
  lower_body_startup_joint_idx_.resize(num_lower_body_joints);
  lower_body_q_init_.resize(num_lower_body_joints);
  lower_body_q_target_.resize(num_lower_body_joints);
  lower_body_policy_blend_start_q_.resize(num_lower_body_joints);
  for (int i = 0; i < num_lower_body_joints; ++i) {
    const int action_idx = lower_body_action_idx[i];
    const int joint_idx = lower_body_joint_idx[i];
    lower_body_startup_joint_idx_(i) = joint_idx;
    lower_body_q_init_(i) = q_real_(joint_idx);
    lower_body_q_target_(i) = (*ref_joint_pos_all_)(0, action_idx);
    lower_body_policy_blend_start_q_(i) = lower_body_q_init_(i);
  }

  const int num_upper_body_joints = static_cast<int>(upper_body_action_idx.size());
  upper_body_startup_joint_idx_.resize(num_upper_body_joints);
  upper_body_q_init_.resize(num_upper_body_joints);
  upper_body_q_target_.resize(num_upper_body_joints);
  for (int i = 0; i < num_upper_body_joints; ++i) {
    const int action_idx = upper_body_action_idx[i];
    const int joint_idx = upper_body_joint_idx[i];
    upper_body_startup_joint_idx_(i) = joint_idx;
    upper_body_q_init_(i) = q_real_(joint_idx);
    upper_body_q_target_(i) = (*ref_joint_pos_all_)(upper_body_interpolation_target_step_, action_idx);
  }

  q_des_ = q_real_;
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  mlp_net_action_->setZero();

  LOG(INFO) << "[WbtRunner::Enter] Startup transition: lower_body_duration="
            << lower_body_startup_interpolation_duration_
            << "s, upper_body_duration=" << upper_body_startup_interpolation_duration_
            << "s, lower_body_blend=" << lower_body_policy_blend_duration_
            << "s, upper_target_step=" << upper_body_interpolation_target_step_
            << ", lower_joints=" << num_lower_body_joints
            << ", upper_joints=" << num_upper_body_joints;
  return true;
}

void RlDanceExampleRunner::ApplyLowerBodyStartupInterpolation(double phase) {
  if (lower_body_startup_interpolation_duration_ <= 0.0 || lower_body_startup_joint_idx_.size() == 0) {
    return;
  }

  const double clamped_phase = std::min(std::max(phase, 0.0), lower_body_startup_interpolation_duration_);
  const int num_startup_joints = static_cast<int>(lower_body_startup_joint_idx_.size());
  Eigen::VectorXd q_cmd = Eigen::VectorXd::Zero(num_startup_joints);
  Eigen::VectorXd qd_cmd = Eigen::VectorXd::Zero(num_startup_joints);
  math::QuinticInterpolate(lower_body_q_init_, lower_body_q_target_, lower_body_startup_interpolation_duration_,
                           clamped_phase, q_cmd, qd_cmd);
  for (int i = 0; i < num_startup_joints; ++i) {
    const int joint_idx = lower_body_startup_joint_idx_(i);
    q_des_(joint_idx) = q_cmd(i);
    qd_des_(joint_idx) = qd_cmd(i);
  }
}

void RlDanceExampleRunner::ApplyUpperBodyStartupInterpolation(double phase) {
  if (upper_body_startup_interpolation_duration_ <= 0.0 || upper_body_startup_joint_idx_.size() == 0) {
    return;
  }

  const double upper_body_phase = phase - lower_body_startup_interpolation_duration_;
  const double clamped_phase =
      std::min(std::max(upper_body_phase, 0.0), upper_body_startup_interpolation_duration_);
  const int num_startup_joints = static_cast<int>(upper_body_startup_joint_idx_.size());
  Eigen::VectorXd q_cmd = Eigen::VectorXd::Zero(num_startup_joints);
  Eigen::VectorXd qd_cmd = Eigen::VectorXd::Zero(num_startup_joints);
  math::QuinticInterpolate(upper_body_q_init_, upper_body_q_target_, upper_body_startup_interpolation_duration_,
                           clamped_phase, q_cmd, qd_cmd);
  for (int i = 0; i < num_startup_joints; ++i) {
    const int joint_idx = upper_body_startup_joint_idx_(i);
    q_des_(joint_idx) = q_cmd(i);
    qd_des_(joint_idx) = qd_cmd(i);
  }
}

void RlDanceExampleRunner::ApplyLowerBodyPolicyBlend(double phase) {
  if (lower_body_policy_blend_duration_ <= 0.0 || lower_body_startup_joint_idx_.size() == 0) {
    return;
  }

  const double clamped_phase = std::min(std::max(phase, 0.0), lower_body_policy_blend_duration_);
  const int num_blend_joints = static_cast<int>(lower_body_startup_joint_idx_.size());
  Eigen::VectorXd q_policy = Eigen::VectorXd::Zero(num_blend_joints);
  Eigen::VectorXd q_cmd = Eigen::VectorXd::Zero(num_blend_joints);
  for (int i = 0; i < num_blend_joints; ++i) {
    q_policy(i) = q_des_(lower_body_startup_joint_idx_(i));
  }
  math::QuinticInterpolate(lower_body_policy_blend_start_q_, q_policy, lower_body_policy_blend_duration_,
                           clamped_phase, q_cmd);
  for (int i = 0; i < num_blend_joints; ++i) {
    const int joint_idx = lower_body_startup_joint_idx_(i);
    q_des_(joint_idx) = q_cmd(i);
    qd_des_(joint_idx) = 0.0;
  }
}

double RlDanceExampleRunner::GetControlPeriod() const {
  constexpr double kFallbackControlPeriod = 0.02;
  return runner_period_ > 0.0 ? runner_period_ : kFallbackControlPeriod;
}

/**
 * @brief Sends computed target positions to the motor controllers via PD control.
 *
 * The low-level driver computes: tau = kp*(q_des-q) + kd*(qd_des-qd) + tau_ff
 */
void RlDanceExampleRunner::SendMotorCommand() {
  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_, tau_ff_des_);
}

// ============================================================================
// Runner Exit Logic
// ============================================================================

/**
 * @brief Immediately allows exit (no graceful transition needed for dance playback).
 */
TransitionState RlDanceExampleRunner::TryExit() { return TransitionState::kCompleted; }

/**
 * @brief Post-exit cleanup. Currently no additional actions needed.
 */
bool RlDanceExampleRunner::Exit() { return true; }

/**
 * @brief Runner termination. Currently no additional actions needed.
 */
void RlDanceExampleRunner::End() {}

// ============================================================================
// Utility Methods
// ============================================================================

/**
 * @brief Initializes the per-observation-component history buffers.
 *
 * Each observation component in `observation_names` gets its own history buffer
 * with dimensions [component_dim x history_length]. The history length is read
 * from `observation_history_lengths`, defaulting to 1 (no history) if not specified.
 *
 * For example, if observation "joint_pos" has dim=12 and history_length=3,
 * its buffer will be a 12x3 matrix, flattening to 36 elements in the final
 * observation vector.
 */
void RlDanceExampleRunner::initHistoryBuffers() {
  observation_history_buffers_.clear();
  observation_history_buffers_.reserve(param_->observation_names.size());
  for (size_t i = 0; i < param_->observation_names.size(); ++i) {
    int dim = GetObservationDim(param_->observation_names[i]);
    int hist_len = (i < param_->observation_history_lengths.size()) ? param_->observation_history_lengths[i] : 1;
    observation_history_buffers_.emplace_back(dim, hist_len);
    observation_history_buffers_.back().setZero();
  }
}

bool RlDanceExampleRunner::LoadAndValidateTrajectory(const std::string& trajectory_path) {
  try {
    trajectory_npz = cnpy::npz_load(trajectory_path);

    const auto joint_pos_it = trajectory_npz.find("joint_pos");
    const auto joint_vel_it = trajectory_npz.find("joint_vel");
    const auto body_quat_it = trajectory_npz.find("body_quat_w");
    if (joint_pos_it == trajectory_npz.end() || joint_vel_it == trajectory_npz.end() ||
        body_quat_it == trajectory_npz.end()) {
      LOG(ERROR) << "[WbtRunner::Enter] Trajectory must contain joint_pos, joint_vel, and body_quat_w: "
                 << trajectory_path;
      return false;
    }
    if (!ValidateFloatTrajectoryArray(joint_pos_it->second, "joint_pos") ||
        !ValidateFloatTrajectoryArray(joint_vel_it->second, "joint_vel") ||
        !ValidateFloatTrajectoryArray(body_quat_it->second, "body_quat_w")) {
      return false;
    }

    trajectory_body_index_ = param_->trajectory_body_index.value_or(0);
    if (trajectory_body_index_ < 0) {
      LOG(ERROR) << "[WbtRunner::Enter] trajectory_body_index must be non-negative";
      return false;
    }
    if (body_quat_it->second.shape.size() == 2 && trajectory_body_index_ != 0) {
      LOG(ERROR) << "[WbtRunner::Enter] trajectory_body_index must be 0 for a 2D body_quat_w array";
      return false;
    }

    Eigen::MatrixXd joint_pos = npyFloatToMatrixXd(joint_pos_it->second);
    Eigen::MatrixXd joint_vel = npyFloatToMatrixXd(joint_vel_it->second);
    Eigen::MatrixXd body_quat = npyFloatToMatrixXd(body_quat_it->second, trajectory_body_index_);
    if (joint_pos.rows() <= 0 || joint_pos.cols() != param_->num_actions) {
      LOG(ERROR) << "[WbtRunner::Enter] joint_pos Runner view must be N x " << param_->num_actions
                 << "; got " << joint_pos.rows() << " x " << joint_pos.cols();
      return false;
    }
    if (joint_vel.rows() != joint_pos.rows() || joint_vel.cols() != param_->num_actions) {
      LOG(ERROR) << "[WbtRunner::Enter] joint_vel Runner view must match joint_pos; got " << joint_vel.rows()
                 << " x " << joint_vel.cols();
      return false;
    }
    if (body_quat.rows() != joint_pos.rows() || body_quat.cols() != 4) {
      LOG(ERROR) << "[WbtRunner::Enter] body_quat_w Runner view must be N x 4 and match joint_pos; got "
                 << body_quat.rows() << " x " << body_quat.cols();
      return false;
    }
    if (!joint_pos.allFinite() || !joint_vel.allFinite() || !body_quat.allFinite()) {
      LOG(ERROR) << "[WbtRunner::Enter] Trajectory contains non-finite values";
      return false;
    }
    for (int frame = 0; frame < body_quat.rows(); ++frame) {
      const double quaternion_norm = body_quat.row(frame).norm();
      if (!std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1e-3) {
        LOG(ERROR) << "[WbtRunner::Enter] body_quat_w frame " << frame
                   << " is not a unit quaternion; norm=" << quaternion_norm;
        return false;
      }
    }

    ref_joint_pos_all_ = std::make_shared<const Eigen::MatrixXd>(std::move(joint_pos));
    ref_joint_vel_all_ = std::make_shared<const Eigen::MatrixXd>(std::move(joint_vel));
    ref_body_quat_w_all_ = std::make_shared<const Eigen::MatrixXd>(std::move(body_quat));
    max_policy_step = static_cast<int>(ref_joint_pos_all_->rows()) - 1;
    return true;
  } catch (const std::exception& error) {
    LOG(ERROR) << "[WbtRunner::Enter] Failed to load trajectory '" << trajectory_path << "': " << error.what();
    return false;
  }
}

bool RlDanceExampleRunner::ValidatePolicyContract() {
  try {
    const Eigen::VectorXd policy_action =
        (mlp_net_->Inference(Eigen::VectorXf::Zero(mlp_net_observation_vec.size()))).cast<double>();
    if (policy_action.size() != param_->num_actions) {
      LOG(ERROR) << "[WbtRunner::Enter] Policy output dimension mismatch: " << policy_action.size()
                 << " != " << param_->num_actions;
      return false;
    }
    if (!policy_action.allFinite()) {
      LOG(ERROR) << "[WbtRunner::Enter] Policy dry-run produced non-finite output";
      return false;
    }
    mlp_net_action_->setZero();
    return true;
  } catch (const std::exception& error) {
    LOG(ERROR) << "[WbtRunner::Enter] Policy dry-run failed: " << error.what();
    return false;
  }
}

/**
 * @brief Converts a cnpy NpyArray (float) to an Eigen MatrixXd (double).
 *
 * Supports two array shapes:
 *   - 2D array [rows x cols]: directly converted to MatrixXd
 *   - 3D array [dim0 x dim1 x dim2]: extracts a 2D slice at the given row_index
 *     along dim1, producing a [dim0 x dim2] matrix
 *
 * @param npy_array The numpy array loaded from .npz file
 * @param row_index For 3D arrays, the index along the second dimension to extract
 * @return Eigen::MatrixXd containing the converted data
 * @throws std::runtime_error if array dimensions are not 2D or 3D, or if row_index is invalid
 */
Eigen::MatrixXd RlDanceExampleRunner::npyFloatToMatrixXd(const cnpy::NpyArray& npy_array, int row_index) {
  const std::vector<size_t>& shape = npy_array.shape;
  const float* data = npy_array.data<float>();
  if (shape.size() == 2) {
    size_t rows = shape[0], cols = shape[1];
    Eigen::MatrixXd mat(rows, cols);
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j) mat(i, j) = static_cast<double>(data[i * cols + j]);
    return mat;
  }
  if (shape.size() == 3) {
    size_t dim0 = shape[0], dim1 = shape[1], dim2 = shape[2];
    if (row_index < 0 || static_cast<size_t>(row_index) >= dim1)
      throw std::runtime_error("Invalid row_index: " + std::to_string(row_index));
    Eigen::MatrixXd mat(dim0, dim2);
    for (size_t d0 = 0; d0 < dim0; ++d0)
      for (size_t d2 = 0; d2 < dim2; ++d2)
        mat(d0, d2) = static_cast<double>(data[d0 * (dim1 * dim2) + row_index * dim2 + d2]);
    return mat;
  }
  throw std::runtime_error("Unsupported array dimension: " + std::to_string(shape.size()));
}

/**
 * @brief Returns the dimension of a named observation component.
 * @param name The observation component name (as registered in wbt_obs)
 * @return The number of elements in a single-step observation for this component
 */
int RlDanceExampleRunner::GetObservationDim(const std::string& name) const {
  return wbt_obs::GetObservationDim(name, param_->num_actions);
}

/**
 * @brief Computes the total flattened observation vector dimension.
 *
 * Sums (component_dim × history_length) for all observation components.
 * This determines the input dimension of the policy network.
 *
 * @return Total number of elements in the assembled observation vector
 */
int RlDanceExampleRunner::ComputeTotalObservationDim() const {
  int total = 0;
  for (size_t i = 0; i < param_->observation_names.size(); ++i) {
    int hist_len = (i < param_->observation_history_lengths.size()) ? param_->observation_history_lengths[i] : 1;
    total += GetObservationDim(param_->observation_names[i]) * hist_len;
  }
  return total;
}

bool RlDanceExampleRunner::IsTrajectoryFinished() const {
  return ref_joint_pos_all_ && policy_step >= static_cast<int>(ref_joint_pos_all_->rows()) - 1;
}

}  // namespace runner

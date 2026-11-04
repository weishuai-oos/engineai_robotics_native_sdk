#include "lcm_interface/lcm_subscriber.h"

LcmSubscriber::LcmSubscriber(const data::LcmParam& param, const std::shared_ptr<LcmDataStore>& lcm_data_store)
    : lcm_interface::PeriodicTask("lcm_subscriber", /* period */ 0.002), lcm_data_store_(lcm_data_store) {
  std::string url;
  int ttl = param.multicast ? param.ttl : 0;
  url = "udpm://" + param.ip_port + "?ttl=" + std::to_string(ttl);
  LOG(INFO) << "LCM URL: " << url;
  lcm_ = std::make_shared<lcm::LCM>(url);
  if (!lcm_->good()) {
    LOG(FATAL) << "LCM is not good.";
  }

  lcm_->subscribe(channel_, &LcmSubscriber::HandleSimCommand, this);
}

void LcmSubscriber::TaskInit() {}

void LcmSubscriber::TaskRun() {
  constexpr int kTimeoutMillis = 2;
  int result = lcm_->handleTimeout(kTimeoutMillis);
  int handled = result > 0 ? 1 : 0;
  while (result > 0 && handled < 64) {
    result = lcm_->handleTimeout(0);
    if (result > 0) {
      ++handled;
    }
  }

  if (result < 0) {
    LOG_EVERY_T(ERROR, 1.0) << "Failed to handle sim_command LCM message.";
  } else if (handled == 64) {
    LOG_EVERY_T(WARNING, 1.0) << "sim_command LCM queue reached drain limit.";
  }
}

void LcmSubscriber::HandleSimCommand(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                                     const data::SimCommand* msg) {
  const int n = lcm_data_store_->num_joints;
  if (msg->num_ranges != n || msg->joint_position.size() != static_cast<size_t>(n) ||
      msg->joint_velocity.size() != static_cast<size_t>(n) ||
      msg->joint_feed_forward_torque.size() != static_cast<size_t>(n) ||
      msg->joint_stiffness.size() != static_cast<size_t>(n) ||
      msg->joint_damping.size() != static_cast<size_t>(n)) {
    LOG_EVERY_T(WARNING, 1.0) << "Ignoring sim_command with incompatible dimensions";
    return;
  }
  sim_command_.q = Eigen::Map<const Eigen::VectorXd>(msg->joint_position.data(), msg->num_ranges);
  sim_command_.qd = Eigen::Map<const Eigen::VectorXd>(msg->joint_velocity.data(), msg->num_ranges);
  sim_command_.tau_ff = Eigen::Map<const Eigen::VectorXd>(msg->joint_feed_forward_torque.data(), msg->num_ranges);
  sim_command_.kp = Eigen::Map<const Eigen::VectorXd>(msg->joint_stiffness.data(), msg->num_ranges);
  sim_command_.kd = Eigen::Map<const Eigen::VectorXd>(msg->joint_damping.data(), msg->num_ranges);

  if (!sim_command_.q.allFinite() || !sim_command_.qd.allFinite() || !sim_command_.tau_ff.allFinite() ||
      !sim_command_.kp.allFinite() || !sim_command_.kd.allFinite()) {
    LOG_EVERY_T(WARNING, 1.0) << "Ignoring non-finite sim_command";
    return;
  }

  lcm_data_store_->sim_command.Set(sim_command_);
  lcm_data_store_->last_command_time_ns.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

#include "input_command_arbiter/rc02_input_adapter.h"

#include <glog/logging.h>

#include "input_command_arbiter/gamepad_key_encoding.h"
#include "parameter/parameter_loader.h"

namespace runner {

namespace {

constexpr double kRc02InputRetainDuration = 0.3;
constexpr auto kRc02ReconnectTimeout = std::chrono::seconds(1);

}  // namespace

// =============================================================================
// 生命周期
// =============================================================================

Rc02InputAdapter::Rc02InputAdapter(std::string name, const std::shared_ptr<data::DataStore>& data_store)
    : BaseInputAdapter(name, data_store) {}

Rc02InputAdapter::~Rc02InputAdapter() {
  if (reconnect_task_.valid()) reconnect_task_.wait();
}

bool Rc02InputAdapter::Init() {
  if (reconnect_task_.valid()) {
    reconnect_task_.wait();
    reconnect_task_ = {};
  }
  rc02_.Close();
  hardware_rc02_publisher_ =
      data::VariantStore::GetInstance().CreatePublisher<data::GamepadInfo>("hardware/rc02_info");
  failed_count_ = 0;
  driver_initialized_ = false;
  rc02_data_available_ = false;
  prev_rc02_data_available_ = false;
  StopRetaining();
  rc02_input_.Reset();
  config_loaded_ = LoadRc02Config();
  if (!config_loaded_) return false;

  // Align with CheckRc02Connected's retry modulus so the first Init call actually attempts serial init.
  static constexpr uint32_t kDriverRetryIntervalCount = 200;
  failed_count_ = kDriverRetryIntervalCount - 1;
  static_cast<void>(TryReconnectRc02());

  LOG(INFO) << "Rc02InputAdapter configured; initialization handshake is running in background.";
  return true;
}

InputAdapterStatus Rc02InputAdapter::Run() {
  rc02_data_available_ = false;
  if (!CheckRc02Connected()) {
    driver_initialized_ = false;
    if (!TryReconnectRc02()) {
      StopRetaining();
      prev_rc02_data_available_ = false;
      rc02_input_.Reset();
      hardware_rc02_publisher_.Publish(rc02_input_);
      return InputAdapterStatus::LOST;
    }
  }

  const bool was_retaining = prev_rc02_data_available_;
  const bool got_valid_rc02_input = rc02_.SendMotionAndACK(0, 0);
  if (got_valid_rc02_input) {
    last_rc02_input_time_ = std::chrono::steady_clock::now();
    const hardware::Rc02InputData raw_input = rc02_.GetInputData();
    rc02_input_.Reset();
    GetKeyInputFromRc02Raw(raw_input);
    RetainInputCommand(kRc02InputRetainDuration);
    rc02_data_available_ = true;
  } else if (IsRetaining()) {
    rc02_data_available_ = true;
  } else {
    if (was_retaining) {
      LOG(WARNING) << "Rc02InputAdapter: rc02 data reception timed out (no new data within retain duration "
                   << kRc02InputRetainDuration << "s).";
    }
    StopRetaining();
    rc02_input_.Reset();

    // An open UART does not prove that the radio/remote is still connected.
    // Re-run the full handshake after a receiver/remote restart or silent link.
    if (std::chrono::steady_clock::now() - last_rc02_input_time_ >= kRc02ReconnectTimeout) {
      LOG(WARNING) << "Rc02InputAdapter: no RC02 input for 1s; closing serial port to repeat initialization handshake.";
      rc02_.Close();
      driver_initialized_ = false;
    }
  }
  prev_rc02_data_available_ = rc02_data_available_;

  hardware_rc02_publisher_.Publish(rc02_input_);
  Log();
  return driver_initialized_ ? InputAdapterStatus::NORMAL : InputAdapterStatus::LOST;
}

void Rc02InputAdapter::Process(data::GamepadInfo& input) { input = rc02_input_; }

bool Rc02InputAdapter::IsActive() const { return rc02_data_available_; }

// =============================================================================
// 连接管理
// =============================================================================

bool Rc02InputAdapter::CheckRc02Connected() { return driver_initialized_ && rc02_.IsConnected(); }

bool Rc02InputAdapter::LoadRc02Config() {
  // The scope is relative to the selected robot's config directory and is
  // copied by install.sh together with that robot's motion/key configuration.
  try {
    constexpr std::string_view kScope = "rc02/default";
    const int product = common::ScopedParameterGetter<int>::Get(kScope, "product");
    if (product <= 0 || product > 127) {
      LOG(ERROR) << "RC02 product must be in [1, 127], got " << product;
      return false;
    }
    const auto protocol = common::ScopedParameterGetter<std::string>::Get(kScope, "protocol_version");
    const auto motion = common::ScopedParameterGetter<std::string>::Get(kScope, "motion_version");
    if (protocol.empty() || motion.empty()) {
      LOG(ERROR) << "RC02 handshake versions must not be empty.";
      return false;
    }
    robot_model_ = static_cast<int8_t>(product);
    protocol_version_ = protocol;
    motion_version_ = motion;
    return true;
  } catch (const std::exception& e) {
    LOG_EVERY_N(ERROR, 200) << "Rc02InputAdapter: failed to initialize RC02 with rc02/default.yaml: " << e.what();
    rc02_.Close();
    return false;
  }
}

bool Rc02InputAdapter::TryReconnectRc02() {
  static constexpr uint32_t kDriverRetryIntervalCount = 200;

  if (!config_loaded_) return false;
  if (!reconnect_task_.valid()) {
    if (++failed_count_ % kDriverRetryIntervalCount != 0) return false;
    // Query/ACK timeouts can take up to 1.5s. Keep them off the 2ms input
    // thread so virtual-input release and expiration continue to run.
    try {
      reconnect_task_ = std::async(std::launch::async, [this] {
        return rc02_.Connect(robot_model_, protocol_version_, motion_version_);
      });
    } catch (const std::exception& e) {
      LOG(ERROR) << "Rc02InputAdapter: could not start handshake task: " << e.what();
    }
    return false;
  }
  if (reconnect_task_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return false;
  try {
    if (!reconnect_task_.get()) return false;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Rc02InputAdapter: handshake task failed: " << e.what();
    rc02_.Close();
    return false;
  }

  driver_initialized_ = true;
  last_rc02_input_time_ = std::chrono::steady_clock::now();
  StopRetaining();
  rc02_input_.Reset();
  prev_rc02_data_available_ = false;
  failed_count_ = 0;
  LOG(INFO) << "Rc02InputAdapter: RC02Driver initialized successfully (init ACK confirmed).";
  return true;
}

// =============================================================================
// 输入解析
// =============================================================================

void Rc02InputAdapter::GetKeyInputFromRc02Raw(const hardware::Rc02InputData& raw) {
  rc02_input_.hardware_connected = true;

  // Map RC02 input to GamepadInfo (SELECT is intentionally ignored).
  rc02_input_.LB = raw.L1;
  rc02_input_.RB = raw.R1;
  rc02_input_.A = raw.A;
  rc02_input_.B = raw.B;
  rc02_input_.X = raw.X;
  rc02_input_.Y = raw.Y;
  rc02_input_.BACK = raw.BACK;
  rc02_input_.START = raw.START;
  rc02_input_.LT = raw.L2;
  rc02_input_.RT = raw.R2;
  rc02_input_.LeftStick_X = raw.LeftStick_ud;
  rc02_input_.LeftStick_Y = raw.LeftStick_lr;
  rc02_input_.RightStick_X = raw.RightStick_ud;
  rc02_input_.RightStick_Y = raw.RightStick_lr;

  if (raw.CROSS_UP)
    rc02_input_.CROSS_X = 1;
  else if (raw.CROSS_DOWN)
    rc02_input_.CROSS_X = -1;
  else
    rc02_input_.CROSS_X = 0;

  if (raw.CROSS_LEFT)
    rc02_input_.CROSS_Y = 1;
  else if (raw.CROSS_RIGHT)
    rc02_input_.CROSS_Y = -1;
  else
    rc02_input_.CROSS_Y = 0;

  int key_value = 0;
  if (raw.L1) key_value |= gamepad_tool_.KeyStringToValue("LB");
  if (raw.R1) key_value |= gamepad_tool_.KeyStringToValue("RB");
  if (raw.A) key_value |= gamepad_tool_.KeyStringToValue("A");
  if (raw.B) key_value |= gamepad_tool_.KeyStringToValue("B");
  if (raw.X) key_value |= gamepad_tool_.KeyStringToValue("X");
  if (raw.Y) key_value |= gamepad_tool_.KeyStringToValue("Y");
  if (raw.BACK) key_value |= gamepad_tool_.KeyStringToValue("BACK");
  if (raw.START) key_value |= gamepad_tool_.KeyStringToValue("START");
  if (raw.CROSS_UP) key_value |= gamepad_tool_.KeyStringToValue("CROSS_X_UP");
  if (raw.CROSS_DOWN) key_value |= gamepad_tool_.KeyStringToValue("CROSS_X_DOWN");
  if (raw.CROSS_LEFT) key_value |= gamepad_tool_.KeyStringToValue("CROSS_Y_LEFT");
  if (raw.CROSS_RIGHT) key_value |= gamepad_tool_.KeyStringToValue("CROSS_Y_RIGHT");
  AddLtTaskModifier(raw.L2, &key_value);
  rc02_input_.combined_key_value = key_value;
}

}  // namespace runner

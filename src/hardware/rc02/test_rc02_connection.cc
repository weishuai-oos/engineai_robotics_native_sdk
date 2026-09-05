#include "rc02/rc02_driver.h"

#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace {

using hardware::LIB_CRC_Checksum;

constexpr auto kIoTimeout = std::chrono::milliseconds(1500);

std::array<uint8_t, 9> HardwareVersionResponse() {
  return {0xAA, 0x05, 0x04, 'R', '0', '0', '9', 0x0B, 0xFF};
}

std::array<uint8_t, 25> ControlInputResponse() {
  std::array<uint8_t, 25> response{0xAA, 0x15, 0x0B, 0x07, 0x80, 0x40,
                                   0x0A, 0xEC, 0xE2, 0x28, 0xFA, 0x7F,
                                   0xFF, 0x12, 0x34, 0,    0,    0,
                                   0,    0,    0,    0,    0,    0,    0};
  const uint16_t crc = LIB_CRC_Checksum(response.data() + 1, 22);
  response[23] = static_cast<uint8_t>(crc & 0xFF);
  response[24] = static_cast<uint8_t>(crc >> 8);
  return response;
}

constexpr std::array<uint8_t, 5> kGoodAck{0xAA, 0x01, 0x0D, 0xF1, 0xFF};
constexpr std::array<uint8_t, 5> kHardwareRequest{0xAA, 0x01, 0x04, 0xFA, 0xFF};

enum class AckMode { kFragmentedGood, kCorrupted, kMissing };

class PtyRc02Emulator {
 public:
  explicit PtyRc02Emulator(AckMode ack_mode) : ack_mode_(ack_mode) {
    master_fd_ = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (master_fd_ < 0) throw std::runtime_error("posix_openpt failed: " + std::string(std::strerror(errno)));
    if (grantpt(master_fd_) != 0 || unlockpt(master_fd_) != 0) {
      const std::string error = std::strerror(errno);
      close(master_fd_);
      master_fd_ = -1;
      throw std::runtime_error("grantpt/unlockpt failed: " + error);
    }
    char* name = ptsname(master_fd_);
    if (name == nullptr) {
      const std::string error = std::strerror(errno);
      close(master_fd_);
      master_fd_ = -1;
      throw std::runtime_error("ptsname failed: " + error);
    }
    slave_path_ = name;
    worker_ = std::jthread([this](std::stop_token stop_token) { Run(stop_token); });
  }

  ~PtyRc02Emulator() {
    worker_.request_stop();
    if (worker_.joinable()) worker_.join();
    if (master_fd_ >= 0) close(master_fd_);
  }

  PtyRc02Emulator(const PtyRc02Emulator&) = delete;
  PtyRc02Emulator& operator=(const PtyRc02Emulator&) = delete;

  const std::string& slave_path() const { return slave_path_; }

  bool WaitForQueries(size_t count) { return WaitForCount(&query_count_, count); }
  bool WaitForInitFrames(size_t count) { return WaitForCount(&init_count_, count); }
  bool WaitForStatusFrames(size_t count) { return WaitForCount(&status_count_, count); }

  size_t init_frame_count() const {
    std::lock_guard lock(mutex_);
    return init_count_;
  }

  std::vector<uint8_t> LastInitFrame() const {
    std::lock_guard lock(mutex_);
    return init_frames_.empty() ? std::vector<uint8_t>{} : init_frames_.back();
  }

  std::vector<uint8_t> LastStatusFrame() const {
    std::lock_guard lock(mutex_);
    return status_frames_.empty() ? std::vector<uint8_t>{} : status_frames_.back();
  }

 private:
  bool WaitForCount(const size_t* count, size_t expected) {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, kIoTimeout, [count, expected] { return *count >= expected; });
  }

  void WriteAll(const uint8_t* data, size_t size) {
    size_t written = 0;
    while (written < size) {
      const ssize_t result = write(master_fd_, data + written, size - written);
      if (result > 0) {
        written += static_cast<size_t>(result);
      } else if (result < 0 && errno == EINTR) {
        continue;
      } else {
        return;
      }
    }
  }

  void RespondToInit() {
    if (ack_mode_ == AckMode::kMissing) return;

    std::array<uint8_t, 5> ack = kGoodAck;
    if (ack_mode_ == AckMode::kCorrupted) ack.back() = 0x00;
    const uint8_t noise = 0x55;
    WriteAll(&noise, 1);
    WriteAll(ack.data(), 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    WriteAll(ack.data() + 2, ack.size() - 2);
  }

  void HandleFrame(std::vector<uint8_t> frame) {
    if (frame == std::vector<uint8_t>(kHardwareRequest.begin(), kHardwareRequest.end())) {
      {
        std::lock_guard lock(mutex_);
        ++query_count_;
      }
      condition_.notify_all();
      const auto response = HardwareVersionResponse();
      WriteAll(response.data(), response.size());
      return;
    }

    if (frame.size() == 55 && frame[0] == 0xAA && frame[1] == 0x33 && frame[2] == 0x0D) {
      {
        std::lock_guard lock(mutex_);
        ++init_count_;
        init_frames_.push_back(std::move(frame));
      }
      condition_.notify_all();
      RespondToInit();
      return;
    }

    if (frame.size() == 9 && frame[0] == 0xAA && frame[1] == 0x05 && frame[2] == 0x0C) {
      {
        std::lock_guard lock(mutex_);
        ++status_count_;
        status_frames_.push_back(std::move(frame));
      }
      condition_.notify_all();
      const auto response = ControlInputResponse();
      WriteAll(response.data(), response.size());
    }
  }

  void Run(std::stop_token stop_token) {
    std::vector<uint8_t> received;
    std::array<uint8_t, 256> buffer{};
    while (!stop_token.stop_requested()) {
      pollfd descriptor{master_fd_, POLLIN, 0};
      const int ready = poll(&descriptor, 1, 20);
      if (ready <= 0) continue;
      if ((descriptor.revents & POLLIN) == 0) continue;

      const ssize_t count = read(master_fd_, buffer.data(), buffer.size());
      if (count <= 0) continue;
      received.insert(received.end(), buffer.begin(), buffer.begin() + count);

      while (received.size() >= 2) {
        if (received[0] != 0xAA) {
          received.erase(received.begin());
          continue;
        }
        const size_t frame_size = 4 + received[1];
        if (received.size() < frame_size) break;
        std::vector<uint8_t> frame(received.begin(), received.begin() + frame_size);
        received.erase(received.begin(), received.begin() + frame_size);
        HandleFrame(std::move(frame));
      }
    }
  }

  const AckMode ack_mode_;
  int master_fd_ = -1;
  std::string slave_path_;
  std::jthread worker_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  size_t query_count_ = 0;
  size_t init_count_ = 0;
  size_t status_count_ = 0;
  std::vector<std::vector<uint8_t>> init_frames_;
  std::vector<std::vector<uint8_t>> status_frames_;
};

class ScopedEnv {
 public:
  ScopedEnv(const char* name, const char* value) : name_(name) {
    if (const char* old = std::getenv(name_)) old_value_ = old;
    if (setenv(name_, value, 1) != 0) {
      throw std::runtime_error("setenv failed: " + std::string(std::strerror(errno)));
    }
  }

  ~ScopedEnv() {
    if (old_value_.has_value()) {
      setenv(name_, old_value_->c_str(), 1);
    } else {
      unsetenv(name_);
    }
  }

 private:
  const char* name_;
  std::optional<std::string> old_value_;
};

std::array<uint8_t, 55> ExpectedT800Init() {
  std::array<uint8_t, 55> frame{};
  frame[0] = 0xAA;
  frame[1] = 0x33;
  frame[2] = 0x0D;
  frame[3] = 0x04;
  frame[4] = 0x03;
  frame[5] = 0x00;
  frame[6] = 0x01;
  frame[7] = 0x03;
  frame[8] = 0x00;
  frame[9] = 0x01;
  frame[10] = 0x03;
  frame[11] = 0x00;
  frame[12] = 0x01;
  frame[13] = 0x09;
  frame[53] = 0xA6;
  frame[54] = 0xFF;
  return frame;
}

TEST(Rc02ConnectionTest, ConnectQueriesHardwareSendsExactInitAndAcceptsFragmentedAck) {
  PtyRc02Emulator emulator(AckMode::kFragmentedGood);
  hardware::RC02Driver driver(emulator.slave_path());

  ASSERT_TRUE(driver.Connect(4, "1.0.3", "1.0.3"));
  ASSERT_TRUE(emulator.WaitForQueries(1));
  ASSERT_TRUE(emulator.WaitForInitFrames(1));
  EXPECT_TRUE(driver.IsConnected());
  const auto expected_init = ExpectedT800Init();
  EXPECT_EQ(emulator.LastInitFrame(), std::vector<uint8_t>(expected_init.begin(), expected_init.end()));

  EXPECT_TRUE(driver.SendMotionStatus(0x12, 0x34));
  ASSERT_TRUE(emulator.WaitForStatusFrames(1));
  EXPECT_EQ(emulator.LastStatusFrame(),
            (std::vector<uint8_t>{0xAA, 0x05, 0x0C, 0x12, 0x34, 0x00, 0x00, 0xA8, 0xFF}));
  driver.Close();
}

TEST(Rc02ConnectionTest, SendMotionAndAckReadsPhysicalInputAfterConnect) {
  PtyRc02Emulator emulator(AckMode::kFragmentedGood);
  hardware::RC02Driver driver(emulator.slave_path());

  ASSERT_TRUE(driver.Connect(4, "1.0.3", "1.0.3"));
  bool received_input = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    if (driver.SendMotionAndACK(0x00, 0x00)) {
      received_input = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(received_input);
  ASSERT_TRUE(emulator.WaitForStatusFrames(1));

  const hardware::Rc02InputData& input = driver.GetInputData();
  EXPECT_EQ(input.heartbeat, 7);
  EXPECT_TRUE(input.L1);
  EXPECT_TRUE(input.A);
  // Preserve the SDK's digital-trigger mapping used by LT task combinations,
  // even when the frame also carries nonzero analog-trigger bytes.
  EXPECT_DOUBLE_EQ(input.L2, 1.0);
  EXPECT_DOUBLE_EQ(input.R2, 0.0);
  EXPECT_NEAR(input.LeftStick_ud, -10.0 / 127.0, 1e-9);
  EXPECT_NEAR(input.LeftStick_lr, 20.0 / 127.0, 1e-9);
  EXPECT_NEAR(input.RightStick_ud, 30.0 / 127.0, 1e-9);
  EXPECT_NEAR(input.RightStick_lr, -40.0 / 127.0, 1e-9);
  driver.Close();
}

TEST(Rc02ConnectionTest, RepeatConnectPerformsTheCompleteHandshakeAgain) {
  PtyRc02Emulator emulator(AckMode::kFragmentedGood);
  hardware::RC02Driver driver(emulator.slave_path());

  ASSERT_TRUE(driver.Connect(4, "1.0.3", "1.0.3"));
  ASSERT_TRUE(driver.Connect(4, "1.0.3", "1.0.3"));
  ASSERT_TRUE(emulator.WaitForQueries(2));
  ASSERT_TRUE(emulator.WaitForInitFrames(2));
  EXPECT_TRUE(driver.IsConnected());
  driver.Close();
}

TEST(Rc02ConnectionTest, CorruptedAckRejectsHandshakeAndClosesPort) {
  PtyRc02Emulator emulator(AckMode::kCorrupted);
  hardware::RC02Driver driver(emulator.slave_path());

  EXPECT_FALSE(driver.Connect(4, "1.0.3", "1.0.3"));
  EXPECT_TRUE(emulator.WaitForQueries(1));
  EXPECT_TRUE(emulator.WaitForInitFrames(1));
  EXPECT_FALSE(driver.IsConnected());
}

TEST(Rc02ConnectionTest, MissingAckRejectsHandshakeAndClosesPort) {
  PtyRc02Emulator emulator(AckMode::kMissing);
  hardware::RC02Driver driver(emulator.slave_path());

  EXPECT_FALSE(driver.Connect(4, "1.0.3", "1.0.3"));
  EXPECT_TRUE(emulator.WaitForQueries(1));
  EXPECT_TRUE(emulator.WaitForInitFrames(1));
  EXPECT_FALSE(driver.IsConnected());
}

TEST(Rc02ConnectionTest, InvalidConfigurationDoesNotCompleteHandshake) {
  PtyRc02Emulator emulator(AckMode::kFragmentedGood);
  hardware::RC02Driver driver(emulator.slave_path());

  EXPECT_FALSE(driver.Connect(4, "not-a-version", "1.0.3"));
  ASSERT_TRUE(emulator.WaitForQueries(1));
  EXPECT_EQ(emulator.init_frame_count(), 0u);
  EXPECT_FALSE(driver.IsConnected());
}

TEST(Rc02ConnectionTest, ExplicitMotionVersionOverridesEnvironmentAndEmptyUsesFallback) {
  ScopedEnv motion_version("ENGINEAI_ROBOTICS_VERSION", "1.0.2");
  hardware::RC02Driver driver;

  const hardware::Rc02InitInfo explicit_version = driver.BuildInitData(4, "1.0.1", "1.0.3");
  EXPECT_EQ(explicit_version.Motion_patch, 3);
  EXPECT_EQ(explicit_version.Motion_minor, 0);
  EXPECT_EQ(explicit_version.Motion_major, 1);

  const hardware::Rc02InitInfo env_version = driver.BuildInitData(4, "1.0.1", "");
  EXPECT_EQ(env_version.Motion_patch, 2);
  EXPECT_EQ(env_version.Motion_minor, 0);
  EXPECT_EQ(env_version.Motion_major, 1);

  const hardware::Rc02InitInfo suffixed_version =
      driver.BuildInitData(4, "1.0.3+hotfix.v2", "1.0.3+hotfix.v2");
  EXPECT_EQ(suffixed_version.Protocol_patch, 3);
  EXPECT_EQ(suffixed_version.Motion_patch, 3);
  EXPECT_THROW(driver.BuildInitData(4, "1.0", "1.0.3"), std::invalid_argument);
  EXPECT_THROW(driver.BuildInitData(4, "1.0.3", "1.0"), std::invalid_argument);
}

}  // namespace

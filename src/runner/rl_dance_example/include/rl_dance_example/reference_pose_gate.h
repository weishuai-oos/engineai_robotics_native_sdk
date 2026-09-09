#pragma once

#include <cmath>

namespace runner {

// Small, deterministic gate used while a trajectory runner brings the robot to
// the trajectory's frame-zero pose.  It deliberately has no timeout: a policy
// must not start until the measured pose has settled for the configured number
// of consecutive cycles.
class ReferencePoseGate {
 public:
  bool Configure(bool enabled, double tolerance, int settle_cycles) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || settle_cycles <= 0) {
      configured_ = false;
      return false;
    }
    enabled_ = enabled;
    tolerance_ = tolerance;
    settle_cycles_required_ = settle_cycles;
    configured_ = true;
    Reset();
    return true;
  }

  void Reset() {
    settling_ = configured_ && enabled_;
    settle_cycles_ = 0;
  }

  bool IsActive() const { return settling_; }
  int settle_cycles() const { return settle_cycles_; }
  int settle_cycles_required() const { return settle_cycles_required_; }

  // Returns whether the gate remains active after this cycle.  During the
  // bridge itself, no settling count is accumulated.  A non-finite error is
  // always treated as unsafe and resets the consecutive count.
  bool Observe(bool bridge_active, double max_tracking_error) {
    if (!settling_) return false;
    if (bridge_active) {
      settle_cycles_ = 0;
      return true;
    }
    if (std::isfinite(max_tracking_error) && max_tracking_error <= tolerance_) {
      ++settle_cycles_;
    } else {
      settle_cycles_ = 0;
    }
    if (settle_cycles_ >= settle_cycles_required_) settling_ = false;
    return settling_;
  }

 private:
  bool configured_ = false;
  bool enabled_ = false;
  bool settling_ = false;
  double tolerance_ = 0.12;
  int settle_cycles_required_ = 8;
  int settle_cycles_ = 0;
};

}  // namespace runner

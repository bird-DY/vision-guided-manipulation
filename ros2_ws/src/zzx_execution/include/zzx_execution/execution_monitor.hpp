// Copyright 2026 zzx
// SPDX-License-Identifier: MIT
#pragma once
#include <vector>

namespace zzx_execution
{
enum class ControllerResult {running, succeeded, canceled, failed};
enum class Verdict {running, settling, succeeded, stale, failed, timeout,
  stopping, stopped, stop_unconfirmed};

struct MonitorConfig
{
  std::vector<bool> continuous;
  double position_tolerance{0.001};
  double velocity_tolerance{0.01};
  double settling_seconds{0.1};
  double feedback_timeout{0.2};
  double stop_timeout{0.5};
};

// All times are seconds in one monotonic clock domain. No ROS or wall-clock sleeps.
class ExecutionMonitor
{
public:
  explicit ExecutionMonitor(MonitorConfig config);
  void start(const std::vector<double> & target, double now, double timeout);
  void request_stop(double now);
  Verdict update(double now, double stamp, const std::vector<double> & positions,
    bool available, ControllerResult controller);
  static double difference(double target, double actual, bool continuous);

private:
  MonitorConfig config_;
  std::vector<double> target_, previous_;
  double deadline_{0}, stop_deadline_{0}, last_stamp_{-1}, stable_since_{-1};
  double last_now_{-1};
  bool stopping_{false}, started_{false};
};
}  // namespace zzx_execution

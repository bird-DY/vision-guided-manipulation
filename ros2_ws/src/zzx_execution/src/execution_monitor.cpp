// Copyright 2026 zzx
// SPDX-License-Identifier: MIT
#include "zzx_execution/execution_monitor.hpp"
#include <cmath>
#include <stdexcept>
#include <utility>

namespace zzx_execution
{
ExecutionMonitor::ExecutionMonitor(MonitorConfig config) : config_(std::move(config))
{
  for (const auto value : {config_.position_tolerance, config_.velocity_tolerance,
      config_.settling_seconds, config_.feedback_timeout, config_.stop_timeout})
  {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("monitor thresholds must be finite and positive");
    }
  }
  if (config_.continuous.empty() || config_.stop_timeout <= config_.settling_seconds) {
    throw std::invalid_argument("monitor needs joints and sufficient stop timeout");
  }
}

double ExecutionMonitor::difference(double target, double actual, bool continuous)
{
  return continuous ? std::remainder(target - actual, 2 * std::acos(-1.0)) : target - actual;
}

void ExecutionMonitor::start(const std::vector<double> & target, double now, double timeout)
{
  if (target.size() != config_.continuous.size() || !std::isfinite(now) || now < 0 ||
    !std::isfinite(timeout) || timeout <= 0 || !std::isfinite(now + timeout))
  {
    throw std::invalid_argument("invalid monitor target or deadline");
  }
  for (double value : target) {
    if (!std::isfinite(value)) {throw std::invalid_argument("nonfinite monitor target");}
  }
  target_ = target;
  previous_.clear();
  deadline_ = now + timeout;
  last_now_ = now;
  last_stamp_ = stable_since_ = -1;
  stopping_ = false;
  started_ = true;
}

void ExecutionMonitor::request_stop(double now)
{
  if (!started_ || !std::isfinite(now) || now < last_now_) {
    throw std::invalid_argument("invalid stop time");
  }
  if (stopping_) {return;}
  stopping_ = true;
  stop_deadline_ = now + config_.stop_timeout;
  stable_since_ = -1;
  previous_.clear();
}

Verdict ExecutionMonitor::update(double now, double stamp,
  const std::vector<double> & positions, bool available, ControllerResult controller)
{
  if (!started_ || !std::isfinite(now) || now < last_now_) {
    throw std::invalid_argument("monitor clock must be monotonic");
  }
  last_now_ = now;
  // Deadlines take priority over terminal success, including a late stop confirmation.
  if (stopping_ && now >= stop_deadline_) {return Verdict::stop_unconfirmed;}
  if (!stopping_ && now >= deadline_) {return Verdict::timeout;}
  bool fresh = available && std::isfinite(stamp) && stamp <= now &&
    now - stamp <= config_.feedback_timeout && stamp >= last_stamp_ &&
    positions.size() == target_.size();
  for (double value : positions) {fresh = fresh && std::isfinite(value);}
  if (!fresh) {
    stable_since_ = -1;
    previous_.clear();
    return stopping_ ? Verdict::stopping : Verdict::stale;
  }
  if (!stopping_ && (controller == ControllerResult::failed ||
    controller == ControllerResult::canceled)) {return Verdict::failed;}
  // Replayed frames cannot accumulate settling evidence.
  if (stamp == last_stamp_) {return stopping_ ? Verdict::stopping : Verdict::running;}
  if (last_stamp_ >= 0 && stamp - last_stamp_ > config_.feedback_timeout) {
    previous_.clear();
    stable_since_ = -1;
  }
  bool stable = !previous_.empty();
  for (size_t i = 0; i < positions.size(); ++i) {
    if (!stopping_) {
      stable = stable && std::abs(difference(target_[i], positions[i],
        config_.continuous[i])) <= config_.position_tolerance;
    }
    if (!previous_.empty()) {
      stable = stable && std::abs(difference(positions[i], previous_[i],
        config_.continuous[i])) / (stamp - last_stamp_) <= config_.velocity_tolerance;
    }
  }
  previous_ = positions;
  last_stamp_ = stamp;
  const bool terminal = controller == ControllerResult::succeeded ||
    (stopping_ && controller == ControllerResult::canceled);
  if (!stable || !terminal) {
    stable_since_ = -1;
    return stopping_ ? Verdict::stopping : Verdict::running;
  }
  if (stable_since_ < 0) {stable_since_ = stamp;}
  if (stamp - stable_since_ >= config_.settling_seconds) {
    return stopping_ ? Verdict::stopped : Verdict::succeeded;
  }
  return stopping_ ? Verdict::stopping : Verdict::settling;
}
}  // namespace zzx_execution

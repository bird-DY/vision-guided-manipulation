// Copyright 2026 zzx
// SPDX-License-Identifier: MIT
#include "zzx_execution/fake_arm_backend.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace zzx_execution
{
Fault parse_fault(const std::string & value)
{
  if (value == "normal") {return Fault::normal;}
  if (value == "slow") {return Fault::slow;}
  if (value == "reject") {return Fault::reject;}
  if (value == "stuck") {return Fault::stuck;}
  if (value == "feedback_loss") {return Fault::feedback_loss;}
  if (value == "cancel_reject") {return Fault::cancel_reject;}
  if (value == "controller_failure") {return Fault::controller_failure;}
  throw std::invalid_argument("unknown fault mode: " + value);
}

FakeArmBackend::FakeArmBackend(std::vector<std::string> names, std::vector<double> initial,
  Fault fault, double duration, double fault_after)
: names_(std::move(names)), positions_(std::move(initial)), fault_(fault),
  duration_(duration), fault_after_(fault_after)
{
  if (names_.empty() || names_.size() != positions_.size() ||
    std::set<std::string>(names_.begin(), names_.end()).size() != names_.size() ||
    std::any_of(names_.begin(), names_.end(), [](const auto & n) {return n.empty();}) ||
    std::any_of(positions_.begin(), positions_.end(), [](double v) {return !std::isfinite(v);}) ||
    !std::isfinite(duration_) || duration_ <= 0 ||
    !std::isfinite(fault_after_) || fault_after_ < 0)
  {
    throw std::invalid_argument("invalid fake backend configuration");
  }
}

bool FakeArmBackend::execute(
  const std::vector<std::string> & names, const std::vector<double> & positions)
{
  if (names.size() != names_.size() || positions.size() != names.size() ||
    std::set<std::string>(names.begin(), names.end()).size() != names.size())
  {
    throw std::invalid_argument("target must contain every joint exactly once");
  }
  auto mapped = positions_;
  for (size_t i = 0; i < names.size(); ++i) {
    const auto found = std::find(names_.begin(), names_.end(), names[i]);
    if (found == names_.end() || !std::isfinite(positions[i])) {
      throw std::invalid_argument("unknown joint or non-finite target");
    }
    mapped[std::distance(names_.begin(), found)] = positions[i];
  }
  if (state_ == State::executing || fault_ == Fault::reject) {return false;}
  start_ = positions_;
  target_ = std::move(mapped);
  elapsed_ = 0.0;
  state_ = State::executing;
  return true;
}

void FakeArmBackend::advance(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0) {
    throw std::invalid_argument("elapsed time must be finite and non-negative");
  }
  clock_ += seconds;
  if (state_ == State::executing) {
    elapsed_ += seconds;
    const double effective = fault_ == Fault::stuck ?
      std::min(elapsed_, fault_after_) : elapsed_;
    const double duration = duration_ * (fault_ == Fault::slow ? 5.0 : 1.0);
    const double ratio = std::min(1.0, effective / duration);
    for (size_t i = 0; i < positions_.size(); ++i) {
      positions_[i] = start_[i] + (target_[i] - start_[i]) * ratio;
    }
    if (ratio >= 1.0) {state_ = State::succeeded;}
    if (fault_ == Fault::controller_failure && elapsed_ >= fault_after_) {
      state_ = State::failed;
    }
  }
  if (!(fault_ == Fault::feedback_loss && elapsed_ >= fault_after_ &&
    state_ != State::idle))
  {
    feedback_stamp_ = clock_;
  }
}

bool FakeArmBackend::cancel()
{
  if (fault_ == Fault::cancel_reject) {return false;}
  if (state_ != State::executing && state_ != State::failed) {return false;}
  state_ = State::canceled;
  return true;
}

Feedback FakeArmBackend::feedback() const
{
  const bool available = !(fault_ == Fault::feedback_loss &&
    elapsed_ >= fault_after_ && state_ != State::idle);
  return {available ? positions_ : std::vector<double>{}, feedback_stamp_, available};
}
}  // namespace zzx_execution

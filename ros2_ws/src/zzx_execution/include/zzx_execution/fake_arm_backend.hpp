// Copyright 2026 zzx
// SPDX-License-Identifier: MIT
#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace zzx_execution
{
enum class Fault {normal, slow, reject, stuck, feedback_loss, cancel_reject, controller_failure};
enum class State {idle, executing, succeeded, canceled, failed};

struct Feedback
{
  std::vector<double> positions;
  double stamp_seconds;
  bool available;
};

// Caller supplies monotonic elapsed time; identical inputs produce identical outputs.
class FakeArmBackend
{
public:
  FakeArmBackend(std::vector<std::string> names, std::vector<double> initial,
    Fault fault = Fault::normal, double duration = 1.0, double fault_after = 0.25);
  bool execute(const std::vector<std::string> & names, const std::vector<double> & positions);
  void advance(double seconds);
  bool cancel();
  Feedback feedback() const;
  State state() const {return state_;}
  const std::vector<std::string> & names() const {return names_;}

private:
  std::vector<std::string> names_;
  std::vector<double> positions_, start_, target_;
  Fault fault_;
  double duration_, fault_after_, clock_{0.0}, elapsed_{0.0}, feedback_stamp_{0.0};
  State state_{State::idle};
};
Fault parse_fault(const std::string & value);
}  // namespace zzx_execution

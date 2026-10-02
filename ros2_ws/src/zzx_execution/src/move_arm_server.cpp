// Copyright 2026 zzx
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "zzx_interfaces/action/move_arm.hpp"
#include "zzx_execution/fake_arm_backend.hpp"

namespace zzx_execution
{
using Action = zzx_interfaces::action::MoveArm;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using Error = zzx_interfaces::msg::ErrorStatus;
using Status = zzx_interfaces::msg::ExecutionStatus;
using Clock = std::chrono::steady_clock;

class MoveArmServer : public rclcpp::Node
{
public:
  MoveArmServer() : Node("move_arm_server")
  {
    const auto names = declare_parameter<std::vector<std::string>>("joint_names",
      {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    const auto initial = declare_parameter<std::vector<double>>("initial_positions",
      std::vector<double>(names.size(), 0.0));
    lower_ = declare_parameter<std::vector<double>>("lower_limits",
      std::vector<double>(names.size(), -3.141592653589793));
    upper_ = declare_parameter<std::vector<double>>("upper_limits",
      std::vector<double>(names.size(), 3.141592653589793));
    tolerance_ = declare_parameter("position_tolerance", 0.001);
    settling_ = declare_parameter("settling_seconds", 0.1);
    backend_ = std::make_unique<FakeArmBackend>(names, initial,
      parse_fault(declare_parameter("fault", std::string("normal"))),
      declare_parameter("duration_seconds", 1.0),
      declare_parameter("fault_after_seconds", 0.25));
    if (lower_.size() != names.size() || upper_.size() != names.size() ||
      !std::isfinite(tolerance_) || tolerance_ <= 0 ||
      !std::isfinite(settling_) || settling_ <= 0)
    {
      throw std::invalid_argument("invalid limits, tolerance or settling time");
    }
    for (size_t i = 0; i < names.size(); ++i) {
      if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) ||
        lower_[i] >= upper_[i] || initial[i] < lower_[i] || initial[i] > upper_[i])
      {
        throw std::invalid_argument("invalid joint limits or initial position");
      }
    }
    publisher_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    server_ = rclcpp_action::create_server<Action>(this, "zzx/manipulation/move_arm",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        if (reserved_ || !ready_) {return rclcpp_action::GoalResponse::REJECT;}
        reserved_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<Handle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<Handle> goal) {accept(goal);});
    last_tick_ = Clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "MoveArm fake backend ready; no hardware connection");
  }

private:
  sensor_msgs::msg::JointState joints()
  {
    sensor_msgs::msg::JointState result;
    const auto feedback = backend_->feedback();
    if (feedback.available) {
      result.header.stamp = now();
      result.name = backend_->names();
      result.position = feedback.positions;
    }
    return result;
  }

  std::string validate(const Action::Goal & goal)
  {
    if (goal.pose_target != geometry_msgs::msg::PoseStamped()) {
      return "joint and pose targets must not be combined";
    }
    if (!std::isfinite(goal.velocity_scaling) || goal.velocity_scaling <= 0 ||
      goal.velocity_scaling > 1 || !std::isfinite(goal.acceleration_scaling) ||
      goal.acceleration_scaling <= 0 || goal.acceleration_scaling > 1)
    {
      return "scalings must be finite and in (0, 1]";
    }
    if (goal.timeout.sec < 0 || goal.timeout.nanosec >= 1000000000u ||
      (goal.timeout.sec == 0 && goal.timeout.nanosec == 0))
    {
      return "timeout must be positive";
    }
    const auto & input = goal.joint_target;
    const auto & names = backend_->names();
    if (input.name.size() != names.size() || input.position.size() != names.size() ||
      !input.velocity.empty() || !input.effort.empty())
    {
      return "provide all joint names and positions; velocity/effort are unsupported";
    }
    target_.assign(names.size(), 0.0);
    std::unordered_set<std::string> seen;
    for (size_t i = 0; i < input.name.size(); ++i) {
      const auto found = std::find(names.begin(), names.end(), input.name[i]);
      if (found == names.end() || !seen.insert(input.name[i]).second) {
        return "unknown or duplicate joint name";
      }
      const auto index = static_cast<size_t>(found - names.begin());
      if (!std::isfinite(input.position[i]) || input.position[i] < lower_[index] ||
        input.position[i] > upper_[index])
      {
        return "nonfinite or out-of-limit joint position";
      }
      target_[index] = input.position[i];
    }
    return {};
  }

  void accept(const std::shared_ptr<Handle> & goal)
  {
    active_ = goal;
    status_ = Status();
    std::ostringstream id;
    for (const auto byte : goal->get_goal_id()) {
      id << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
    }
    status_.operation_id = id.str();
    publish(Status::STATE_VALIDATING, "validating", 0);
    const auto request = goal->get_goal();
    if (request->target_type == Action::Goal::TARGET_POSE) {
      finish(Error::UNSUPPORTED, "pose targets require a MoveIt backend");
      return;
    }
    if (request->target_type != Action::Goal::TARGET_JOINTS) {
      finish(Error::INVALID_ARGUMENT, "unknown target type");
      return;
    }
    const auto invalid = validate(*request);
    if (!invalid.empty()) {finish(Error::INVALID_ARGUMENT, invalid); return;}
    publish(Status::STATE_PLANNING, "fake_limit_check", 0);
    if (request->plan_only) {finish(Error::OK, "planned_only"); return;}
    if (!backend_->execute(backend_->names(), target_)) {
      finish(Error::BACKEND_REJECTED, "fake backend rejected execution");
      return;
    }
    started_ = last_tick_ = Clock::now();
    settled_since_ = Clock::time_point{};
    timeout_ = request->timeout.sec + request->timeout.nanosec * 1e-9;
    // Synthetic duration scaling only, not a physical velocity/acceleration controller.
    scale_ = std::min(request->velocity_scaling, std::sqrt(request->acceleration_scaling));
    publish(Status::STATE_EXECUTING, "executing", 0);
  }

  void publish(uint8_t state, const std::string & stage, float progress)
  {
    status_.header.stamp = now();
    status_.state = state;
    status_.stage = stage;
    status_.progress = progress;
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->execution = status_;
    feedback->current_joint_state = joints();
    active_->publish_feedback(feedback);
  }

  void finish(int32_t code, const std::string & message)
  {
    auto result = std::make_shared<Action::Result>();
    result->success = code == Error::OK;
    result->error.code = code;
    result->error.message = message;
    status_.header.stamp = now();
    status_.state = code == Error::OK ? Status::STATE_SUCCEEDED :
      (code == Error::CANCELED ? Status::STATE_CANCELED : Status::STATE_FAILED);
    status_.stage = message;
    status_.error = result->error;
    if (result->success) {status_.progress = 1;}
    result->execution = status_;
    result->final_joint_state = joints();
    if (result->success) {active_->succeed(result);}
    else if (code == Error::CANCELED) {active_->canceled(result);}
    else {active_->abort(result);}
    active_.reset();
    reserved_ = false;
  }

  void tick()
  {
    const auto current = Clock::now();
    const double dt = std::chrono::duration<double>(current - last_tick_).count();
    last_tick_ = current;
    backend_->advance(dt * (active_ ? scale_ : 1.0));
    if (backend_->feedback().available) {publisher_->publish(joints());}
    if (!active_) {return;}
    if (!backend_->feedback().available) {
      backend_->cancel();
      ready_ = false;
      finish(Error::STALE_DATA, "feedback lost; restart fake server to recover");
      return;
    }
    if (active_->is_canceling()) {
      backend_->cancel();
      finish(Error::CANCELED, "fake execution canceled");
      return;
    }
    if (std::chrono::duration<double>(current - started_).count() >= timeout_) {
      backend_->cancel();
      finish(Error::TIMEOUT, "execution deadline exceeded");
      return;
    }
    const auto feedback = backend_->feedback();
    double max_error = 0;
    for (size_t i = 0; i < target_.size(); ++i) {
      max_error = std::max(max_error, std::abs(target_[i] - feedback.positions[i]));
    }
    if (backend_->state() == State::succeeded && max_error <= tolerance_) {
      if (settled_since_ == Clock::time_point{}) {settled_since_ = current;}
      publish(Status::STATE_SETTLING, "settling", 0.95f);
      if (std::chrono::duration<double>(current - settled_since_).count() >= settling_) {
        finish(Error::OK, "target reached and settled");
      }
    } else {
      settled_since_ = Clock::time_point{};
      publish(Status::STATE_EXECUTING, "executing", 0.5f);
    }
  }

  std::unique_ptr<FakeArmBackend> backend_;
  std::vector<double> lower_, upper_, target_;
  double tolerance_, settling_, timeout_{0}, scale_{1};
  bool reserved_{false}, ready_{true};
  Clock::time_point last_tick_, started_, settled_since_;
  Status status_;
  std::shared_ptr<Handle> active_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace zzx_execution

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<zzx_execution::MoveArmServer>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("move_arm_server"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

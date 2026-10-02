// Copyright 2026 zzx
// SPDX-License-Identifier: MIT
#include "zzx_execution/fake_arm_backend.hpp"

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

class FakeArmNode : public rclcpp::Node
{
public:
  FakeArmNode() : Node("fake_arm_backend")
  {
    auto names = declare_parameter<std::vector<std::string>>("joint_names",
      {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    auto initial = declare_parameter<std::vector<double>>("initial_positions", std::vector<double>(6, 0));
    backend_ = std::make_unique<zzx_execution::FakeArmBackend>(names, initial,
      zzx_execution::parse_fault(declare_parameter<std::string>("fault", "normal")),
      declare_parameter<double>("duration_seconds", 1.0),
      declare_parameter<double>("fault_after_seconds", 0.25));
    publisher_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>("~/target", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr target) {
        try {
          if (!target->velocity.empty() || !target->effort.empty()) {
            throw std::invalid_argument("fake target accepts positions only");
          }
          if (!backend_->execute(target->name, target->position)) {
            RCLCPP_WARN(get_logger(), "Target rejected: busy or injected rejection");
          }
        } catch (const std::invalid_argument & error) {
          RCLCPP_WARN(get_logger(), "Invalid target: %s", error.what());
        }
      });
    cancel_ = create_service<std_srvs::srv::Trigger>("~/cancel",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = backend_->cancel();
        response->message = response->success ? "Fake motion canceled" : "No active fake motion";
      });
    last_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
      const auto current = std::chrono::steady_clock::now();
      backend_->advance(std::chrono::duration<double>(current - last_).count());
      last_ = current;
      const auto feedback = backend_->feedback();
      if (feedback.available) {
        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        message.name = backend_->names();
        message.position = feedback.positions;
        publisher_->publish(message);
      }
    });
    RCLCPP_INFO(get_logger(), "Fake backend ready; targets only change in-memory joints");
  }

private:
  std::unique_ptr<zzx_execution::FakeArmBackend> backend_;
  std::chrono::steady_clock::time_point last_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FakeArmNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("fake_arm_backend"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

// Copyright 2026 zzx
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gtest/gtest.h>

#include <type_traits>

#include "zzx_interfaces/action/control_hand.hpp"
#include "zzx_interfaces/action/move_arm.hpp"
#include "zzx_interfaces/msg/backend_capabilities.hpp"
#include "zzx_interfaces/msg/error_status.hpp"
#include "zzx_interfaces/msg/execution_status.hpp"
#include "zzx_interfaces/msg/object_target.hpp"
#include "zzx_interfaces/msg/task_command.hpp"

TEST(InterfaceTypes, SharedStatusIsAvailableToCpp)
{
  zzx_interfaces::msg::ExecutionStatus status;
  status.operation_id = "operation-1";
  status.state = zzx_interfaces::msg::ExecutionStatus::STATE_SUCCEEDED;
  status.progress = 1.0F;
  status.error.code = zzx_interfaces::msg::ErrorStatus::OK;

  EXPECT_EQ(status.error.code, 0);
  EXPECT_FLOAT_EQ(status.progress, 1.0F);
}

TEST(InterfaceTypes, PerceptionProvenanceIsAvailableToCpp)
{
  zzx_interfaces::msg::ObjectTarget target;
  target.object_id = "block-1";
  target.image_width = 1280;
  target.image_height = 720;
  target.position_valid = true;
  target.orientation_valid = false;
  target.calibration_id = "gemini335-1280x720-v1";
  target.quality.method_id = "roi_median_depth_v1";

  EXPECT_EQ(target.image_width, 1280U);
  EXPECT_TRUE(target.position_valid);
  EXPECT_FALSE(target.orientation_valid);
}

TEST(InterfaceTypes, ActionGoalsUseGeneratedTypes)
{
  zzx_interfaces::action::MoveArm::Goal arm_goal;
  arm_goal.target_type = zzx_interfaces::action::MoveArm::Goal::TARGET_JOINTS;
  arm_goal.joint_target.name = {"joint2", "joint1"};
  arm_goal.joint_target.position = {0.2, -0.1};

  zzx_interfaces::action::ControlHand::Goal hand_goal;
  hand_goal.position_unit =
    zzx_interfaces::action::ControlHand::Goal::UNIT_NORMALIZED;
  hand_goal.joint_names = {"index_pip"};
  hand_goal.positions = {0.5};

  EXPECT_EQ(arm_goal.joint_target.name.size(), 2U);
  EXPECT_EQ(hand_goal.positions.size(), 1U);
  static_assert(
    std::is_same_v<decltype(arm_goal.velocity_scaling), double>,
    "MoveArm scaling must remain float64");
}

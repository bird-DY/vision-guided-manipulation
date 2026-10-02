// Copyright 2026 zzx
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <limits>
#include "zzx_execution/fake_arm_backend.hpp"

using zzx_execution::FakeArmBackend;
using zzx_execution::Fault;
using zzx_execution::State;

TEST(FakeArm, StartupNeverMoves)
{
  FakeArmBackend arm({"a", "b"}, {0.1, 0.2});
  arm.advance(100);
  EXPECT_EQ(arm.state(), State::idle);
  EXPECT_EQ(arm.feedback().positions, (std::vector<double>{0.1, 0.2}));
}

TEST(FakeArm, MapsNamesAndCompletes)
{
  FakeArmBackend arm({"a", "b"}, {0, 0});
  ASSERT_TRUE(arm.execute({"b", "a"}, {2, 1}));
  arm.advance(0.5);
  EXPECT_EQ(arm.feedback().positions, (std::vector<double>{0.5, 1}));
  EXPECT_EQ(arm.state(), State::executing);
  arm.advance(0.5);
  EXPECT_EQ(arm.state(), State::succeeded);
}

TEST(FakeArm, RejectDoesNotMutate)
{
  FakeArmBackend arm({"a"}, {0}, Fault::reject);
  EXPECT_FALSE(arm.execute({"a"}, {1}));
  arm.advance(2);
  EXPECT_EQ(arm.feedback().positions[0], 0);
  EXPECT_EQ(arm.state(), State::idle);
}

TEST(FakeArm, SlowTakesFiveTimesLonger)
{
  FakeArmBackend arm({"a"}, {0}, Fault::slow);
  arm.execute({"a"}, {1});
  arm.advance(1);
  EXPECT_DOUBLE_EQ(arm.feedback().positions[0], 0.2);
  EXPECT_EQ(arm.state(), State::executing);
  arm.advance(4);
  EXPECT_EQ(arm.state(), State::succeeded);
}

TEST(FakeArm, StuckNeverCompletesAndCanCancel)
{
  FakeArmBackend arm({"a"}, {0}, Fault::stuck);
  arm.execute({"a"}, {1});
  arm.advance(10);
  EXPECT_DOUBLE_EQ(arm.feedback().positions[0], 0.25);
  EXPECT_EQ(arm.state(), State::executing);
  EXPECT_TRUE(arm.cancel());
  arm.advance(20);
  EXPECT_DOUBLE_EQ(arm.feedback().positions[0], 0.25);
  EXPECT_EQ(arm.state(), State::canceled);
}

TEST(FakeArm, FeedbackLossDoesNotExposeHiddenState)
{
  FakeArmBackend arm({"a"}, {0}, Fault::feedback_loss);
  arm.execute({"a"}, {1});
  arm.advance(0.1);
  ASSERT_TRUE(arm.feedback().available);
  const auto stamp = arm.feedback().stamp_seconds;
  arm.advance(2);
  EXPECT_FALSE(arm.feedback().available);
  EXPECT_TRUE(arm.feedback().positions.empty());
  EXPECT_DOUBLE_EQ(arm.feedback().stamp_seconds, stamp);
}

TEST(FakeArm, BusyCannotReplaceActiveGoal)
{
  FakeArmBackend arm({"a"}, {0});
  arm.execute({"a"}, {1});
  EXPECT_FALSE(arm.execute({"a"}, {5}));
  arm.advance(1);
  EXPECT_DOUBLE_EQ(arm.feedback().positions[0], 1);
}

TEST(FakeArm, InvalidInputsLeaveStateUnchanged)
{
  FakeArmBackend arm({"a", "b"}, {0, 0});
  EXPECT_THROW(arm.execute({"a", "a"}, {1, 2}), std::invalid_argument);
  EXPECT_THROW(arm.execute({"a", "c"}, {1, 2}), std::invalid_argument);
  EXPECT_THROW(arm.execute({"a"}, {1}), std::invalid_argument);
  EXPECT_THROW(arm.execute({"a", "b"},
    {std::numeric_limits<double>::quiet_NaN(), 0}), std::invalid_argument);
  EXPECT_THROW(arm.advance(-1), std::invalid_argument);
  EXPECT_EQ(arm.state(), State::idle);
}

TEST(FakeArm, ReplayIsDeterministic)
{
  for (auto fault : {Fault::normal, Fault::slow, Fault::stuck, Fault::feedback_loss}) {
    FakeArmBackend first({"a"}, {0}, fault), second({"a"}, {0}, fault);
    first.execute({"a"}, {1});
    second.execute({"a"}, {1});
    for (int i = 0; i < 100; ++i) {
      first.advance(0.02);
      second.advance(0.02);
      EXPECT_EQ(first.feedback().positions, second.feedback().positions);
      EXPECT_EQ(first.feedback().available, second.feedback().available);
      EXPECT_EQ(first.state(), second.state());
    }
  }
}

TEST(FakeArm, ConfigurationRejectsInvalidValues)
{
  EXPECT_THROW(FakeArmBackend({"a"}, {0}, Fault::normal, 0), std::invalid_argument);
  EXPECT_THROW(FakeArmBackend({"a", "a"}, {0, 0}), std::invalid_argument);
  EXPECT_THROW(zzx_execution::parse_fault("typo"), std::invalid_argument);
}

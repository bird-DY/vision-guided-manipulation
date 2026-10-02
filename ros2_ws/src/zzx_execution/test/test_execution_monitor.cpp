// Copyright 2026 zzx
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "zzx_execution/execution_monitor.hpp"

using namespace zzx_execution;
using C = ControllerResult;
using V = Verdict;

static ExecutionMonitor monitor(bool continuous = false)
{
  MonitorConfig config;
  config.continuous = {continuous};
  return ExecutionMonitor(config);
}

TEST(ExecutionMonitor, RequiresNewStableSamplesAndControllerSuccess)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  EXPECT_EQ(m.update(0, 0, {1}, true, C::running), V::running);
  EXPECT_EQ(m.update(.05, .05, {1}, true, C::running), V::running);
  EXPECT_EQ(m.update(.1, .1, {1}, true, C::succeeded), V::settling);
  EXPECT_EQ(m.update(.19, .19, {1}, true, C::succeeded), V::settling);
  EXPECT_EQ(m.update(.21, .21, {1}, true, C::succeeded), V::succeeded);
}

TEST(ExecutionMonitor, ControllerSuccessDoesNotHidePositionError)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  m.update(0, 0, {0}, true, C::succeeded);
  EXPECT_EQ(m.update(.15, .15, {0}, true, C::succeeded), V::running);
}

TEST(ExecutionMonitor, SettlingResetsOnMotion)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  m.update(0, 0, {1}, true, C::succeeded);
  m.update(.05, .05, {1}, true, C::succeeded);
  EXPECT_EQ(m.update(.1, .1, {.99}, true, C::succeeded), V::running);
  EXPECT_EQ(m.update(.15, .15, {1}, true, C::succeeded), V::running);
  EXPECT_EQ(m.update(.2, .2, {1}, true, C::succeeded), V::settling);
  EXPECT_EQ(m.update(.31, .31, {1}, true, C::succeeded), V::succeeded);
}

TEST(ExecutionMonitor, ReplayCannotCompleteAndEventuallyStales)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  m.update(0, 0, {1}, true, C::succeeded);
  m.update(.05, .05, {1}, true, C::succeeded);
  EXPECT_EQ(m.update(.19, .05, {1}, true, C::succeeded), V::running);
  EXPECT_EQ(m.update(.26, .05, {1}, true, C::succeeded), V::stale);
}

TEST(ExecutionMonitor, SampleGapRestartsSettling)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  m.update(0, 0, {1}, true, C::succeeded);
  m.update(.05, .05, {1}, true, C::succeeded);
  EXPECT_EQ(m.update(.5, .5, {1}, true, C::succeeded), V::running);
}

TEST(ExecutionMonitor, InvalidFeedbackCannotSucceed)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  EXPECT_EQ(m.update(.1, .2, {1}, true, C::succeeded), V::stale);
  EXPECT_EQ(m.update(.2, .2, {}, true, C::succeeded), V::stale);
  EXPECT_EQ(m.update(.3, .3, {std::numeric_limits<double>::quiet_NaN()}, true,
    C::succeeded), V::stale);
  EXPECT_EQ(m.update(.4, .4, {1}, false, C::succeeded), V::stale);
}

TEST(ExecutionMonitor, ContinuousWrapUsesShortestError)
{
  const double pi = std::acos(-1.0);
  EXPECT_NEAR(ExecutionMonitor::difference(-pi + .0001, pi - .0001, true), .0002, 1e-10);
  auto m = monitor(true);
  m.start({-pi + .0001}, 0, 2);
  m.update(0, 0, {pi - .0001}, true, C::succeeded);
  EXPECT_EQ(m.update(.05, .05, {-pi + .0001}, true, C::succeeded), V::settling);
  EXPECT_EQ(m.update(.16, .16, {-pi + .0001}, true, C::succeeded), V::succeeded);
  EXPECT_GT(std::abs(ExecutionMonitor::difference(-pi, pi, false)), 6);
}

TEST(ExecutionMonitor, FailureAndDeadlineTakePriority)
{
  auto m = monitor();
  m.start({1}, 0, .2);
  EXPECT_EQ(m.update(.1, .1, {1}, true, C::failed), V::failed);
  EXPECT_EQ(m.update(.2, .2, {1}, true, C::succeeded), V::timeout);
}

TEST(ExecutionMonitor, CancelNeedsFreshStoppedFeedback)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  m.update(.1, .1, {.5}, true, C::running);
  m.request_stop(.1);
  EXPECT_EQ(m.update(.12, .12, {.5}, true, C::canceled), V::stopping);
  EXPECT_EQ(m.update(.15, .15, {.5}, true, C::canceled), V::stopping);
  EXPECT_EQ(m.update(.26, .26, {.5}, true, C::canceled), V::stopped);
}

TEST(ExecutionMonitor, CancelWithoutFeedbackIsUnconfirmed)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  m.request_stop(.1);
  EXPECT_EQ(m.update(.2, .2, {}, false, C::canceled), V::stopping);
  EXPECT_EQ(m.update(.61, .61, {}, false, C::canceled), V::stop_unconfirmed);
}

TEST(ExecutionMonitor, RepeatedCancelDoesNotExtendDeadline)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  m.request_stop(.1);
  m.update(.2, .2, {.5}, true, C::running);
  m.request_stop(.4);
  EXPECT_EQ(m.update(.61, .61, {.5}, true, C::running), V::stop_unconfirmed);
}

TEST(ExecutionMonitor, InvalidConfigurationAndClockAreRejected)
{
  MonitorConfig config;
  config.continuous = {false};
  config.feedback_timeout = -1;
  EXPECT_THROW(ExecutionMonitor{config}, std::invalid_argument);
  auto m = monitor();
  EXPECT_THROW(m.start({}, 0, 2), std::invalid_argument);
  m.start({1}, 1, 2);
  EXPECT_THROW(m.update(.5, .5, {1}, true, C::succeeded), std::invalid_argument);
}

TEST(ExecutionMonitor, WithinPositionToleranceButMovingIsNotSettled)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  m.update(0, 0, {1}, true, C::succeeded);
  EXPECT_EQ(m.update(.02, .02, {1.0008}, true, C::succeeded), V::running);
  EXPECT_EQ(m.update(.04, .04, {.9992}, true, C::succeeded), V::running);
}

TEST(ExecutionMonitor, CancellationDuringSettlingCannotSucceed)
{
  auto m = monitor();
  m.start({1}, 0, 2);
  m.update(0, 0, {1}, true, C::succeeded);
  m.update(.02, .02, {1}, true, C::succeeded);
  m.request_stop(.03);
  EXPECT_EQ(m.update(.04, .04, {1}, true, C::succeeded), V::stopping);
  EXPECT_EQ(m.update(.06, .06, {1}, true, C::succeeded), V::stopping);
  EXPECT_EQ(m.update(.17, .17, {1}, true, C::succeeded), V::stopped);
}

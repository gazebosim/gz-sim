/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#include <thread>

#include "ignition/math/Stopwatch.hh"

using namespace ignition;

/////////////////////////////////////////////////
// Helper function that runs a few tests
void runTimer(math::Stopwatch &_time)
{
  // Windows uses a system_clock for std::this_thread::sleep_for. This can
  // cause incorrect sleep durations. So, we add some room for error on
  // windows.
  std::chrono::duration<int, std::milli> handleSteadyClock =
    std::chrono::milliseconds(0);
#ifdef _WIN32
  handleSteadyClock += std::chrono::milliseconds(100);
#endif

  // Start the timer
  EXPECT_TRUE(_time.Start());
  // The timer should be running.
  EXPECT_TRUE(_time.Running());
  // The start time should be greater than the stop time.
  EXPECT_GT(_time.StartTime(), _time.StopTime());
  // The elapsed stop time should still be zero.
  EXPECT_EQ(ignition::math::clock::duration::zero(),
            _time.ElapsedStopTime());

  // Wait for some time...
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // Now the elapsed time should be greater than or equal to the time slept.
  EXPECT_GE(_time.ElapsedRunTime() + handleSteadyClock,
      std::chrono::milliseconds(1000));

  // Stop the timer.
  EXPECT_TRUE(_time.Stop());
  // The timer should not be running.
  EXPECT_FALSE(_time.Running());
  // The stop time should be greater than the start time.
  EXPECT_GT(_time.StopTime(), _time.StartTime());
  // The elapsed time should still be greater than the time slept.
  EXPECT_GE(_time.ElapsedRunTime() + handleSteadyClock,
      std::chrono::milliseconds(1000));

  // Save the elapsed time.
  auto elapsedTime = _time.ElapsedRunTime();

  // The timer is now stopped, let's sleep some more.
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // The elapsed stop time should be greater than or equal to the time
  // slept.
  EXPECT_GE(_time.ElapsedStopTime() + handleSteadyClock,
      std::chrono::milliseconds(1000));
  // The elapsed time should be the same.
  EXPECT_EQ(elapsedTime, _time.ElapsedRunTime());

  // Start the timer again.
  EXPECT_TRUE(_time.Start());
  // Store the elapsed stop time.
  auto elapsedStopTime = _time.ElapsedStopTime();
  // The timer should be running.
  EXPECT_TRUE(_time.Running());
  // Sleep for some time.
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // The elapsed stop time should remain the same
  EXPECT_EQ(elapsedStopTime, _time.ElapsedStopTime());
  // The elapsed time should be greater than the previous elapsed time.
  EXPECT_GT(_time.ElapsedRunTime(), elapsedTime);
  // The elapsed time should be greater than or equal to the the previous
  // two sleep times.
  EXPECT_GE(_time.ElapsedRunTime() + handleSteadyClock,
      std::chrono::milliseconds(2000));
}

/////////////////////////////////////////////////
TEST(Stopwatch, Constructor)
{
  math::Stopwatch watch;

  EXPECT_FALSE(watch.Running());
  EXPECT_EQ(watch.StopTime(), watch.StartTime());
  EXPECT_EQ(ignition::math::clock::duration::zero(), watch.ElapsedRunTime());
  EXPECT_EQ(ignition::math::clock::duration::zero(), watch.ElapsedStopTime());

  runTimer(watch);

  math::Stopwatch watch2(watch);
  EXPECT_EQ(watch, watch2);

  math::Stopwatch watch3(std::move(watch2));
  EXPECT_EQ(watch, watch3);
}

/////////////////////////////////////////////////
TEST(Stopwatch, EqualOperator)
{
  math::Stopwatch watch;
  math::Stopwatch watch2;
  math::Stopwatch watch3;
  EXPECT_EQ(watch, watch2);
  EXPECT_EQ(watch, watch3);

  runTimer(watch);
  runTimer(watch2);
  runTimer(watch3);

  EXPECT_NE(watch, watch2);
  EXPECT_NE(watch, watch3);

  watch2 = watch;
  EXPECT_EQ(watch, watch2);

  watch3 = std::move(watch2);
  EXPECT_EQ(watch, watch3);
}

/////////////////////////////////////////////////
TEST(Stopwatch, StartStopReset)
{
  math::Stopwatch watch;

  runTimer(watch);

  watch.Reset();

  EXPECT_FALSE(watch.Running());
  EXPECT_EQ(watch.StopTime(), watch.StartTime());
  EXPECT_EQ(ignition::math::clock::duration::zero(), watch.ElapsedRunTime());
  EXPECT_EQ(ignition::math::clock::duration::zero(), watch.ElapsedStopTime());

  runTimer(watch);

  EXPECT_TRUE(watch.Running());

  watch.Start(true);
  EXPECT_TRUE(watch.Running());
  EXPECT_LT(watch.StopTime(), watch.StartTime());
  EXPECT_NE(ignition::math::clock::duration::zero(), watch.ElapsedRunTime());
  EXPECT_EQ(ignition::math::clock::duration::zero(), watch.ElapsedStopTime());
}

/////////////////////////////////////////////////
TEST(Stopwatch, FailStartStop)
{
  math::Stopwatch watch;

  // Can't stop while not running
  EXPECT_FALSE(watch.Stop());
  EXPECT_FALSE(watch.Running());

  // Can start while not running
  EXPECT_TRUE(watch.Start());
  EXPECT_TRUE(watch.Running());

  // Can't start while running
  EXPECT_FALSE(watch.Start());
  EXPECT_TRUE(watch.Running());

  // Can stop while running
  EXPECT_TRUE(watch.Stop());
  EXPECT_FALSE(watch.Running());

  // Can start while not running
  EXPECT_TRUE(watch.Start());
  EXPECT_TRUE(watch.Running());
}

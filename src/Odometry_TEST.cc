/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "ignition/math/Angle.hh"
#include "ignition/math/Helpers.hh"
#include "ignition/math/Odometry.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(OdometryTest, Odometry)
{
  math::Odometry odom;
  EXPECT_DOUBLE_EQ(0.0, odom.Heading());
  EXPECT_DOUBLE_EQ(0.0, odom.X());
  EXPECT_DOUBLE_EQ(0.0, odom.Y());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearVelocity());
  EXPECT_DOUBLE_EQ(0.0, odom.AngularVelocity());

  odom.SetWheelParams(2.0, 0.5, 0.5);
  odom.Init(std::chrono::steady_clock::now());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  odom.Update(IGN_DTOR(1.0), IGN_DTOR(1.0), std::chrono::steady_clock::now());
  EXPECT_DOUBLE_EQ(0.0, odom.Heading());
  EXPECT_NEAR(0.008726, odom.X(), 1e-6);
  EXPECT_DOUBLE_EQ(0.0, odom.Y());

}

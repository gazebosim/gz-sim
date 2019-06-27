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

#include "ignition/math/Helpers.hh"
#include "ignition/math/RollingMean.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(OdometryFourWheelTest, OdometryFourWheel)
{
  math::OdometryFourWheel odom;
  EXPECT_DOUBLE_EQ(0.0, odom.Heading());
  EXPECT_DOUBLE_EQ(0.0, odom.X());
  EXPECT_DOUBLE_EQ(0.0, odom.Y());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearVelocity());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearVelocityX());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearVelocityY());
  EXPECT_DOUBLE_EQ(0.0, odom.AngularVelocity());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearAcceleration());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearJerk());
  EXPECT_DOUBLE_EQ(0.0, odom.FrontSteerVelocity());
  EXPECT_DOUBLE_EQ(0.0, odom.RearSteerVelocity());
}

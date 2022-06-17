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

#include "gz/math/Angle.hh"
#include "gz/math/Helpers.hh"
#include "gz/math/DiffDriveOdometry.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(DiffDriveOdometryTest, DiffDriveOdometry)
{
  math::DiffDriveOdometry odom;
  EXPECT_DOUBLE_EQ(0.0, *odom.Heading());
  EXPECT_DOUBLE_EQ(0.0, odom.X());
  EXPECT_DOUBLE_EQ(0.0, odom.Y());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearVelocity());
  EXPECT_DOUBLE_EQ(0.0, *odom.AngularVelocity());
  EXPECT_FALSE(odom.Initialized());

  double wheelSeparation = 2.0;
  double wheelRadius = 0.5;
  double wheelCircumference = 2 * GZ_PI * wheelRadius;

  // This is the linear distance traveled per degree of wheel rotation.
  double distPerDegree = wheelCircumference / 360.0;

  // Setup the wheel parameters, and initialize
  odom.SetWheelParams(wheelSeparation, wheelRadius, wheelRadius);
  auto startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);
  EXPECT_TRUE(odom.Initialized());

  // Expect false if time difference is too small
  EXPECT_FALSE(odom.Update(0.0, 0.0, startTime));

  // Sleep for a little while, then update the odometry with the new wheel
  // position.
  auto time1 = startTime + std::chrono::milliseconds(100);
  EXPECT_TRUE(odom.Update(GZ_DTOR(1.0), GZ_DTOR(1.0), time1));
  EXPECT_DOUBLE_EQ(0.0, *odom.Heading());
  EXPECT_DOUBLE_EQ(distPerDegree, odom.X());
  EXPECT_DOUBLE_EQ(0.0, odom.Y());
  // Linear velocity should be dist_traveled / time_elapsed.
  EXPECT_NEAR(distPerDegree / 0.1, odom.LinearVelocity(), 1e-3);
  // Angular velocity should be zero since the "robot" is traveling in a
  // straight line.
  EXPECT_NEAR(0.0, *odom.AngularVelocity(), 1e-3);

  // Sleep again, then update the odometry with the new wheel position.
  auto time2 = time1 + std::chrono::milliseconds(100);
  EXPECT_TRUE(odom.Update(GZ_DTOR(2.0), GZ_DTOR(2.0), time2));
  EXPECT_DOUBLE_EQ(0.0, *odom.Heading());
  EXPECT_NEAR(distPerDegree * 2.0, odom.X(), 3e-6);
  EXPECT_DOUBLE_EQ(0.0, odom.Y());
  // Linear velocity should be dist_traveled / time_elapsed.
  EXPECT_NEAR(distPerDegree / 0.1, odom.LinearVelocity(), 1e-3);
  // Angular velocity should be zero since the "robot" is traveling in a
  // straight line.
  EXPECT_NEAR(0.0, *odom.AngularVelocity(), 1e-3);

  // Initialize again, and odom values should be reset.
  startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);
  EXPECT_DOUBLE_EQ(0.0, *odom.Heading());
  EXPECT_DOUBLE_EQ(0.0, odom.X());
  EXPECT_DOUBLE_EQ(0.0, odom.Y());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearVelocity());
  EXPECT_DOUBLE_EQ(0.0, *odom.AngularVelocity());
  EXPECT_TRUE(odom.Initialized());

  // Sleep again, this time move 2 degrees in 100ms.
  time1 = startTime + std::chrono::milliseconds(100);
  EXPECT_TRUE(odom.Update(GZ_DTOR(2.0), GZ_DTOR(2.0), time1));
  EXPECT_DOUBLE_EQ(0.0, *odom.Heading());
  EXPECT_NEAR(distPerDegree * 2.0, odom.X(), 3e-6);
  EXPECT_DOUBLE_EQ(0.0, odom.Y());
  // Linear velocity should be dist_traveled / time_elapsed.
  EXPECT_NEAR(distPerDegree * 2 / 0.1, odom.LinearVelocity(), 1e-3);
  // Angular velocity should be zero since the "robot" is traveling in a
  // straight line.
  EXPECT_NEAR(0.0, *odom.AngularVelocity(), 1e-3);


  // Sleep again, this time rotate the right wheel by 1 degree.
  time2 = time1 + std::chrono::milliseconds(100);
  EXPECT_TRUE(odom.Update(GZ_DTOR(2.0), GZ_DTOR(3.0), time2));
  // The heading should be the arc tangent of the linear distance traveled
  // by the right wheel (the left wheel was stationary) divided by the
  // wheel separation.
  EXPECT_NEAR(atan2(distPerDegree, wheelSeparation), *odom.Heading(), 1e-6);


  // The X odom reading should have increased by the sine of the heading *
  // half the wheel separation.
  double xDistTraveled =
    sin(atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5;
  double prevXPos = distPerDegree * 2.0;
  EXPECT_NEAR(xDistTraveled + prevXPos, odom.X(), 3e-6);

  // The Y odom reading should have increased by the cosine of the header *
  // half the wheel separation.
  double yDistTraveled = (wheelSeparation * 0.5) -
      cos(atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5;
  double prevYPos = 0.0;
  EXPECT_NEAR(yDistTraveled + prevYPos, odom.Y(), 3e-6);

  // Angular velocity should be the difference between the x and y distance
  // traveled divided by the wheel separation divided by the seconds
  // elapsed.
  EXPECT_NEAR(
      ((xDistTraveled - yDistTraveled) / wheelSeparation) / 0.1,
      *odom.AngularVelocity(), 1e-3);
}

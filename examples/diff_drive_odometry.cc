/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
//! [complete]
#include <iostream>
#include <chrono>

#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/DiffDriveOdometry.hh>

int main(int argc, char **argv)
{

//! [Create a DiffDriveOdometry]
  gz::math::DiffDriveOdometry odom;
//! [Create an DiffDriveOdometry]

  double wheelSeparation = 2.0;
  double wheelRadius = 0.5;
  double wheelCircumference = 2 * GZ_PI * wheelRadius;

  // This is the linear distance traveled per degree of wheel rotation.
  double distPerDegree = wheelCircumference / 360.0;

  // Setup the wheel parameters, and initialize
  odom.SetWheelParams(wheelSeparation, wheelRadius, wheelRadius);
  auto startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);

  // Sleep for a little while, then update the odometry with the new wheel
  // position.
  std::cout << "--- Rotate both wheels by 1 degree. ---" << '\n';
  auto time1 = startTime + std::chrono::milliseconds(100);
  odom.Update(GZ_DTOR(1.0), GZ_DTOR(1.0), time1);

  std::cout << "\tLinear velocity:\t" << distPerDegree / 0.1 << " m/s"
            << "\n\tOdom linear velocity:\t" << odom.LinearVelocity() << " m/s"
            << std::endl;

  std::cout << "Angular velocity should be zero since the \"robot\" is traveling\n"
            << "in a straight line:\n"
            << "\tOdom angular velocity:\t"
            << *odom.AngularVelocity() << " rad/s" << std::endl;

  // Sleep again, this time rotate the left wheel by 1 and the right wheel by 2
  // degrees.
  std::cout << "--- This time rotate the left wheel by 1 and the right wheel "
            << "by 2 degrees ---"
            << std::endl;
  auto time2 = time1 + std::chrono::milliseconds(100);
  odom.Update(GZ_DTOR(2.0), GZ_DTOR(3.0), time2);

  std::cout << "The heading should be the arc tangent of the linear distance\n"
            << "traveled by the right wheel (the left wheel was stationary)\n"
            << "divided by the wheel separation.\n"
            << "\tHeading:\t\t" << atan2(distPerDegree, wheelSeparation) << " rad"
            << "\n\tOdom Heading:\t\t" << *odom.Heading() << " rad" << '\n';

  // The X odom reading should have increased by the sine of the heading *
  // half the wheel separation.
  double xDistTraveled =
    sin(atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5;
  double prevXPos = distPerDegree * 2.0;
  std::cout << "\tX distance traveled:\t" << xDistTraveled + prevXPos << " m"
             << "\n\tOdom X:\t\t" << odom.X() << " m" << std::endl;

  // The Y odom reading should have increased by the cosine of the heading *
  // half the wheel separation.
  double yDistTraveled = (wheelSeparation * 0.5) -
      cos(atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5;
  double prevYPos = 0.0;
  std::cout << "\tY distance traveled:\t" << yDistTraveled + prevYPos << " m"
             << "\n\tOdom Y:\t\t" << odom.Y() << " m" << std::endl;

  std::cout << "Angular velocity should be the difference between the x and y\n"
            << "distance traveled divided by the wheel separation divided by\n"
            << "the seconds elapsed.\n"
            << "\tAngular velocity:\t"
            << ((xDistTraveled - yDistTraveled) / wheelSeparation) / 0.1 << " rad/s"
            << "\n\tOdom angular velocity:\t" << *odom.AngularVelocity() << " rad/s"
            << std::endl;
}
//! [complete]

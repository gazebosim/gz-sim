/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include "TransformTypes.hh"

using namespace gz;
using namespace sim;

TEST(TransformationTypes, transformFrame)
{
  // Create a vehicle rotated 90 degrees in the yaw axis
  math::Pose3d pose(math::Vector3d(0, 0, 0), math::Quaterniond(0, 0, GZ_PI_2));

  // Vehicle should be moving at 1m/s along x-axis (In global frame)
  math::Vector3d velocity(1, 0, 0);

  // Create a current moving against the vehicle
  math::Vector3d current(-3, 0, 0);

  // Test transforms
  // Global transformFrame should keep things unaffected.
  auto res = transformFrame(TransformType::GLOBAL, pose, velocity, current);
  EXPECT_EQ(res, current);

  res = transformFrame(
    TransformType::ADD_VELOCITY_GLOBAL, pose, velocity, current);
  EXPECT_EQ(res, math::Vector3d(-4, 0, 0));

  // Tranforming to local frame without accounting for velocity. Considering
  // robot is facing Y axis.
  // Top down view:
  //   vvvvvvvvv    Current direction
  //       ^ (Global X, Robot -Y, Robot Direction of motion)
  //       |
  //       |                                       Robot coords
  //  ----------->  (Global Y, Robot +X heading)   --->+x
  //       |                                       |
  //       |                                       v +y
  //
  res = transformFrame(TransformType::LOCAL, pose, velocity, current);
  EXPECT_LT((res - math::Vector3d(0, 3, 0)).Length(), 1e-5);

  // Tranforming with vehicle's velocity in account
  res = transformFrame(
    TransformType::ADD_VELOCITY_LOCAL, pose, velocity, current);
  EXPECT_LT((res - math::Vector3d(0, 4, 0)).Length(), 1e-5);
}

TEST(TransformationTypes, getTransformType)
{
  EXPECT_EQ(
    getTransformType("ADD_VELOCITY_LOCAL"), TransformType::ADD_VELOCITY_LOCAL);
  EXPECT_EQ(
    getTransformType("ADD_VELOCITY_GLOBAL"),
    TransformType::ADD_VELOCITY_GLOBAL);
  EXPECT_EQ(getTransformType("LOCAL"), TransformType::LOCAL);
  EXPECT_EQ(getTransformType("GLOBAL"), TransformType::GLOBAL);
  EXPECT_EQ(getTransformType("nonsense"), std::nullopt);
}

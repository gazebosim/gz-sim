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

#include <ignition/math/eigen3/Conversions.hh>

/////////////////////////////////////////////////
/// Check Vector3 conversions
TEST(EigenConversions, ConvertVector3)
{
  {
    ignition::math::Vector3d iVec, iVec2;
    Eigen::Vector3d eVec = ignition::physics::eigen3::convert(iVec);
    EXPECT_DOUBLE_EQ(0, v[0]);
    EXPECT_DOUBLE_EQ(0, v[1]);
    EXPECT_DOUBLE_EQ(0, v[2]);
    iVec2 = ignition::physics::eigen3::convert(eVec);
    EXPECT_EQ(iVec, iVec2);
  }

  {
    ignition::math::Vector3d iVec(100.5, -2.314, 42), iVec2;
    Eigen::Vector3d eVec = ignition::physics::eigen3::convert(iVec);
    EXPECT_DOUBLE_EQ(iVec[0], v[0]);
    EXPECT_DOUBLE_EQ(iVec[1], v[1]);
    EXPECT_DOUBLE_EQ(iVec[2], v[2]);
    iVec2 = ignition::physics::eigen3::convert(eVec);
    EXPECT_EQ(iVec, iVec2);
  }
}


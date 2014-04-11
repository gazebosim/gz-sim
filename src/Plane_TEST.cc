/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "ignition/math/Planed.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(PlaneTest, PlaneConstructor)
{
  math::Planed plane(math::Vector3d(1, 0, 0), 0.1);
  EXPECT_EQ(plane.GetNormal(), math::Vector3d(1, 0, 0));
  EXPECT_NEAR(plane.GetOffset(), 0.1, 1e-6);
}

/////////////////////////////////////////////////
TEST(PlaneTest, Distance)
{
  math::Planed plane(math::Vector3d(0, 0, 1), 0.1);
  EXPECT_NEAR(plane.Distance(math::Vector3d(0, 0, 0),
              math::Vector3d(0, 0, 1)), 0.1, 1e-6);

  EXPECT_NEAR(plane.Distance(math::Vector3d(0, 0, 0.1),
              math::Vector3d(0, 0, 1)), 0, 1e-6);

  EXPECT_NEAR(plane.Distance(math::Vector3d(0, 0, 0.2),
              math::Vector3d(0, 0, 1)), -0.1, 1e-6);

  EXPECT_NEAR(plane.Distance(math::Vector3d(0, 0, 0.1),
              math::Vector3d(1, 0, 0)), 0, 1e-6);
}

/////////////////////////////////////////////////
TEST(PlaneTest, Plane)
{
  {
    math::Planed plane;
    EXPECT_TRUE(math::equal(plane.GetOffset(), 0.0));
    EXPECT_TRUE(plane.GetNormal() == math::Vector3d());
    EXPECT_TRUE(plane.GetSize() == math::Vector2d(0, 0));
  }

  {
    math::Planed plane(math::Vector3d(0, 0, 1), math::Vector2d(2, 3), 2.0);
    EXPECT_TRUE(math::equal(plane.GetOffset(), 2.0));
    EXPECT_TRUE(plane.GetNormal() == math::Vector3d(0, 0, 1));
    EXPECT_TRUE(plane.GetSize() == math::Vector2d(2, 3));

    EXPECT_DOUBLE_EQ(-1, plane.Distance(math::Vector3d(0, 0, 1),
          math::Vector3d(0, 0, -1)));

    plane.Set(math::Vector3d(1, 0, 0), math::Vector2d(1, 1), 1.0);
    EXPECT_TRUE(math::equal(plane.GetOffset(), 1.0));
    EXPECT_TRUE(plane.GetNormal() == math::Vector3d(1, 0, 0));
    EXPECT_TRUE(plane.GetSize() == math::Vector2d(1, 1));

    plane = math::Planed(math::Vector3d(0, 1, 0), math::Vector2d(4, 4), 5.0);
    EXPECT_TRUE(math::equal(plane.GetOffset(), 5.0));
    EXPECT_TRUE(plane.GetNormal() == math::Vector3d(0, 1, 0));
    EXPECT_TRUE(plane.GetSize() == math::Vector2d(4, 4));
  }
}

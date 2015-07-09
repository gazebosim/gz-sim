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
#include "ignition/math/Plane.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(PlaneTest, PlaneConstructor)
{
  math::Planed plane(math::Vector3d(1, 0, 0), 0.1);
  EXPECT_EQ(plane.Normal(), math::Vector3d(1, 0, 0));
  EXPECT_NEAR(plane.Offset(), 0.1, 1e-6);
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
    EXPECT_TRUE(math::equal(plane.Offset(), 0.0));
    EXPECT_TRUE(plane.Normal() == math::Vector3d());
    EXPECT_TRUE(plane.Size() == math::Vector2d(0, 0));
  }

  {
    math::Planed plane(math::Vector3d(0, 0, 1), math::Vector2d(2, 3), 2.0);
    EXPECT_TRUE(math::equal(plane.Offset(), 2.0));
    EXPECT_TRUE(plane.Normal() == math::Vector3d(0, 0, 1));
    EXPECT_TRUE(plane.Size() == math::Vector2d(2, 3));

    EXPECT_DOUBLE_EQ(-1, plane.Distance(math::Vector3d(0, 0, 1),
          math::Vector3d(0, 0, -1)));

    plane.Set(math::Vector3d(1, 0, 0), math::Vector2d(1, 1), 1.0);
    EXPECT_TRUE(math::equal(plane.Offset(), 1.0));
    EXPECT_TRUE(plane.Normal() == math::Vector3d(1, 0, 0));
    EXPECT_TRUE(plane.Size() == math::Vector2d(1, 1));

    plane = math::Planed(math::Vector3d(0, 1, 0), math::Vector2d(4, 4), 5.0);
    EXPECT_TRUE(math::equal(plane.Offset(), 5.0));
    EXPECT_TRUE(plane.Normal() == math::Vector3d(0, 1, 0));
    EXPECT_TRUE(plane.Size() == math::Vector2d(4, 4));
  }
}

/////////////////////////////////////////////////
TEST(PlaneTest, SidePoint)
{
  math::Planed plane(math::Vector3d(0, 0, 1), 1);

  // On the negative side of the plane (below the plane)
  math::Vector3d point(0, 0, 0);
  EXPECT_EQ(plane.Side(point), math::Planed::NEGATIVE_SIDE);

  // Still on the negative side of the plane (below the plane)
  point.Set(1, 1, 0);
  EXPECT_EQ(plane.Side(point), math::Planed::NEGATIVE_SIDE);

  // Above the plane (positive side)
  point.Set(1, 1, 2);
  EXPECT_EQ(plane.Side(point), math::Planed::POSITIVE_SIDE);

  // On the plane
  point.Set(0, 0, 1);
  EXPECT_EQ(plane.Side(point), math::Planed::NO_SIDE);

  // Change the plane, but the point is still on the negative side
  plane.Set(math::Vector3d(1, 0, 0), 4);
  EXPECT_EQ(plane.Side(point), math::Planed::NEGATIVE_SIDE);

  // Point is now on the positive side
  point.Set(4.1, 0, 1);
  EXPECT_EQ(plane.Side(point), math::Planed::POSITIVE_SIDE);
}

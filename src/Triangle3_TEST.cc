/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "ignition/math/Triangle3.hh"
#include "ignition/math/Helpers.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(Triangle3Test, Constructor)
{
  {
    // Default constructor
    Triangle3d tri;
    EXPECT_EQ(tri[0], Vector3d(0, 0, 0));
    EXPECT_EQ(tri[1], Vector3d(0, 0, 0));
    EXPECT_EQ(tri[2], Vector3d(0, 0, 0));
    EXPECT_FALSE(tri.Valid());
  }

  {
    // Construct from three points
    Triangle3d tri(Vector3d(0, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, 0));

    EXPECT_TRUE(tri.Valid());

    EXPECT_EQ(tri[0], Vector3d(0, 0, 0));
    EXPECT_EQ(tri[1], Vector3d(0, 1, 0));
    EXPECT_EQ(tri[2], Vector3d(1, 0, 0));
    EXPECT_NO_THROW(tri[3]);
    EXPECT_EQ(tri[3], tri[2]);
  }

  {
    // Construct degenerate from 3 collinear points
    Triangle3d tri(Vector3d(0, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 2, 0));

    // Expect not Valid
    EXPECT_FALSE(tri.Valid());
  }
}

/////////////////////////////////////////////////
TEST(Triangle3Test, Set)
{
  Triangle3d tri;

  tri.Set(0, Vector3d(3, 4, 1));
  tri.Set(1, Vector3d(5, -6, 2));
  tri.Set(2, Vector3d(7, 8, -3));
  EXPECT_EQ(tri[0], Vector3d(3, 4, 1));
  EXPECT_EQ(tri[1], Vector3d(5, -6, 2));
  EXPECT_EQ(tri[2], Vector3d(7, 8, -3));

  tri.Set(Vector3d(0.1, 0.2, -0.3),
          Vector3d(0.3, -0.4, 0.5),
          Vector3d(1.5, -2.6, 3.7));
  EXPECT_EQ(tri[0], Vector3d(0.1, 0.2, -0.3));
  EXPECT_EQ(tri[1], Vector3d(0.3, -0.4, 0.5));
  EXPECT_EQ(tri[2], Vector3d(1.5, -2.6, 3.7));

  EXPECT_NO_THROW(tri.Set(3, Vector3d(1.5, 2.6, 3.8)));
  EXPECT_EQ(tri[3], Vector3d(1.5, 2.6, 3.8));
}

/////////////////////////////////////////////////
TEST(Triangle3Test, Side)
{
  Triangle3d tri(Vector3d(0, 0, 0), Vector3d(0, 1, 1), Vector3d(1, 0, 2));

  EXPECT_TRUE(tri.Side(0) == Line3d(0, 0, 0, 0, 1, 1));
  EXPECT_TRUE(tri.Side(1) == Line3d(0, 1, 1, 1, 0, 2));
  EXPECT_TRUE(tri.Side(2) == Line3d(1, 0, 2, 0, 0, 0));

  EXPECT_NO_THROW(tri.Side(3));
  EXPECT_TRUE(tri.Side(3) == tri.Side(2));
}

/////////////////////////////////////////////////
TEST(Triangle3Test, ContainsLine)
{
  Triangle3d tri(Vector3d(0, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, 0));

  EXPECT_TRUE(tri.Contains(tri.Side(0)));
  EXPECT_TRUE(tri.Contains(tri.Side(1)));
  EXPECT_TRUE(tri.Contains(tri.Side(2)));

  EXPECT_TRUE(tri.Contains(Line3d(0.1, 0.1, 0, 0.5, 0.5, 0)));

  EXPECT_FALSE(tri.Contains(Line3d(0.1, 0.1, 0, 0.6, 0.6, 0)));
  EXPECT_FALSE(tri.Contains(Line3d(-0.1, -0.1, 0, 0.5, 0.5, 0)));
  EXPECT_FALSE(tri.Contains(Line3d(0.1, 0.1, 0.1, 0.5, 0.5, 0.1)));
  EXPECT_FALSE(tri.Contains(Line3d(0.1, 0.1, -0.1, 0.1, 0.1, 0.1)));
}

/////////////////////////////////////////////////
TEST(Triangle3Test, Intersects)
{
  Vector3d pt1;
  Triangle3d tri(Vector3d(0, 0, 0),
                 Vector3d(0, 1, 0),
                 Vector3d(1, 0, 0));

  EXPECT_TRUE(tri.Intersects(tri.Side(0), pt1));
  EXPECT_EQ(pt1, Vector3d(0, 0, 0));

  EXPECT_TRUE(tri.Intersects(tri.Side(1), pt1));
  EXPECT_EQ(pt1, Vector3d(0, 1, 0));

  EXPECT_TRUE(tri.Intersects(tri.Side(2), pt1));
  EXPECT_EQ(pt1, Vector3d(1, 0, 0));

  EXPECT_TRUE(tri.Intersects(Line3d(0.1, 0.1, 0, 0.5, 0.5, 0), pt1));
  EXPECT_EQ(pt1, Vector3d(0.1, 0.1, 0));

  EXPECT_TRUE(tri.Intersects(Line3d(0.1, 0.1, 0, 0.6, 0.6, 0), pt1));
  EXPECT_EQ(pt1, Vector3d(0.5, 0.5, 0));

  EXPECT_TRUE(tri.Intersects(Line3d(0.6, 0.6, 0, 0.1, 0.1, 0), pt1));
  EXPECT_EQ(pt1, Vector3d(0.5, 0.5, 0));

  EXPECT_TRUE(tri.Intersects(Line3d(0.1, 0.1, 0, -0.6, 0.1, 0), pt1));
  EXPECT_EQ(pt1, Vector3d(0.0, 0.1, 0));

  EXPECT_TRUE(tri.Intersects(Line3d(0.1, 0.1, 0, 0.1, -0.1, 0), pt1));
  EXPECT_EQ(pt1, Vector3d(0.1, 0, 0));

  EXPECT_TRUE(tri.Intersects(Line3d(-0.1, -0.1, 0, 0.5, 0.5, 0), pt1));
  EXPECT_EQ(pt1, Vector3d(0.0, 0.0, 0));

  EXPECT_TRUE(tri.Intersects(Line3d(-2, -2, 0, 0.2, 0.2, 0), pt1));
  EXPECT_EQ(pt1, Vector3d(0.0, 0.0, 0));

  EXPECT_TRUE(tri.Intersects(Line3d(0.1, 0.1, -1, 0.1, 0.1, 1), pt1));
  EXPECT_EQ(pt1, Vector3d(0.1, 0.1, 0));

  EXPECT_TRUE(tri.Intersects(Line3d(0.1, 0.1, -1, 0.3, 0.3, 1), pt1));
  EXPECT_EQ(pt1, Vector3d(0.2, 0.2, 0));

  EXPECT_FALSE(tri.Intersects(Line3d(-0.1, 0, 0, -0.1, 1, 0), pt1));

  // A flat triangle raised along the z-axis
  {
    tri.Set(Vector3d(0, 0, 0.5),
        Vector3d(0, 1, 0.5),
        Vector3d(1, 0, 0.5));
    EXPECT_TRUE(tri.Intersects(Line3d(0.5, 0.5, 2, 0.5, 0.5, 0), pt1));
    EXPECT_EQ(pt1, Vector3d(0.5, 0.5, 0.5));
  }

  // An angled triangle
  {
    tri.Set(Vector3d(0, 0, 1),
        Vector3d(1, 0, 0),
        Vector3d(0, 1, 0));

    EXPECT_TRUE(tri.Intersects(Line3d(1, 0, 2, 1, 0, 0), pt1));
    EXPECT_EQ(pt1, Vector3d(1, 0, 0));

    EXPECT_FALSE(tri.Intersects(Line3d(1.1, 0, 2, 1.1, 0, 0), pt1));
    EXPECT_FALSE(tri.Intersects(Line3d(0, 1.1, 2, 0, 1.1, 0), pt1));
    EXPECT_FALSE(tri.Intersects(Line3d(-0.1, 0, 2, -0.1, 0, 0), pt1));
    EXPECT_FALSE(tri.Intersects(Line3d(0.51, 0.51, 2, 0.51, 0.51, 0), pt1));

    EXPECT_TRUE(tri.Intersects(Line3d(0, 1, 2, 0, 1, 0), pt1));
    EXPECT_EQ(pt1, Vector3d(0, 1, 0));

    EXPECT_TRUE(tri.Intersects(Line3d(0, 0, 2, 0, 0, 0), pt1));
    EXPECT_EQ(pt1, Vector3d(0, 0, 1.0));

    EXPECT_TRUE(tri.Intersects(Line3d(0.1, 0.1, 2, 0.1, 0.1, 0), pt1));
    EXPECT_EQ(pt1, Vector3d(0.1, 0.1, 0.8));

    EXPECT_TRUE(tri.Intersects(Line3d(0.2, 0.2, 2, 0.2, 0.2, 0), pt1));
    EXPECT_EQ(pt1, Vector3d(0.2, 0.2, 0.6));

    EXPECT_TRUE(tri.Intersects(Line3d(0.3, 0.3, 2, 0.3, 0.3, 0), pt1));
    EXPECT_EQ(pt1, Vector3d(0.3, 0.3, 0.4));

    EXPECT_TRUE(tri.Intersects(Line3d(0.4, 0.4, 2, 0.4, 0.4, 0), pt1));
    EXPECT_EQ(pt1, Vector3d(0.4, 0.4, 0.2));

    EXPECT_TRUE(tri.Intersects(Line3d(0.5, 0.5, 2, 0.5, 0.5, 0), pt1));
    EXPECT_EQ(pt1, Vector3d(0.5, 0.5, 0));
  }
}

/////////////////////////////////////////////////
TEST(Triangle3Test, ContainsPt)
{
  Triangle3d tri(Vector3d(0, 0, 0),
                 Vector3d(0, 1, 0),
                 Vector3d(1, 0, 0));

  EXPECT_TRUE(tri.Contains(tri[0]));
  EXPECT_TRUE(tri.Contains(tri[1]));
  EXPECT_TRUE(tri.Contains(tri[2]));

  EXPECT_TRUE(tri.Contains(Vector3d(0.1, 0.1, 0)));
  EXPECT_TRUE(tri.Contains(Vector3d(0, 0.5, 0)));
  EXPECT_TRUE(tri.Contains(Vector3d(0.5, 0, 0)));
  EXPECT_TRUE(tri.Contains(Vector3d(0.5, 0.5, 0)));

  EXPECT_FALSE(tri.Contains(Vector3d(-0.01, -0.01, 0)));
  EXPECT_FALSE(tri.Contains(Vector3d(1.01, 0, 0)));
  EXPECT_FALSE(tri.Contains(Vector3d(0, 1.01, 0)));
  EXPECT_FALSE(tri.Contains(Vector3d(0.1, 0.1, 0.1)));
  EXPECT_FALSE(tri.Contains(Vector3d(0.1, 0.1, -0.1)));
}

/////////////////////////////////////////////////
TEST(Triangle3Test, Perimeter)
{
  {
    Triangle3d tri(Vector3d(0, 0, 0),
                   Vector3d(0, 1, 0),
                   Vector3d(1, 0, 0));

    EXPECT_DOUBLE_EQ(tri.Perimeter(), 2.0 + sqrt(2.0));
  }
  {
    Triangle3d tri(Vector3d(0, 0, 1),
                   Vector3d(0, 1, 0),
                   Vector3d(1, 0, 0));

    EXPECT_DOUBLE_EQ(tri.Perimeter(), 3*sqrt(2.0));
  }
}

/////////////////////////////////////////////////
TEST(Triangle3Test, Area)
{
  {
    Triangle3d tri(Vector3d(0, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, 0));

    EXPECT_NEAR(tri.Area(), 0.5, 1e-6);
  }
  {
    Triangle3d tri(Vector3d(0, 0, 1), Vector3d(0, 1, 0), Vector3d(1, 0, 0));

    // (base * height) / 2
    EXPECT_NEAR(tri.Area(), (sqrt(2) * sqrt(1.5))*0.5, 1e-6);
  }
}

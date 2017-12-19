/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "ignition/math/Triangle.hh"
#include "ignition/math/Helpers.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(TriangleTest, Constructor)
{
  {
    // Default constructor
    math::Triangled tri;
    EXPECT_EQ(tri[0], math::Vector2d(0, 0));
    EXPECT_EQ(tri[1], math::Vector2d(0, 0));
    EXPECT_EQ(tri[2], math::Vector2d(0, 0));
  }

  {
    // Construct from three points
    math::Triangled tri(math::Vector2d(0, 0),
                         math::Vector2d(0, 1),
                         math::Vector2d(1, 0));

    EXPECT_TRUE(tri.Valid());

    EXPECT_EQ(tri[0], math::Vector2d(0, 0));
    EXPECT_EQ(tri[1], math::Vector2d(0, 1));
    EXPECT_EQ(tri[2], math::Vector2d(1, 0));
    EXPECT_NO_THROW(tri[3]);
    EXPECT_EQ(tri[3], tri[2]);
  }

  {
    // Construct degenerate from 3 collinear points
    math::Triangled tri(math::Vector2d(0, 0),
                         math::Vector2d(0, 1),
                         math::Vector2d(0, 2));

    // Expect not Valid
    EXPECT_FALSE(tri.Valid());
  }
}

/////////////////////////////////////////////////
TEST(TriangleTest, Set)
{
  math::Triangled tri;

  tri.Set(0, math::Vector2d(3, 4));
  tri.Set(1, math::Vector2d(5, 6));
  tri.Set(2, math::Vector2d(7, 8));
  EXPECT_EQ(tri[0], math::Vector2d(3, 4));
  EXPECT_EQ(tri[1], math::Vector2d(5, 6));
  EXPECT_EQ(tri[2], math::Vector2d(7, 8));

  tri.Set(math::Vector2d(0.1, 0.2),
          math::Vector2d(0.3, 0.4),
          math::Vector2d(1.5, 2.6));
  EXPECT_EQ(tri[0], math::Vector2d(0.1, 0.2));
  EXPECT_EQ(tri[1], math::Vector2d(0.3, 0.4));
  EXPECT_EQ(tri[2], math::Vector2d(1.5, 2.6));

  EXPECT_NO_THROW(tri.Set(3, math::Vector2d(1.5, 2.6)));
}

/////////////////////////////////////////////////
TEST(TriangleTest, Side)
{
  math::Triangled tri(math::Vector2d(0, 0),
                      math::Vector2d(0, 1),
                      math::Vector2d(1, 0));

  EXPECT_TRUE(tri.Side(0) == math::Line2d(0, 0, 0, 1));
  EXPECT_TRUE(tri.Side(1) == math::Line2d(0, 1, 1, 0));
  EXPECT_TRUE(tri.Side(2) == math::Line2d(1, 0, 0, 0));

  EXPECT_NO_THROW(tri.Side(3));
}

/////////////////////////////////////////////////
TEST(TriangleTest, ContainsLine)
{
  math::Triangled tri(math::Vector2d(0, 0),
                      math::Vector2d(0, 1),
                      math::Vector2d(1, 0));

  EXPECT_TRUE(tri.Contains(tri.Side(0)));
  EXPECT_TRUE(tri.Contains(tri.Side(1)));
  EXPECT_TRUE(tri.Contains(tri.Side(2)));

  EXPECT_TRUE(tri.Contains(math::Line2d(0.1, 0.1, 0.5, 0.5)));

  EXPECT_FALSE(tri.Contains(math::Line2d(0.1, 0.1, 0.6, 0.6)));
  EXPECT_FALSE(tri.Contains(math::Line2d(-0.1, -0.1, 0.5, 0.5)));
}

/////////////////////////////////////////////////
TEST(TriangleTest, Intersects)
{
  math::Vector2d pt1, pt2;
  math::Triangled tri(math::Vector2d(0, 0),
                      math::Vector2d(0, 1),
                      math::Vector2d(1, 0));

  EXPECT_TRUE(tri.Intersects(tri.Side(0), pt1, pt2));
  EXPECT_EQ(pt1, math::Vector2d(0, 0));
  EXPECT_EQ(pt2, math::Vector2d(0, 1));

  EXPECT_TRUE(tri.Intersects(tri.Side(1), pt1, pt2));
  EXPECT_EQ(pt1, math::Vector2d(0, 1));
  EXPECT_EQ(pt2, math::Vector2d(1, 0));

  EXPECT_TRUE(tri.Intersects(tri.Side(2), pt1, pt2));
  EXPECT_EQ(pt1, math::Vector2d(1, 0));
  EXPECT_EQ(pt2, math::Vector2d(0, 0));


  EXPECT_TRUE(tri.Intersects(math::Line2d(0.1, 0.1, 0.5, 0.5), pt1, pt2));
  EXPECT_EQ(pt1, math::Vector2d(0.1, 0.1));
  EXPECT_EQ(pt2, math::Vector2d(0.5, 0.5));

  EXPECT_TRUE(tri.Intersects(math::Line2d(0.1, 0.1, 0.6, 0.6), pt1, pt2));
  EXPECT_EQ(pt1, math::Vector2d(0.5, 0.5));
  EXPECT_EQ(pt2, math::Vector2d(0.1, 0.1));

  EXPECT_TRUE(tri.Intersects(math::Line2d(-0.1, -0.1, 0.5, 0.5), pt1, pt2));
  EXPECT_EQ(pt1, math::Vector2d(0.0, 0.0));
  EXPECT_EQ(pt2, math::Vector2d(0.5, 0.5));

  EXPECT_TRUE(tri.Intersects(math::Line2d(-2, -2, 0.2, 0.2), pt1, pt2));
  EXPECT_EQ(pt1, math::Vector2d(0.0, 0.0));
  EXPECT_EQ(pt2, math::Vector2d(0.2, 0.2));

  EXPECT_FALSE(tri.Intersects(math::Line2d(-0.1, 0, -0.1, 1), pt1, pt2));
}

/////////////////////////////////////////////////
TEST(TriangleTest, ContainsPt)
{
  math::Triangled tri(math::Vector2d(0, 0),
                      math::Vector2d(0, 1),
                      math::Vector2d(1, 0));

  EXPECT_TRUE(tri.Contains(tri[0]));
  EXPECT_TRUE(tri.Contains(tri[1]));
  EXPECT_TRUE(tri.Contains(tri[2]));

  EXPECT_TRUE(tri.Contains(math::Vector2d(0.1, 0.1)));
  EXPECT_TRUE(tri.Contains(math::Vector2d(0, 0.5)));
  EXPECT_TRUE(tri.Contains(math::Vector2d(0.5, 0)));
  EXPECT_TRUE(tri.Contains(math::Vector2d(0.5, 0.5)));

  EXPECT_FALSE(tri.Contains(math::Vector2d(-0.01, -0.01)));
  EXPECT_FALSE(tri.Contains(math::Vector2d(1.01, 0)));
  EXPECT_FALSE(tri.Contains(math::Vector2d(0, 1.01)));
}

/////////////////////////////////////////////////
TEST(TriangleTest, Perimeter)
{
  math::Triangled tri(math::Vector2d(0, 0),
                      math::Vector2d(0, 1),
                      math::Vector2d(1, 0));

  EXPECT_DOUBLE_EQ(tri.Perimeter(), 2.0 + sqrt(2.0));
}

/////////////////////////////////////////////////
TEST(TriangleTest, Area)
{
  math::Triangled tri(math::Vector2d(0, 0),
                      math::Vector2d(0, 1),
                      math::Vector2d(1, 0));

  EXPECT_NEAR(tri.Area(), 0.499999, 1e-6);
}

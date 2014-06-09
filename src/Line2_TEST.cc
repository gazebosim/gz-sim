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

#include "ignition/math/Line2.hh"
#include "ignition/math/Helpers.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Line2Test, Constructor)
{
  math::Line2d lineA(0, 0, 10, 10);
  EXPECT_DOUBLE_EQ(lineA[0].x(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[0].y(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[1].x(), 10.0);
  EXPECT_DOUBLE_EQ(lineA[1].y(), 10.0);

  math::Line2d lineB(math::Vector2d(1, 2), math::Vector2d(3, 4));
  EXPECT_DOUBLE_EQ(lineB[0].x(), 1.0);
  EXPECT_DOUBLE_EQ(lineB[0].y(), 2.0);
  EXPECT_DOUBLE_EQ(lineB[1].x(), 3.0);
  EXPECT_DOUBLE_EQ(lineB[1].y(), 4.0);

  EXPECT_THROW(lineB[2].x(), math::IndexException);
  EXPECT_NO_THROW(lineA[0].x());
}

/////////////////////////////////////////////////
TEST(Line2Test, Length)
{
  math::Line2d lineA(0, 0, 10, 10);
  EXPECT_NEAR(lineA.Length(), sqrt(200), 1e-10);
}

/////////////////////////////////////////////////
TEST(Line2Test, Slope)
{
  math::Line2d lineA(0, 0, 10, 10);
  EXPECT_NEAR(lineA.Slope(), 1.0, 1e-10);

  math::Line2d lineB(0, 0, 0, 10);
  EXPECT_TRUE(math::isnan(lineB.Slope()));

  math::Line2d lineC(-10, 0, 100, 0);
  EXPECT_EQ(lineC.Slope(), 0.0);
}

/////////////////////////////////////////////////
TEST(Line2Test, Intersect)
{
  math::Vector2d pt;

  // Parallel horizontal lines
  math::Line2d lineA(1, 1, 2, 1);
  math::Line2d lineB(1, 2, 2, 2);
  EXPECT_FALSE(lineA.Intersect(lineB, pt));

  // Parallel vertical lines
  lineA.Set(1, 1, 1, 10);
  lineB.Set(2, 1, 2, 10);
  EXPECT_FALSE(lineA.Intersect(lineB, pt));

  // Two lines that form an inverted T with a gap
  lineA.Set(1, 1, 1, 10);
  lineB.Set(0, 0, 2, 0);
  EXPECT_FALSE(lineA.Intersect(lineB, pt));

  // Two lines that form a T with a gap
  lineA.Set(1, 1, 1, 10);
  lineB.Set(0, 10.1, 2, 10.1);
  EXPECT_FALSE(lineA.Intersect(lineB, pt));

  // Two lines that form an inverted T with a gap
  lineA.Set(0, -10, 0, 10);
  lineB.Set(1, 0, 10, 0);
  EXPECT_FALSE(lineA.Intersect(lineB, pt));

  // Two lines that form a T with a gap
  lineA.Set(0, -10, 0, 10);
  lineB.Set(-1, 0, -10, 0);
  EXPECT_FALSE(lineA.Intersect(lineB, pt));

  // Two co-linear lines, one starts where the other stopped
  lineA.Set(1, 1, 1, 10);
  lineB.Set(1, 10, 1, 11);
  EXPECT_TRUE(lineA.Intersect(lineB, pt));
  EXPECT_EQ(pt, math::Vector2d(1, 10));

  // Two co-linear lines, one overlaps the other
  lineA.Set(0, 0, 0, 10);
  lineB.Set(0, 9, 0, 11);
  EXPECT_TRUE(lineA.Intersect(lineB, pt));
  EXPECT_EQ(pt, math::Vector2d(0, 9));

  // Two co-linear lines, one overlaps the other
  lineA.Set(0, 0, 0, 10);
  lineB.Set(0, -10, 0, 1);
  EXPECT_TRUE(lineA.Intersect(lineB, pt));
  EXPECT_EQ(pt, math::Vector2d(0, 1));

  // Two intersecting lines
  lineA.Set(0, 0, 10, 10);
  lineB.Set(0, 10, 10, 0);
  EXPECT_TRUE(lineA.Intersect(lineB, pt));
  EXPECT_EQ(pt, math::Vector2d(5, 5));
}

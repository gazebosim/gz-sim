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
  EXPECT_DOUBLE_EQ(lineA[0].X(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[0].Y(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[1].X(), 10.0);
  EXPECT_DOUBLE_EQ(lineA[1].Y(), 10.0);

  math::Line2d lineB(math::Vector2d(1, 2), math::Vector2d(3, 4));
  EXPECT_DOUBLE_EQ(lineB[0].X(), 1.0);
  EXPECT_DOUBLE_EQ(lineB[0].Y(), 2.0);
  EXPECT_DOUBLE_EQ(lineB[1].X(), 3.0);
  EXPECT_DOUBLE_EQ(lineB[1].Y(), 4.0);

  EXPECT_NO_THROW(lineB[2].X());
  EXPECT_DOUBLE_EQ(lineB[2].X(), lineB[1].X());
  EXPECT_NO_THROW(lineA[0].X());
}

/////////////////////////////////////////////////
TEST(Line2Test, Length)
{
  math::Line2d lineA(0, 0, 10, 10);
  EXPECT_NEAR(lineA.Length(), sqrt(200), 1e-10);
}

#ifdef _MSC_VER
#pragma warning(push)
// C4723: potential divide by 0
#pragma warning(disable : 4723)
#endif
/////////////////////////////////////////////////
TEST(Line2Test, Slope)
{
  {
    math::Line2d line(0, 0, 10, 10);
    EXPECT_NEAR(line.Slope(), 1.0, 1e-10);
  }

  {
    math::Line2d line(0, 0, 0, 10);
    EXPECT_TRUE(math::isnan(line.Slope()));
  }

  {
    math::Line2d line(-10, 0, 100, 0);
    EXPECT_DOUBLE_EQ(line.Slope(), 0.0);
  }
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif

/////////////////////////////////////////////////
TEST(Line2Test, ParallelLine)
{
  {
    // Line is always parallel with itself
    math::Line2d line(0, 0, 10, 0);
    EXPECT_TRUE(line.Parallel(line, 1e-10));
  }

  {
    // Degenerate line segment
    // Still expect Line is parallel with itself
    math::Line2d line(0, 0, 0, 0);
    EXPECT_TRUE(line.Parallel(line, 1e-10));
  }

  math::Line2d lineA(0, 0, 10, 0);
  math::Line2d lineB(0, 0, 10, 0);
  EXPECT_TRUE(lineA.Parallel(lineB, 1e-10));

  lineB.Set(0, 0, 0, 10);
  EXPECT_FALSE(lineA.Parallel(lineB));

  lineB.Set(0, 10, 10, 10);
  EXPECT_TRUE(lineA.Parallel(lineB));

  lineB.Set(0, 10, 10, 10.00001);
  EXPECT_FALSE(lineA.Parallel(lineB, 1e-10));
  EXPECT_FALSE(lineA.Parallel(lineB));
  EXPECT_TRUE(lineA.Parallel(lineB, 1e-3));
}

/////////////////////////////////////////////////
TEST(Line2Test, CollinearLine)
{
  {
    // Line is always collinear with itself
    math::Line2d line(0, 0, 10, 0);
    EXPECT_TRUE(line.Collinear(line, 1e-10));
  }

  math::Line2d lineA(0, 0, 10, 0);
  math::Line2d lineB(0, 0, 10, 0);
  EXPECT_TRUE(lineA.Collinear(lineB, 1e-10));

  lineB.Set(0, 10, 10, 10);
  EXPECT_FALSE(lineA.Collinear(lineB));

  lineB.Set(9, 0, 10, 0.00001);
  EXPECT_FALSE(lineA.Collinear(lineB, 1e-10));
  EXPECT_FALSE(lineA.Collinear(lineB));
  EXPECT_TRUE(lineA.Collinear(lineB, 1e-3));
}

/////////////////////////////////////////////////
TEST(Line2Test, CollinearPoint)
{
  math::Line2d lineA(0, 0, 10, 0);
  math::Vector2d pt(0, 0);
  EXPECT_TRUE(lineA.Collinear(pt));
  {
    math::Line2d ptLine(pt, pt);
    EXPECT_TRUE(lineA.Collinear(ptLine));
  }

  pt.Set(1000, 0);
  EXPECT_TRUE(lineA.Collinear(pt, 1e-10));
  {
    math::Line2d ptLine(pt, pt);
    EXPECT_TRUE(lineA.Parallel(ptLine));
    EXPECT_FALSE(lineA.Intersect(ptLine));
    EXPECT_FALSE(lineA.Collinear(ptLine, 1e-10));

    pt.Set(10, 0);
    ptLine.Set(pt, pt);
    EXPECT_TRUE(lineA.Collinear(ptLine, 1e-10));
  }

  pt.Set(0, 0.00001);
  EXPECT_FALSE(lineA.Collinear(pt));
  EXPECT_TRUE(lineA.Collinear(pt, 1e-3));
  {
    math::Line2d ptLine(pt, pt);
    EXPECT_FALSE(lineA.Collinear(ptLine));
    EXPECT_TRUE(lineA.Parallel(ptLine));
    EXPECT_FALSE(lineA.Intersect(ptLine));
    EXPECT_TRUE(lineA.Intersect(ptLine, 1e-2));
    EXPECT_TRUE(lineA.Collinear(ptLine, 1e-3));
  }

  pt.Set(0, -0.00001);
  EXPECT_FALSE(lineA.Collinear(pt));
  EXPECT_TRUE(lineA.Collinear(pt, 1e-3));
  {
    math::Line2d ptLine(pt, pt);
    EXPECT_FALSE(lineA.Collinear(ptLine));
    EXPECT_TRUE(lineA.Collinear(ptLine, 1e-4));
  }
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

  // Two collinear lines, one starts where the other stopped
  lineA.Set(1, 1, 1, 10);
  lineB.Set(1, 10, 1, 11);
  EXPECT_TRUE(lineA.Intersect(lineB, pt));
  EXPECT_EQ(pt, math::Vector2d(1, 10));

  // Two collinear lines, one overlaps the other
  lineA.Set(0, 0, 0, 10);
  lineB.Set(0, 9, 0, 11);
  EXPECT_TRUE(lineA.Intersect(lineB, pt));
  EXPECT_EQ(pt, math::Vector2d(0, 9));

  // Two collinear lines, one overlaps the other
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

/////////////////////////////////////////////////
TEST(Line2Test, Equality)
{
  math::Line2d lineA(1, 1, 2, 1);
  math::Line2d lineB(1, 2, 2, 2);

  EXPECT_TRUE(lineA != lineB);
  EXPECT_TRUE(lineA == lineA);

  lineB.Set(1, 1, 2, 1.1);
  EXPECT_FALSE(lineA == lineB);

  lineB.Set(1, 1, 2.1, 1);
  EXPECT_FALSE(lineA == lineB);

  lineB.Set(1, 1.1, 2, 1);
  EXPECT_FALSE(lineA == lineB);

  lineB.Set(1.1, 1, 2, 1);
  EXPECT_FALSE(lineA == lineB);
}

/////////////////////////////////////////////////
TEST(Line2Test, OperatorStreamOut)
{
  math::Line2d line(0, 1, 2, 3);
  std::ostringstream stream;
  stream << line;
  EXPECT_EQ(stream.str(), "0 1 2 3");
}

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

#include "ignition/math/Line3.hh"
#include "ignition/math/Helpers.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Line3Test, Constructor)
{
  math::Line3d lineA(0, 0, 10, 10);
  EXPECT_DOUBLE_EQ(lineA[0].X(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[0].Y(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[0].Z(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[1].X(), 10.0);
  EXPECT_DOUBLE_EQ(lineA[1].Y(), 10.0);
  EXPECT_DOUBLE_EQ(lineA[1].Z(), 0.0);

  math::Line3d lineB(math::Vector3d(1, 2, 3), math::Vector3d(4, 5, 6));
  EXPECT_DOUBLE_EQ(lineB[0].X(), 1.0);
  EXPECT_DOUBLE_EQ(lineB[0].Y(), 2.0);
  EXPECT_DOUBLE_EQ(lineB[0].Z(), 3.0);
  EXPECT_DOUBLE_EQ(lineB[1].X(), 4.0);
  EXPECT_DOUBLE_EQ(lineB[1].Y(), 5.0);
  EXPECT_DOUBLE_EQ(lineB[1].Z(), 6.0);

  math::Line3d lineC(0, 0, 5, 10, 10, 6);
  EXPECT_DOUBLE_EQ(lineC[0].X(), 0.0);
  EXPECT_DOUBLE_EQ(lineC[0].Y(), 0.0);
  EXPECT_DOUBLE_EQ(lineC[0].Z(), 5.0);
  EXPECT_DOUBLE_EQ(lineC[1].X(), 10.0);
  EXPECT_DOUBLE_EQ(lineC[1].Y(), 10.0);
  EXPECT_DOUBLE_EQ(lineC[1].Z(), 6.0);

  EXPECT_THROW(lineB[2].X(), math::IndexException);
  EXPECT_NO_THROW(lineA[0].X());
}

TEST(Line3Test, Set)
{
  math::Line3d lineA;
  lineA.Set(1, 1, 2, 2);
  EXPECT_DOUBLE_EQ(lineA[0].X(), 1.0);
  EXPECT_DOUBLE_EQ(lineA[0].Y(), 1.0);
  EXPECT_DOUBLE_EQ(lineA[0].Z(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[1].X(), 2.0);
  EXPECT_DOUBLE_EQ(lineA[1].Y(), 2.0);
  EXPECT_DOUBLE_EQ(lineA[1].Z(), 0.0);

  lineA.Set(10, 11, 12, 13, 14, 15);
  EXPECT_DOUBLE_EQ(lineA[0].X(), 10.0);
  EXPECT_DOUBLE_EQ(lineA[0].Y(), 11.0);
  EXPECT_DOUBLE_EQ(lineA[0].Z(), 12.0);
  EXPECT_DOUBLE_EQ(lineA[1].X(), 13.0);
  EXPECT_DOUBLE_EQ(lineA[1].Y(), 14.0);
  EXPECT_DOUBLE_EQ(lineA[1].Z(), 15.0);

  lineA.SetA(math::Vector3<double>(0, -1, -2));
  EXPECT_DOUBLE_EQ(lineA[0].X(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[0].Y(), -1.0);
  EXPECT_DOUBLE_EQ(lineA[0].Z(), -2.0);
  EXPECT_DOUBLE_EQ(lineA[1].X(), 13.0);
  EXPECT_DOUBLE_EQ(lineA[1].Y(), 14.0);
  EXPECT_DOUBLE_EQ(lineA[1].Z(), 15.0);

  lineA.SetB(math::Vector3<double>(5, 6, 7));
  EXPECT_DOUBLE_EQ(lineA[0].X(), 0.0);
  EXPECT_DOUBLE_EQ(lineA[0].Y(), -1.0);
  EXPECT_DOUBLE_EQ(lineA[0].Z(), -2.0);
  EXPECT_DOUBLE_EQ(lineA[1].X(), 5.0);
  EXPECT_DOUBLE_EQ(lineA[1].Y(), 6.0);
  EXPECT_DOUBLE_EQ(lineA[1].Z(), 7.0);
}

/////////////////////////////////////////////////
TEST(Line3Test, Length)
{
  math::Line3d lineA(0, 0, 0, 10, 10, 10);
  EXPECT_NEAR(lineA.Length(), sqrt(300), 1e-10);
}

/////////////////////////////////////////////////
TEST(Line3Test, Equality)
{
  math::Line3d lineA(1, 1, 1, 2, 1, 2);
  math::Line3d lineB(1, 2, 3, 2, 2, 4);

  EXPECT_TRUE(lineA != lineB);
  EXPECT_TRUE(lineA == lineA);

  lineB.Set(1, 1, 1, 2, 1.1, 2);
  EXPECT_FALSE(lineA == lineB);

  lineB.Set(1, 1, 1, 2.1, 1, 2);
  EXPECT_FALSE(lineA == lineB);

  lineB.Set(1, 1, 1.1, 2, 1, 2);
  EXPECT_FALSE(lineA == lineB);

  lineB.Set(1.1, 1, 1, 2, 1, 2);
  EXPECT_FALSE(lineA == lineB);
}

/////////////////////////////////////////////////
TEST(Line3Test, OperatorStreamOut)
{
  math::Line3d line(0, 1, 4, 2, 3, 7);
  std::ostringstream stream;
  stream << line;
  EXPECT_EQ(stream.str(), "0 1 4 2 3 7");
}

/////////////////////////////////////////////////
TEST(Line3Test, CopyConstructor)
{
  math::Line3d lineA(0, 1, 4, 2, 3, 7);
  math::Line3d lineB(lineA);

  EXPECT_EQ(lineA, lineB);
}

/////////////////////////////////////////////////
TEST(Line3Test, OperatorAssign)
{
  math::Line3d lineA(0, 1, 4, 2, 3, 7);
  math::Line3d lineB = lineA;

  EXPECT_EQ(lineA, lineB);
}

/////////////////////////////////////////////////
TEST(Line3Test, Direction)
{
  math::Line3d lineA(1, 1, 1, 0, 0, 0);
  math::Line3d lineB(2, 2, 2, 0, 0, 0);
  math::Line3d lineC(0, 0, 0, 1, 1, 1);
  EXPECT_TRUE(lineA.Direction() == (lineA[1] - lineA[0]).Normalize());
  EXPECT_TRUE(lineA.Direction() == lineB.Direction());
  EXPECT_FALSE(lineA.Direction() == lineC.Direction());

  lineA.Set(1, 1, 2, 1, 1, 10);
  EXPECT_TRUE(lineA.Direction() == math::Vector3d::UnitZ);

  lineA.Set(1, 5, 1, 1, 1, 1);
  EXPECT_TRUE(lineA.Direction() == -math::Vector3d::UnitY);

  lineA.Set(1, 1, 1, 7, 1, 1);
  EXPECT_TRUE(lineA.Direction() == math::Vector3d::UnitX);
}

/////////////////////////////////////////////////
TEST(Line3Test, Within)
{
  math::Line3d line(0, 0, 0, 1, 1, 1);
  EXPECT_TRUE(line.Within(math::Vector3d(0, 0, 0)));
  EXPECT_TRUE(line.Within(math::Vector3d(1, 1, 1)));
  EXPECT_TRUE(line.Within(math::Vector3d(0.5, 0.5, 0.5)));

  EXPECT_FALSE(line.Within(math::Vector3d(-0.5, 0.5, 0.5)));
  EXPECT_FALSE(line.Within(math::Vector3d(0.5, -0.5, 0.5)));
  EXPECT_FALSE(line.Within(math::Vector3d(0.5, 0.5, -0.5)));
}

/////////////////////////////////////////////////
TEST(Line3Test, Distance)
{
  math::Line3d line(0, 0, 0, 0, 1, 0);
  math::Line3d result;

  EXPECT_TRUE(line.Distance(math::Line3d(1, 0.5, 0, -1, 0.5, 0), result));
  EXPECT_EQ(result.Length(), 0);
  EXPECT_EQ(result, math::Line3d(0, 0.5, 0, 0, 0.5, 0));

  EXPECT_TRUE(line.Distance(math::Line3d(1, 0, 0, -1, 0, 0), result));
  EXPECT_EQ(result.Length(), 0);
  EXPECT_EQ(result, math::Line3d(0, 0, 0, 0, 0, 0));

  EXPECT_TRUE(line.Distance(math::Line3d(1, 1.1, 0, -1, 1.1, 0), result));
  EXPECT_NEAR(result.Length(), 0.1, 1e-4);
  EXPECT_EQ(result, math::Line3d(0, 1, 0, 0, 1.1, 0));

  EXPECT_TRUE(line.Distance(math::Line3d(1, 0.5, 0.4, -1, 0.5, 0.4), result));
  EXPECT_NEAR(result.Length(), 0.4, 1e-4);
  EXPECT_EQ(result, math::Line3d(0, 0.5, 0, 0, 0.5, 0.4));

  EXPECT_TRUE(line.Distance(math::Line3d(0, 0.5, 1, 1, 0.5, 0), result));
  EXPECT_NEAR(result.Length(), sin(IGN_PI / 4), 1e-4);
  EXPECT_EQ(result, math::Line3d(0, 0.5, 0, 0.5, 0.5, 0.5));

  // Expect true when lines are parallel
  EXPECT_TRUE(line.Distance(math::Line3d(2, 0, 0, 2, 1, 0), result));
  EXPECT_EQ(result[0], line[0]);
  EXPECT_EQ(result[1], math::Vector3d(2, 0, 0));

  EXPECT_TRUE(line.Distance(math::Line3d(2, 1, 0, 2, 0, 0), result));
  EXPECT_EQ(result[0], line[0]);
  EXPECT_EQ(result[1], math::Vector3d(2, 0, 0));

  EXPECT_TRUE(line.Distance(math::Line3d(1, 1, 0, 1, 2, 0), result));
  EXPECT_EQ(result[0], line[1]);
  EXPECT_EQ(result[1], math::Vector3d(1, 1, 0));

  EXPECT_TRUE(line.Distance(math::Line3d(1, 2, 0, 1, 1, 0), result));
  EXPECT_EQ(result[0], line[1]);
  EXPECT_EQ(result[1], math::Vector3d(1, 1, 0));

  // Expect false when the passed in line is a point
  EXPECT_FALSE(line.Distance(math::Line3d(2, 0, 0, 2, 0, 0), result));

  // Expect false when the first line is a point.
  line.Set(0, 0, 0, 0, 0, 0);
  EXPECT_FALSE(line.Distance(math::Line3d(2, 0, 0, 2, 1, 0), result));
}

/////////////////////////////////////////////////
TEST(Line3Test, Intersect)
{
  math::Line3d line(0, 0, 0, 0, 1, 0);
  math::Vector3d pt;

  EXPECT_TRUE(line.Intersect(math::Line3d(1, 0.5, 0, -1, 0.5, 0)));
  EXPECT_TRUE(line.Intersect(math::Line3d(1, 0.5, 0, -1, 0.5, 0), pt));
  EXPECT_EQ(pt, math::Vector3d(0, 0.5, 0));

  EXPECT_TRUE(line.Intersect(math::Line3d(1, 0, 0, -1, 0, 0)));
  EXPECT_TRUE(line.Intersect(math::Line3d(1, 0, 0, -1, 0, 0), pt));
  EXPECT_EQ(pt, math::Vector3d(0, 0, 0));

  EXPECT_TRUE(line.Intersect(math::Line3d(1, 1, 0, -1, 1, 0)));
  EXPECT_TRUE(line.Intersect(math::Line3d(1, 1, 0, -1, 1, 0), pt));
  EXPECT_EQ(pt, math::Vector3d(0, 1, 0));

  EXPECT_TRUE(line.Intersect(math::Line3d(0, 0.5, -1, 0, 0.5, 1)));
  EXPECT_TRUE(line.Intersect(math::Line3d(0, 0.5, -1, 0, 0.5, 1), pt));
  EXPECT_EQ(pt, math::Vector3d(0, 0.5, 0));

  EXPECT_TRUE(line.Intersect(math::Line3d(-1, 0.5, -1, 1, 0.5, 1)));
  EXPECT_TRUE(line.Intersect(math::Line3d(-1, 0.5, -1, 1, 0.5, 1), pt));
  EXPECT_EQ(pt, math::Vector3d(0, 0.5, 0));

  EXPECT_FALSE(line.Intersect(math::Line3d(1, 1.1, 0, -1, 1.1, 0)));
  EXPECT_FALSE(line.Intersect(math::Line3d(1, -0.1, 0, -1, -0.1, 0)));

  EXPECT_FALSE(line.Intersect(math::Line3d(0.1, 0.1, 0, 0.6, 0.6, 0)));
  EXPECT_FALSE(line.Intersect(math::Line3d(-0.1, 0, 0, -0.1, 1, 0)));

  EXPECT_TRUE(line.Intersect(math::Line3d(0, -1, 0, 0, 0.1, 0)));
  EXPECT_TRUE(line.Intersect(math::Line3d(0, 1, 0, 0, 1.1, 0)));
}

/////////////////////////////////////////////////
TEST(Line3Test, Parallel)
{
  math::Line3d line(0, 0, 0, 0, 1, 0);
  EXPECT_TRUE(line.Parallel(math::Line3d(1, 0, 0, 1, 1, 0)));
  EXPECT_TRUE(line.Parallel(math::Line3d(1, 1, 0, 1, 0, 0)));
  EXPECT_TRUE(line.Parallel(math::Line3d(0, 0, 0, 0, 10, 0)));
  EXPECT_TRUE(line.Parallel(math::Line3d(-100, 100, 20, -100, 200, 20)));

  EXPECT_FALSE(line.Parallel(math::Line3d(1, 0, 0, 1, 1, 1)));
  EXPECT_FALSE(line.Parallel(math::Line3d(1, 0, 0, 2, 0, 0)));
  EXPECT_FALSE(line.Parallel(math::Line3d(1, 0, 1, 2, 0, 1)));
}

/////////////////////////////////////////////////
TEST(Line3Test, Coplanar)
{
  math::Line3d line(0, 0, 0, 0, 1, 0);
  EXPECT_TRUE(line.Coplanar(math::Line3d(1, 0, 0, 1, 1, 0)));
  EXPECT_TRUE(line.Coplanar(math::Line3d(0, 0, 0, 0, 10, 0)));
  EXPECT_TRUE(line.Coplanar(math::Line3d(-100, 100, 20, -100, 200, 20)));

  EXPECT_FALSE(line.Coplanar(math::Line3d(1, 0, 0, 1, 1, 1)));
  EXPECT_FALSE(line.Coplanar(math::Line3d(1, 0, 1, 2, 0, 0)));
}

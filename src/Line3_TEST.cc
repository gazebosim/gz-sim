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

  EXPECT_THROW(lineB[2].X(), math::IndexException);
  EXPECT_NO_THROW(lineA[0].X());

  math::Line3d lineC(0, 0, 5, 10, 10, 6);
  EXPECT_DOUBLE_EQ(lineC[0].X(), 0.0);
  EXPECT_DOUBLE_EQ(lineC[0].Y(), 0.0);
  EXPECT_DOUBLE_EQ(lineC[0].Z(), 5.0);
  EXPECT_DOUBLE_EQ(lineC[1].X(), 10.0);
  EXPECT_DOUBLE_EQ(lineC[1].Y(), 10.0);
  EXPECT_DOUBLE_EQ(lineC[1].Z(), 6.0);
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

  lineA.Set(1, 1, 3, 2, 2, 4);
  EXPECT_DOUBLE_EQ(lineA[0].X(), 1.0);
  EXPECT_DOUBLE_EQ(lineA[0].Y(), 1.0);
  EXPECT_DOUBLE_EQ(lineA[0].Z(), 3.0);
  EXPECT_DOUBLE_EQ(lineA[1].X(), 2.0);
  EXPECT_DOUBLE_EQ(lineA[1].Y(), 2.0);
  EXPECT_DOUBLE_EQ(lineA[1].Z(), 4.0);
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
TEST(Line3Test, Direction)
{
  math::Line3d lineA(1, 1, 1, 2, 1, 6);
  EXPECT_TRUE(lineA.Dir() == lineA[1] - lineA[0]);
  EXPECT_FALSE(lineA.Dir() == lineA[0] - lineA[1]);
}

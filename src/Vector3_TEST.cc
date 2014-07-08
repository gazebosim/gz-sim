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

#include "ignition/math/Vector3.hh"
#include "ignition/math/Helpers.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Vector3Test, Vector3d)
{
  math::Vector3d v;

  // ::Distance, ::LEngth()
  v.Set(1, 2, 3);
  EXPECT_FLOAT_EQ(v.Length(), v.Distance(math::Vector3d(0, 0, 0)));

  // ::Round
  v.Set(1.23, 2.34, 3.55);
  EXPECT_TRUE(v.Rounded() == math::Vector3d(1, 2, 4));

  // ::Round
  v.Round();
  EXPECT_TRUE(v.Round() == math::Vector3d(1, 2, 4));

  // ::DotProd
  EXPECT_TRUE(math::equal(17.0, v.Dot(math::Vector3d(1, 2, 3)), 1e-2));

  // ::DistToLine
  v.Set(0, 0, 0);
  EXPECT_DOUBLE_EQ(1.0, v.DistToLine(math::Vector3d(1, -1, 0),
        math::Vector3d(1, 1, 0)));

  // ::operator= double
  v = 4.0;
  EXPECT_TRUE(v == math::Vector3d(4, 4, 4));

  // ::operator/ vector3
  v = v / math::Vector3d(1, 2, 4);
  EXPECT_TRUE(v == math::Vector3d(4, 2, 1));

  // ::operator / double
  v = v / 2;
  EXPECT_TRUE(v == math::Vector3d(2, 1, .5));

  // ::operator * vector3
  v = v * math::Vector3d(2, 3, 4);
  EXPECT_TRUE(v == math::Vector3d(4, 3, 2));

  // ::operator[]
  EXPECT_DOUBLE_EQ(v[0], 4);
  EXPECT_DOUBLE_EQ(v[1], 3);
  EXPECT_DOUBLE_EQ(v[2], 2);

  v.Set(1.23, 2.35, 3.654321);
  v.Round(1);
  EXPECT_TRUE(v == math::Vector3d(1.2, 2.4, 3.7));

  // operator GetAbs
  v.Set(-1, -2, -3);
  EXPECT_TRUE(v.Abs() == math::Vector3d(1, 2, 3));

  // operator /=
  v.Set(1, 2, 4);
  v /= math::Vector3d(1, 4, 4);
  EXPECT_TRUE(v == math::Vector3d(1, .5, 1));

  // operator *=
  v.Set(1, 2, 4);
  v *= math::Vector3d(2, .5, .1);
  EXPECT_TRUE(v.Equal(math::Vector3d(2, 1, .4)));

  // Test the static defines.
  EXPECT_TRUE(math::Vector3d::Zero == math::Vector3d(0, 0, 0));
  EXPECT_TRUE(math::Vector3d::One == math::Vector3d(1, 1, 1));
  EXPECT_TRUE(math::Vector3d::UnitX == math::Vector3d(1, 0, 0));
  EXPECT_TRUE(math::Vector3d::UnitY == math::Vector3d(0, 1, 0));
  EXPECT_TRUE(math::Vector3d::UnitZ == math::Vector3d(0, 0, 1));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Distance)
{
  math::Vector3d vec1(0, 0, 0);
  math::Vector3d vec2(1, 2, 3);

  double dist = vec1.Distance(vec2);
  EXPECT_NEAR(dist, 3.74165738677, 1e-6);

  double dist2 = vec1.Distance(1, 2, 3);
  EXPECT_DOUBLE_EQ(dist, dist2);
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Sum)
{
  math::Vector3d vec1(0, 0, 0);
  math::Vector3d vec2(1, 2, 3);

  double sum1 = vec1.Sum();
  double sum2 = vec2.Sum();

  EXPECT_DOUBLE_EQ(sum1, 0);
  EXPECT_DOUBLE_EQ(sum2, 6);
}

/////////////////////////////////////////////////
TEST(Vector3dTest, SquaredLength)
{
  math::Vector3d vec1(0, 0, 0);
  math::Vector3d vec2(1, 2, 3);

  double sum1 = vec1.SquaredLength();
  double sum2 = vec2.SquaredLength();

  EXPECT_DOUBLE_EQ(sum1, 0);
  EXPECT_DOUBLE_EQ(sum2, 14);
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Normalize)
{
  math::Vector3d vec1(0, 0, 0);
  math::Vector3d vec2(1, 2, 3);

  math::Vector3d vec3 = vec1.Normalize();
  EXPECT_EQ(vec3, vec1);
  EXPECT_EQ(vec1, math::Vector3d(0, 0, 0));

  vec3 = vec2.Normalize();
  EXPECT_EQ(vec3, vec2);
  EXPECT_EQ(vec2, math::Vector3d(0.267261, 0.534522, 0.801784));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, GetNormal)
{
  math::Vector3d vec1(0, 0, 0);
  math::Vector3d vec2(0, 1, 0);
  math::Vector3d vec3(1, 1, 0);

  math::Vector3d norm = math::Vector3d::Normal(vec1, vec2, vec3);
  EXPECT_EQ(norm, math::Vector3d(0, 0, -1));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Perpendicular)
{
  math::Vector3d vec1(1, 1, 0);
  math::Vector3d vec2(0, 1, 1);
  math::Vector3d vec3(1e-7, 1e-7, 1e-7);
  math::Vector3d vec4(1, 0, 0);

  EXPECT_EQ(vec1.Perpendicular(), math::Vector3d(0, 0, -1));
  EXPECT_EQ(vec2.Perpendicular(), math::Vector3d(0, 1, -1));
  EXPECT_EQ(vec3.Perpendicular(), math::Vector3d(0, 0, 0));
  EXPECT_EQ(vec4.Perpendicular(), math::Vector3d(0, 0, 1));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Max)
{
  math::Vector3d vec1(0.1, 0.2, 0.3);
  math::Vector3d vec2(0.2, 0.3, 0.4);
  math::Vector3d vec3(0.1, 0.2, 0.3);

  EXPECT_DOUBLE_EQ(vec1.Max(), 0.3);

  vec1.SetToMax(vec2);
  EXPECT_EQ(vec1, math::Vector3d(0.2, 0.3, 0.4));

  vec1.SetToMax(vec3);
  EXPECT_EQ(vec1, math::Vector3d(0.2, 0.3, 0.4));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Min)
{
  math::Vector3d vec1(0.1, 0.2, 0.3);
  math::Vector3d vec2(0.2, 0.3, 0.4);
  math::Vector3d vec3(0.05, 0.1, 0.2);

  EXPECT_DOUBLE_EQ(vec1.Min(), 0.1);

  vec1.SetToMin(vec2);
  EXPECT_EQ(vec1, math::Vector3d(0.1, 0.2, 0.3));

  vec1.SetToMin(vec3);
  EXPECT_EQ(vec1, math::Vector3d(0.05, 0.1, 0.2));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Add)
{
  math::Vector3d vec1(0.1, 0.2, 0.4);
  math::Vector3d vec2(1.1, 2.2, 3.4);

  math::Vector3d vec3 = vec1;
  vec3 += vec2;

  EXPECT_EQ(vec1 + vec2, math::Vector3d(1.2, 2.4, 3.8));
  EXPECT_EQ(vec3, math::Vector3d(1.2, 2.4, 3.8));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Sub)
{
  math::Vector3d vec1(0.1, 0.2, 0.4);
  math::Vector3d vec2(1.1, 2.2, 3.4);

  math::Vector3d vec3 = vec2;
  vec3 -= vec1;

  EXPECT_EQ(vec2 - vec1, math::Vector3d(1.0, 2.0, 3.0));
  EXPECT_EQ(vec3, math::Vector3d(1.0, 2.0, 3.0));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Divide)
{
  math::Vector3d vec1(0.1, 0.2, 0.4);

  math::Vector3d vec3 = vec1 / 2.0;
  EXPECT_EQ(vec3, math::Vector3d(0.05, 0.1, 0.2));

  vec3 /= 4.0;
  EXPECT_EQ(vec3, math::Vector3d(0.0125, 0.025, 0.05));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Multiply)
{
  math::Vector3d vec1(0.1, 0.2, 0.3);

  math::Vector3d vec3 = vec1 * 2.0;
  EXPECT_EQ(vec3, math::Vector3d(0.2, 0.4, 0.6));

  vec3 *= 4.0;
  EXPECT_EQ(vec3, math::Vector3d(0.8, 1.6, 2.4));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, NotEqual)
{
  math::Vector3d vec1(0.1, 0.2, 0.3);
  math::Vector3d vec2(0.2, 0.2, 0.3);
  math::Vector3d vec3(0.1, 0.2, 0.3);

  EXPECT_TRUE(vec1 != vec2);
  EXPECT_FALSE(vec1 != vec3);
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Finite)
{
  math::Vector3d vec1(0.1, 0.2, 0.3);

  EXPECT_TRUE(vec1.IsFinite());
}

/////////////////////////////////////////////////
TEST(Vector3dTest, IndexException)
{
  math::Vector3d v(1, 2, 3);
  EXPECT_NO_THROW(math::equal(v[0], 1.0));
  EXPECT_NO_THROW(math::equal(v[1], 2.0));
  EXPECT_NO_THROW(math::equal(v[2], 3.0));

  EXPECT_THROW(math::equal(v[3], 4.0), math::IndexException);
}

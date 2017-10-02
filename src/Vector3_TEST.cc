/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

  // ::Distance, ::Length()
  v.Set(1, 2, 3);
  EXPECT_DOUBLE_EQ(v.Length(), v.Distance(math::Vector3d(0, 0, 0)));

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
  v[0] = 40;
  v[1] = 30;
  v[2] = 20;
  EXPECT_DOUBLE_EQ(v[0], 40);
  EXPECT_DOUBLE_EQ(v[1], 30);
  EXPECT_DOUBLE_EQ(v[2], 20);

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
TEST(Vector3dTest, Length)
{
  // Zero vector
  EXPECT_DOUBLE_EQ(math::Vector3d::Zero.Length(), 0.0);
  EXPECT_DOUBLE_EQ(math::Vector3d::Zero.SquaredLength(), 0.0);

  // UnitXYZ vectors
  EXPECT_DOUBLE_EQ(math::Vector3d::UnitX.Length(), 1.0);
  EXPECT_DOUBLE_EQ(math::Vector3d::UnitY.Length(), 1.0);
  EXPECT_DOUBLE_EQ(math::Vector3d::UnitZ.Length(), 1.0);
  EXPECT_DOUBLE_EQ(math::Vector3d::UnitX.SquaredLength(), 1.0);
  EXPECT_DOUBLE_EQ(math::Vector3d::UnitY.SquaredLength(), 1.0);
  EXPECT_DOUBLE_EQ(math::Vector3d::UnitZ.SquaredLength(), 1.0);

  // One vector
  EXPECT_NEAR(math::Vector3d::One.Length(), sqrt(3.0), 1e-10);
  EXPECT_DOUBLE_EQ(math::Vector3d::One.SquaredLength(), 3.0);

  // Arbitrary vector
  math::Vector3d v(0.1, -4.2, 2.5);
  EXPECT_NEAR(v.Length(), 4.88876262463, 1e-10);
  EXPECT_DOUBLE_EQ(v.SquaredLength(), 23.9);
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

  const math::Vector3d vecConst(1, 2, 3);
  EXPECT_EQ(vecConst.Normalized(), vec3);
  EXPECT_EQ(vecConst, math::Vector3d(1, 2, 3));
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

  vec1.Max(vec2);
  EXPECT_EQ(vec1, math::Vector3d(0.2, 0.3, 0.4));

  vec1.Max(vec3);
  EXPECT_EQ(vec1, math::Vector3d(0.2, 0.3, 0.4));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Min)
{
  math::Vector3d vec1(0.1, 0.2, 0.3);
  math::Vector3d vec2(0.2, 0.3, 0.4);
  math::Vector3d vec3(0.05, 0.1, 0.2);

  EXPECT_DOUBLE_EQ(vec1.Min(), 0.1);

  vec1.Min(vec2);
  EXPECT_EQ(vec1, math::Vector3d(0.1, 0.2, 0.3));

  vec1.Min(vec3);
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

  // Add zeros
  {
    // Scalar left and right
    EXPECT_EQ(0 + vec1, vec1);
    EXPECT_EQ(vec1 + 0, vec1);

    // Vector left and right
    EXPECT_EQ(math::Vector3d::Zero + vec1, vec1);
    EXPECT_EQ(vec1 + math::Vector3d::Zero, vec1);

    // Addition assignment
    math::Vector3d vec4(vec1);
    vec4 += 0;
    EXPECT_EQ(vec4, vec1);
    vec4 += math::Vector3d::Zero;
    EXPECT_EQ(vec4, vec1);
  }

  // Add non-trivial scalar values left and right
  {
    EXPECT_EQ(2.5 + vec1, math::Vector3d(2.6, 2.7, 2.9));
    EXPECT_EQ(vec1 + 2.5, math::Vector3d(2.6, 2.7, 2.9));

    math::Vector3d vec4(vec1);
    vec4 += 2.5;
    EXPECT_EQ(vec4, math::Vector3d(2.6, 2.7, 2.9));
  }
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

  // Subtraction with zeros
  {
    // Scalar left and right
    EXPECT_EQ(0 - vec1, -vec1);
    EXPECT_EQ(vec1 - 0, vec1);

    // Vector left and right
    EXPECT_EQ(math::Vector3d::Zero - vec1, -vec1);
    EXPECT_EQ(vec1 - math::Vector3d::Zero, vec1);

    // Subtraction assignment
    math::Vector3d vec4(vec1);
    vec4 -= 0;
    EXPECT_EQ(vec4, vec1);
    vec4 -= math::Vector3d::Zero;
    EXPECT_EQ(vec4, vec1);
  }

  // Subtract non-trivial scalar values left and right
  {
    EXPECT_EQ(2.5 - vec1, math::Vector3d(2.4, 2.3, 2.1));
    EXPECT_EQ(vec1 - 2.5, -math::Vector3d(2.4, 2.3, 2.1));

    math::Vector3d vec4(vec1);
    vec4 -= 2.5;
    EXPECT_EQ(vec4, -math::Vector3d(2.4, 2.3, 2.1));
  }
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
  math::Vector3d v(0.1, 0.2, 0.3);

  math::Vector3d vec3 = v * 2.0;
  EXPECT_EQ(vec3, math::Vector3d(0.2, 0.4, 0.6));

  vec3 *= 4.0;
  EXPECT_EQ(vec3, math::Vector3d(0.8, 1.6, 2.4));

  // Multiply by zero
  {
    // Scalar left and right
    EXPECT_EQ(0 * v, math::Vector3d::Zero);
    EXPECT_EQ(v * 0, math::Vector3d::Zero);

    // Element-wise vector multiplication
    EXPECT_EQ(v * math::Vector3d::Zero, math::Vector3d::Zero);
  }

  // Multiply by one
  {
    // Scalar left and right
    EXPECT_EQ(1 * v, v);
    EXPECT_EQ(v * 1, v);

    // Element-wise vector multiplication
    EXPECT_EQ(v * math::Vector3d::One, v);
  }

  // Multiply by non-trivial scalar value
  {
    const double scalar = 2.5;
    math::Vector3d expect(0.25, 0.5, 0.75);
    EXPECT_EQ(scalar * v, expect);
    EXPECT_EQ(v * scalar, expect);
  }

  // Multiply by itself element-wise
  EXPECT_EQ(v*v, math::Vector3d(0.01, 0.04, 0.09));
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
// Test Equal function with specified tolerance
TEST(Vector2Test, EqualTolerance)
{
  EXPECT_FALSE(math::Vector3d::Zero.Equal(math::Vector3d::One, 1e-6));
  EXPECT_FALSE(math::Vector3d::Zero.Equal(math::Vector3d::One, 1e-3));
  EXPECT_FALSE(math::Vector3d::Zero.Equal(math::Vector3d::One, 1e-1));
  EXPECT_TRUE(math::Vector3d::Zero.Equal(math::Vector3d::One, 1));
  EXPECT_TRUE(math::Vector3d::Zero.Equal(math::Vector3d::One, 1.1));
}

/////////////////////////////////////////////////
TEST(Vector3dTest, Finite)
{
  math::Vector3d vec1(0.1, 0.2, 0.3);

  EXPECT_TRUE(vec1.IsFinite());
}

/////////////////////////////////////////////////
TEST(Vector3dTest, NoException)
{
  math::Vector3d v(1, 2, 3);
  EXPECT_NO_THROW(math::equal(v[0], 1.0));
  EXPECT_NO_THROW(math::equal(v[1], 2.0));
  EXPECT_NO_THROW(math::equal(v[2], 3.0));

  EXPECT_NO_THROW(math::equal(v[3], 4.0));
  EXPECT_DOUBLE_EQ(v[3], 3.0);
}


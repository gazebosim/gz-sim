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
#include <cmath>

#include "ignition/math/Helpers.hh"
#include "ignition/math/Vector2.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Vector2Test, Vector2)
{
  {
    math::Vector2d v;
    EXPECT_DOUBLE_EQ(0, v.X());
    EXPECT_DOUBLE_EQ(0, v.Y());
  }

  // Constructor
  math::Vector2d v(1, 2);
  EXPECT_DOUBLE_EQ(1, v.X());
  EXPECT_DOUBLE_EQ(2, v.Y());

  // ::Distance
  EXPECT_TRUE(math::equal(2.236, v.Distance(math::Vector2d(0, 0)), 1e-2));

  // ::Normalize
  v.Normalize();
  EXPECT_TRUE(v == math::Vector2d(0.447214, 0.894427));

  // ::Set
  v.Set(4, 5);
  EXPECT_TRUE(v == math::Vector2d(4, 5));

  // ::operator=
  v = math::Vector2d(6, 7);
  EXPECT_TRUE(v == math::Vector2d(6, 7));

  // ::operator= int
  v = 5;
  EXPECT_TRUE(v == math::Vector2d(5, 5));

  // ::operator+
  v = v + math::Vector2d(1, 2);
  EXPECT_TRUE(v == math::Vector2d(6, 7));

  // ::operator +=
  v += math::Vector2d(5, 6);
  EXPECT_TRUE(v == math::Vector2d(11, 13));

  // ::operator -
  v = v - math::Vector2d(2, 4);
  EXPECT_TRUE(v == math::Vector2d(9, 9));

  // ::operator -=
  v.Set(2, 4);
  v -= math::Vector2d(1, 6);
  EXPECT_TRUE(v == math::Vector2d(1, -2));

  // ::operator /
  v.Set(10, 6);
  v = v / math::Vector2d(2, 3);
  EXPECT_TRUE(v == math::Vector2d(5, 2));

  // ::operator /=
  v.Set(10, 6);
  v /= math::Vector2d(2, 3);
  EXPECT_TRUE(v == math::Vector2d(5, 2));

  // ::operator / int
  v.Set(10, 6);
  v = v / 2;
  EXPECT_TRUE(v == math::Vector2d(5, 3));

  // ::operator /= int
  v.Set(10, 6);
  v /= 2;
  EXPECT_TRUE(v == math::Vector2d(5, 3));

  // ::operator * int
  v.Set(10, 6);
  v = v * 2;
  EXPECT_TRUE(v == math::Vector2d(20, 12));

  // ::operator *= int
  v.Set(10, 6);
  v *= 2;
  EXPECT_TRUE(v == math::Vector2d(20, 12));

  // ::operator * vector2i
  v.Set(10, 6);
  v = v * math::Vector2d(2, 4);
  EXPECT_TRUE(v == math::Vector2d(20, 24));

  // ::operator *= vector2i
  v.Set(10, 6);
  v *= math::Vector2d(2, 4);
  EXPECT_TRUE(v == math::Vector2d(20, 24));


  // ::IsFinite
  EXPECT_TRUE(v.IsFinite());

  // ::operator[]
  v[0] = 6;
  v[1] = 7;
  EXPECT_DOUBLE_EQ(6, v[0]);
  EXPECT_DOUBLE_EQ(7, v[1]);
}

/////////////////////////////////////////////////
TEST(Vector2Test, NoException)
{
  math::Vector2d v(1, 2);
  EXPECT_NO_THROW(math::equal(v[0], 1.0));
  EXPECT_NO_THROW(math::equal(v[1], 2.0));

  EXPECT_NO_THROW(math::equal(v[2], 1.0));
  EXPECT_DOUBLE_EQ(v[2], 2.0);
}

/////////////////////////////////////////////////
// Test Equal function with specified tolerance
TEST(Vector2Test, EqualTolerance)
{
  EXPECT_FALSE(math::Vector2d::Zero.Equal(math::Vector2d::One, 1e-6));
  EXPECT_FALSE(math::Vector2d::Zero.Equal(math::Vector2d::One, 1e-3));
  EXPECT_FALSE(math::Vector2d::Zero.Equal(math::Vector2d::One, 1e-1));
  EXPECT_TRUE(math::Vector2d::Zero.Equal(math::Vector2d::One, 1));
  EXPECT_TRUE(math::Vector2d::Zero.Equal(math::Vector2d::One, 1.1));
}

/////////////////////////////////////////////////
TEST(Vector2Test, Dot)
{
  math::Vector2d v(1, 2);

  EXPECT_DOUBLE_EQ(v.Dot(math::Vector2d(3, 4)), 11.0);
  EXPECT_DOUBLE_EQ(v.Dot(math::Vector2d(0, 0)), 0.0);
  EXPECT_DOUBLE_EQ(v.Dot(math::Vector2d(1, 0)), 1.0);
  EXPECT_DOUBLE_EQ(v.Dot(math::Vector2d(0, 1)), 2.0);
}

/////////////////////////////////////////////////
TEST(Vector2Test, OperatorStreamOut)
{
  math::Vector2d v(0.1, 1.2);
  std::ostringstream stream;
  stream << v;
  EXPECT_EQ(stream.str(), "0.1 1.2");
}

/////////////////////////////////////////////////
TEST(Vector2dTest, Add)
{
  math::Vector2d vec1(0.1, 0.2);
  math::Vector2d vec2(1.1, 2.2);

  math::Vector2d vec3 = vec1;
  vec3 += vec2;

  EXPECT_EQ(vec1 + vec2, math::Vector2d(1.2, 2.4));
  EXPECT_EQ(vec3, math::Vector2d(1.2, 2.4));

  // Add zeros
  {
    // Scalar left and right
    EXPECT_EQ(0 + vec1, vec1);
    EXPECT_EQ(vec1 + 0, vec1);

    // Vector left and right
    EXPECT_EQ(math::Vector2d::Zero + vec1, vec1);
    EXPECT_EQ(vec1 + math::Vector2d::Zero, vec1);

    // Addition assignment
    math::Vector2d vec4(vec1);
    vec4 += 0;
    EXPECT_EQ(vec4, vec1);
    vec4 += math::Vector2d::Zero;
    EXPECT_EQ(vec4, vec1);
  }

  // Add non-trivial scalar values left and right
  {
    EXPECT_EQ(2.5 + vec1, math::Vector2d(2.6, 2.7));
    EXPECT_EQ(vec1 + 2.5, math::Vector2d(2.6, 2.7));

    math::Vector2d vec4(vec1);
    vec4 += 2.5;
    EXPECT_EQ(vec4, math::Vector2d(2.6, 2.7));
  }
}

/////////////////////////////////////////////////
TEST(Vector2dTest, Sub)
{
  math::Vector2d vec1(0.1, 0.2);
  math::Vector2d vec2(1.1, 2.2);

  math::Vector2d vec3 = vec2;
  vec3 -= vec1;

  EXPECT_EQ(vec2 - vec1, math::Vector2d(1.0, 2.0));
  EXPECT_EQ(vec3, math::Vector2d(1.0, 2.0));

  // Subtraction with zeros
  {
    // Scalar left and right
    EXPECT_EQ(0 - vec1, -vec1);
    EXPECT_EQ(vec1 - 0, vec1);

    // Vector left and right
    EXPECT_EQ(math::Vector2d::Zero - vec1, -vec1);
    EXPECT_EQ(vec1 - math::Vector2d::Zero, vec1);

    // Subtraction assignment
    math::Vector2d vec4(vec1);
    vec4 -= 0;
    EXPECT_EQ(vec4, vec1);
    vec4 -= math::Vector2d::Zero;
    EXPECT_EQ(vec4, vec1);
  }

  // Subtract non-trivial scalar values left and right
  {
    EXPECT_EQ(2.5 - vec1, math::Vector2d(2.4, 2.3));
    EXPECT_EQ(vec1 - 2.5, -math::Vector2d(2.4, 2.3));

    math::Vector2d vec4(vec1);
    vec4 -= 2.5;
    EXPECT_EQ(vec4, -math::Vector2d(2.4, 2.3));
  }
}

/////////////////////////////////////////////////
TEST(Vector2Test, Multiply)
{
  math::Vector2d v(0.1, -4.2);

  // Multiply by zero
  {
    // Scalar left and right
    EXPECT_EQ(0 * v, math::Vector2d::Zero);
    EXPECT_EQ(v * 0, math::Vector2d::Zero);

    // Element-wise vector multiplication
    EXPECT_EQ(v * math::Vector2d::Zero, math::Vector2d::Zero);
  }

  // Multiply by one
  {
    // Scalar left and right
    EXPECT_EQ(1 * v, v);
    EXPECT_EQ(v * 1, v);

    // Element-wise vector multiplication
    EXPECT_EQ(v * math::Vector2d::One, v);
  }

  // Multiply by non-trivial scalar value
  {
    const double scalar = 2.5;
    math::Vector2d expect(0.25, -10.5);
    EXPECT_EQ(scalar * v, expect);
    EXPECT_EQ(v * scalar, expect);
  }

  // Multiply by itself element-wise
  EXPECT_EQ(v*v, math::Vector2d(0.01, 17.64));
}

/////////////////////////////////////////////////
TEST(Vector2dTest, Length)
{
  // Zero vector
  EXPECT_DOUBLE_EQ(math::Vector2d::Zero.Length(), 0.0);
  EXPECT_DOUBLE_EQ(math::Vector2d::Zero.SquaredLength(), 0.0);

  // One vector
  EXPECT_NEAR(math::Vector2d::One.Length(), IGN_SQRT2, 1e-10);
  EXPECT_DOUBLE_EQ(math::Vector2d::One.SquaredLength(), 2.0);

  // Arbitrary vector
  math::Vector2d v(0.1, -4.2);
  EXPECT_NEAR(v.Length(), 4.20119030752, 1e-10);
  EXPECT_DOUBLE_EQ(v.SquaredLength(), 17.65);
}


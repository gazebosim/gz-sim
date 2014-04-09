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
#include "ignition/math/Matrix4d.hh"
#include "ignition/math/Vector4d.hh"

using namespace ignition;

TEST(Vector4dTest, Vector4d)
{
  {
    math::Vector4d v;
    EXPECT_TRUE(math::equal(v.x(), 0.0));
    EXPECT_TRUE(math::equal(v.y(), 0.0));
    EXPECT_TRUE(math::equal(v.z(), 0.0));
    EXPECT_TRUE(math::equal(v.w(), 0.0));
  }

  math::Vector4d v1(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(v1.x(), 1.0));
  EXPECT_TRUE(math::equal(v1.y(), 2.0));
  EXPECT_TRUE(math::equal(v1.z(), 3.0));
  EXPECT_TRUE(math::equal(v1.w(), 4.0));

  math::Vector4d v(v1);
  EXPECT_TRUE(v == v1);

  EXPECT_TRUE(math::equal(v.Distance(
          math::Vector4d(0, 0, 0, 0)), 5.4772, 1e-3));

  // ::GetLength()
  v.Set(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(v.GetLength(), 5.4772, 1e-3));

  // ::GetSquaredLength()
  EXPECT_TRUE(math::equal(v.GetSquaredLength(), 30.0));

  // ::Normalize
  v.Normalize();
  EXPECT_TRUE(v == math::Vector4d(0.182574, 0.365148, 0.547723, 0.730297));

  // ::Set
  v.Set(2, 4, 6, 8);
  EXPECT_TRUE(v == math::Vector4d(2, 4, 6, 8));

  // ::operator= vector4
  v = v1;
  EXPECT_TRUE(v == v1);

  // ::operator= double
  v = 1.2;
  EXPECT_TRUE(v == math::Vector4d(1.2, 1.2, 1.2, 1.2));

  // ::operator+ vector4
  v = v + math::Vector4d(1, 2, 3, 4);
  EXPECT_TRUE(v == math::Vector4d(2.2, 3.2, 4.2, 5.2));

  // ::operator+=
  v += v;
  EXPECT_TRUE(v == math::Vector4d(4.4, 6.4, 8.4, 10.4));

  // ::operator- vector4
  v = v - math::Vector4d(1, 1, 1, 1);
  EXPECT_TRUE(v == math::Vector4d(3.4, 5.4, 7.4, 9.4));

  // ::operator-= vector4
  v -= math::Vector4d(1, 1, 1, 1);
  EXPECT_TRUE(v == math::Vector4d(2.4, 4.4, 6.4, 8.4));


  // ::operator/ vector4
  v = v / math::Vector4d(.1, .1, .1, .1);
  EXPECT_TRUE(v == math::Vector4d(24, 44, 64, 84));

  // ::operator/ double
  v = v / 2.0;
  EXPECT_TRUE(v == math::Vector4d(12, 22, 32, 42));

  // ::operator/= vector4
  v /= math::Vector4d(4, 4, 4, 4);
  EXPECT_TRUE(v == math::Vector4d(3, 5.5, 8, 10.5));

  // ::operator/= double
  v /= .1;
  EXPECT_TRUE(v == math::Vector4d(30, 55, 80, 105));

  // ::operator * matrix4
  v = v * math::Matrix4d(1, 2, 3, 4,
                        5, 6, 7, 8,
                        9, 10, 11, 12,
                        13, 14, 15, 16);
  EXPECT_TRUE(v == math::Vector4d(2390, 2660, 2930, 3200));


  // ::operator * vector4
  v = v * math::Vector4d(.2, .3, .4, .5);
  EXPECT_TRUE(v == math::Vector4d(478, 798, 1172, 1600));

  // ::operator *= vector4
  v *= math::Vector4d(2, 4, 6, 8);
  EXPECT_TRUE(v == math::Vector4d(956, 3192, 7032, 12800));

  // ::operator * double
  v = v * 5.2;
  EXPECT_TRUE(v == math::Vector4d(4971.2, 16598.4, 36566.4, 66560));

  // ::operator *= double
  v *= 10.0;
  EXPECT_TRUE(v == math::Vector4d(49712, 1.65984e+05, 3.65664e+05, 6.656e+05));

  // ::operator != vector4
  EXPECT_TRUE(v != math::Vector4d());

  // ::IsFinite
  EXPECT_TRUE(v.IsFinite());

  // ::operator[]
  v.Set(1, 2, 3, 4);
  EXPECT_DOUBLE_EQ(v[0], 1);
  EXPECT_DOUBLE_EQ(v[1], 2);
  EXPECT_DOUBLE_EQ(v[2], 3);
  EXPECT_DOUBLE_EQ(v[3], 4);
}

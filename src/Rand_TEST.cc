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
#include "ignition/math/Rand.hh"

using namespace ignition;

//////////////////////////////////////////////////
TEST(RandTest, Rand)
{
  // TODO: implement a proper random number generator test

  double d = math::Rand::DblUniform(1, 2);
  EXPECT_GE(d, 1);
  EXPECT_LE(d, 2);

  int i = math::Rand::IntUniform(1, 2);
  EXPECT_GE(i, 1);
  EXPECT_LE(i, 2);

#ifndef _MSC_VER
  {
    // Test setting the random number seed
    math::Rand::Seed(1001);

    d = math::Rand::DblNormal(2, 3);
    i = math::Rand::IntNormal(10, 5);

    // \todo OSX seems to produce different results. See issue #14.
#ifdef __APPLE__
    EXPECT_EQ(i, 8);
    EXPECT_NEAR(d, 5.01545, 1e-5);
#else
    EXPECT_EQ(i, 9);
    EXPECT_NEAR(d, 3.00618, 1e-5);
#endif
  }
#endif
}

//////////////////////////////////////////////////
TEST(RandTest, SetSeed)
{
  int N = 10;
  std::vector<int> first;
  std::vector<int> second;

  for (int i = 0; i < N; ++i)
  {
    math::Rand::Seed(i);
    first.push_back(math::Rand::IntUniform(-10, 10));
    second.push_back(math::Rand::IntUniform(-10, 10));
  }

  for (int i = 0; i < N; ++i)
  {
    math::Rand::Seed(i);
    EXPECT_EQ(math::Rand::Seed(), i);
    EXPECT_EQ(first[i], math::Rand::IntUniform(-10, 10));
    EXPECT_EQ(second[i], math::Rand::IntUniform(-10, 10));
  }
}

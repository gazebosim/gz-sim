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
  double d;
  int i;
  // TODO: implement a proper random number generator test

  d = ignition::math::Rand::GetDblUniform(1, 2);
  EXPECT_LE(d, 2);
  EXPECT_GE(d, 1);

  d = math::Rand::GetDblNormal(2, 3);
  i = math::Rand::GetIntUniform(1, 2);
  EXPECT_LE(i, 2);
  EXPECT_GE(i, 1);

  i = math::Rand::GetIntNormal(2, 3);

  // Test setting the random number seed
  {
    math::Rand::SetSeed(1001);

    d = math::Rand::GetDblNormal(2, 3);
    EXPECT_NEAR(d, 4.72597, 1e-5);
  }
}

//////////////////////////////////////////////////
TEST(RandTest, SetSeed)
{
  int N = 10;
  std::vector<int> first;
  std::vector<int> second;

  for (int i = 0; i < N; ++i)
  {
    math::Rand::SetSeed(i);
    first.push_back(math::Rand::GetIntUniform(-10, 10));
    second.push_back(math::Rand::GetIntUniform(-10, 10));
  }

  for (int i = 0; i < N; ++i)
  {
    math::Rand::SetSeed(i);
    EXPECT_EQ(first[i], math::Rand::GetIntUniform(-10, 10));
    EXPECT_EQ(second[i], math::Rand::GetIntUniform(-10, 10));
  }
}

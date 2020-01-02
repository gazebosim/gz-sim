/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include "ignition/math/RollingMean.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(RollingMeanTest, RollingMean)
{
  math::RollingMean mean;
  EXPECT_EQ(0u, mean.Count());
  EXPECT_EQ(10u, mean.WindowSize());

  mean.SetWindowSize(4);
  EXPECT_EQ(4u, mean.WindowSize());
  mean.SetWindowSize(0);
  EXPECT_EQ(4u, mean.WindowSize());

  mean.Push(1.0);
  EXPECT_DOUBLE_EQ(1.0, mean.Mean());
  mean.Push(2.0);
  EXPECT_DOUBLE_EQ(1.5, mean.Mean());
  mean.Push(3.0);
  EXPECT_DOUBLE_EQ(2.0, mean.Mean());
  mean.Push(10.0);
  EXPECT_DOUBLE_EQ(4.0, mean.Mean());
  mean.Push(20.0);
  EXPECT_DOUBLE_EQ(8.75, mean.Mean());

  mean.Clear();
  EXPECT_TRUE(math::isnan(mean.Mean()));

  mean.Push(100.0);
  mean.Push(200.0);
  mean.Push(300.0);
  EXPECT_EQ(3u, mean.Count());
  mean.SetWindowSize(2);
  EXPECT_EQ(0u, mean.Count());
}

/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "gz/math/GaussMarkovProcess.hh"
#include "gz/math/Rand.hh"

using namespace gz;
using namespace math;

/////////////////////////////////////////////////
TEST(GaussMarkovProcessTest, DefaultConstructor)
{
  GaussMarkovProcess gmp;
  EXPECT_DOUBLE_EQ(0.0, gmp.Start());
  EXPECT_DOUBLE_EQ(0.0, gmp.Value());
  EXPECT_DOUBLE_EQ(0.0, gmp.Theta());
  EXPECT_DOUBLE_EQ(0.0, gmp.Mu());
  EXPECT_DOUBLE_EQ(0.0, gmp.Sigma());
  // EXPECT_EQ(clock::time_point(), gmp.LastUpdate());
}

/////////////////////////////////////////////////
TEST(GaussMarkovProcessTest, Update)
{
  // Start value of -1.2
  // Theta (rate at which the process should approach the mean) of 1.0
  // Mu (mean value) 2.5.
  // Sigma (volatility) of 0.
  GaussMarkovProcess gmp(-1.2, 1.0, 2.5, 0);
  EXPECT_DOUBLE_EQ(-1.2, gmp.Start());
  EXPECT_DOUBLE_EQ(-1.2, gmp.Value());
  EXPECT_DOUBLE_EQ(1.0, gmp.Theta());
  EXPECT_DOUBLE_EQ(2.5, gmp.Mu());
  EXPECT_DOUBLE_EQ(0.0, gmp.Sigma());
  // EXPECT_EQ(clock::time_point(), gmp.LastUpdate());

  clock::duration dt = std::chrono::milliseconds(100);

  // This process should steadily increase to the mean value of 2.5 since
  // there is no noise.
  for (int i = 0; i < 200; ++i)
  {
    double value = gmp.Update(dt);
    EXPECT_GT(value, -1.2);
  }

  EXPECT_NEAR(2.5, gmp.Value(), 1e-4);

  // The same should occur after a reset.
  gmp.Reset();
  for (int i = 0; i < 200; ++i)
  {
    double value = gmp.Update(dt);
    EXPECT_GT(value, -1.2);
  }

  EXPECT_NEAR(2.5, gmp.Value(), 1e-4);
}

/////////////////////////////////////////////////
TEST(GaussMarkovProcessTest, Noise)
{
  // Start value of 20.2
  // Theta (rate at which the process should approach the mean) of 0.1
  // Mu (mean value) 0.
  // Sigma (volatility) of 0.5.
  GaussMarkovProcess gmp(20.2, 0.1, 0, 0.5);
  clock::duration dt = std::chrono::milliseconds(100);
  Rand::Seed(1001);

  // This process should decrease toward the mean value of 0. With noise,
  // the process will walk.
  for (int i = 0; i < 1000; ++i)
  {
    double value = gmp.Update(dt);
    // Hand-tuned values that are repeatable given the seed set above.
    EXPECT_GT(value, -11);
    EXPECT_LT(value, 22);
  }
  // Hand-tuned values that are repeatable given the seed set above.
#ifdef __APPLE__
  EXPECT_NEAR(-4.960893, gmp.Value(), 1e-4);
#elif _MSC_VER
  EXPECT_NEAR(-4.960893, gmp.Value(), 1e-4);
#else
  EXPECT_NEAR(-4.118732, gmp.Value(), 1e-4);
#endif
}

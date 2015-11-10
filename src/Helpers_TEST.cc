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

using namespace ignition;

/////////////////////////////////////////////////
// Test a few function in Helpers
TEST(HelpersTest, Helpers)
{
  EXPECT_EQ(12345, math::parseInt("12345"));
  EXPECT_EQ(-12345, math::parseInt("-12345"));
  EXPECT_EQ(-12345, math::parseInt("    -12345"));
  EXPECT_EQ(0, math::parseInt("    "));

  EXPECT_EQ(math::NAN_I, math::parseInt(""));
  EXPECT_EQ(math::NAN_I, math::parseInt("?"));
  EXPECT_EQ(math::NAN_I, math::parseInt("23ab67"));

  EXPECT_FLOAT_EQ(12.345, math::parseFloat("12.345"));
  EXPECT_FLOAT_EQ(-12.345, math::parseFloat("-12.345"));
  EXPECT_FLOAT_EQ(-12.345, math::parseFloat("    -12.345"));
  EXPECT_FLOAT_EQ(0.0, math::parseFloat("    "));
  EXPECT_TRUE(math::equal(123.45, math::parseFloat("1.2345e2"), 1e-2));
  EXPECT_TRUE(math::equal(123.45, math::parseFloat("1.2345e+2"), 1e-2));
  EXPECT_TRUE(math::equal(123.45, math::parseFloat("1.2345e+002"), 1e-2));
  EXPECT_TRUE(math::equal(.012345, math::parseFloat("1.2345e-2"), 1e-2));
  EXPECT_TRUE(math::equal(.012345, math::parseFloat("1.2345e-002"), 1e-2));
  EXPECT_TRUE(math::equal(1.2345, math::parseFloat("1.2345e+"), 1e-2));
  EXPECT_TRUE(math::equal(1.2345, math::parseFloat("1.2345e-"), 1e-2));
  EXPECT_DOUBLE_EQ(1.2345, math::parseFloat("1.2345e+0"));

  EXPECT_TRUE(math::isnan(math::parseFloat("")));
  EXPECT_TRUE(math::isnan(math::parseFloat("?")));
  EXPECT_TRUE(math::isnan(math::parseFloat("23ab67")));

  EXPECT_EQ(1u, math::roundUpPowerOfTwo(0));
  EXPECT_EQ(1u, math::roundUpPowerOfTwo(1));
  EXPECT_EQ(2u, math::roundUpPowerOfTwo(2));
  EXPECT_EQ(2048u, math::roundUpPowerOfTwo(1025));
}

/////////////////////////////////////////////////
// Test Helpers::precision
TEST(HelpersTest, Precision)
{
  EXPECT_DOUBLE_EQ(0, math::precision(0.0, 1));
  EXPECT_DOUBLE_EQ(0.1, math::precision(0.1, 1));
  EXPECT_DOUBLE_EQ(0.1, math::precision(0.14, 1));
  EXPECT_DOUBLE_EQ(0.2, math::precision(0.15, 1));
  EXPECT_DOUBLE_EQ(0.15, math::precision(0.15, 2));

  EXPECT_DOUBLE_EQ(1, math::precision(1.4, 0));
  EXPECT_EQ(0, math::precision(0, 0));
}

/////////////////////////////////////////////////
// Test Helpers::isPowerOfTwo
TEST(HelpersTest, PowerOfTwo)
{
  EXPECT_FALSE(math::isPowerOfTwo(0));
  EXPECT_FALSE(math::isPowerOfTwo(3));

  EXPECT_TRUE(math::isPowerOfTwo(1));

  EXPECT_TRUE(math::isPowerOfTwo(2));
  EXPECT_TRUE(math::isPowerOfTwo(4));
}

// MSVC report errors on division by zero
#ifndef _MSC_VER
/////////////////////////////////////////////////
// Test Helpers::fixnan functions
TEST(HelpersTest, FixNaN)
{
  EXPECT_DOUBLE_EQ(math::fixnan(1.0 / 0.0), 0.0);
  EXPECT_DOUBLE_EQ(math::fixnan(-1.0 / 0.0), 0.0);
  EXPECT_DOUBLE_EQ(math::fixnan(0.0 / 0.0), 0.0);

  EXPECT_DOUBLE_EQ(math::fixnan(42.0), 42.0);
  EXPECT_DOUBLE_EQ(math::fixnan(-42.0), -42.0);

  EXPECT_FLOAT_EQ(math::fixnan(1.0f / 0.0f), 0.0f);
  EXPECT_FLOAT_EQ(math::fixnan(-1.0f / 0.0f), 0.0f);
  EXPECT_FLOAT_EQ(math::fixnan(0.0f / 0.0f), 0.0f);

  EXPECT_FLOAT_EQ(math::fixnan(42.0f), 42.0f);
  EXPECT_FLOAT_EQ(math::fixnan(-42.0f), -42.0f);
}
#endif

/////////////////////////////////////////////////
// Even test
TEST(HelpersTest, Even)
{
  int i = 1;
  signed s = 1;
  signed int si = 1;
  unsigned u = 1;
  unsigned int ui = 1;

  EXPECT_FALSE(math::isEven(i));
  EXPECT_FALSE(math::isEven(s));
  EXPECT_FALSE(math::isEven(si));
  EXPECT_FALSE(math::isEven(u));
  EXPECT_FALSE(math::isEven(ui));

  i = -1;
  s = -1;
  si = -1;
  u = -1;
  ui = -1;

  EXPECT_FALSE(math::isEven(i));
  EXPECT_FALSE(math::isEven(s));
  EXPECT_FALSE(math::isEven(si));
  EXPECT_FALSE(math::isEven(u));
  EXPECT_FALSE(math::isEven(ui));

  i = 4;
  s = 4;
  si = 4;
  u = 4;
  ui = 4;

  EXPECT_TRUE(math::isEven(i));
  EXPECT_TRUE(math::isEven(s));
  EXPECT_TRUE(math::isEven(si));
  EXPECT_TRUE(math::isEven(u));
  EXPECT_TRUE(math::isEven(ui));

  i = -2;
  s = -2;
  si = -2;
  u = -2;
  ui = -2;

  EXPECT_TRUE(math::isEven(i));
  EXPECT_TRUE(math::isEven(s));
  EXPECT_TRUE(math::isEven(si));
  EXPECT_TRUE(math::isEven(u));
  EXPECT_TRUE(math::isEven(ui));

  i = 0;
  s = 0;
  si = 0;
  u = 0;
  ui = 0;

  EXPECT_TRUE(math::isEven(i));
  EXPECT_TRUE(math::isEven(s));
  EXPECT_TRUE(math::isEven(si));
  EXPECT_TRUE(math::isEven(u));
  EXPECT_TRUE(math::isEven(ui));
}

/////////////////////////////////////////////////
// Odd test
TEST(HelpersTest, Odd)
{
  int i = 1;
  signed s = 1;
  signed int si = 1;
  unsigned u = 1;
  unsigned int ui = 1;

  EXPECT_TRUE(math::isOdd(i));
  EXPECT_TRUE(math::isOdd(s));
  EXPECT_TRUE(math::isOdd(si));
  EXPECT_TRUE(math::isOdd(u));
  EXPECT_TRUE(math::isOdd(ui));

  i = -1;
  s = -1;
  si = -1;
  u = -1;
  ui = -1;

  EXPECT_TRUE(math::isOdd(i));
  EXPECT_TRUE(math::isOdd(s));
  EXPECT_TRUE(math::isOdd(si));
  EXPECT_TRUE(math::isOdd(u));
  EXPECT_TRUE(math::isOdd(ui));

  i = 4;
  s = 4;
  si = 4;
  u = 4;
  ui = 4;

  EXPECT_FALSE(math::isOdd(i));
  EXPECT_FALSE(math::isOdd(s));
  EXPECT_FALSE(math::isOdd(si));
  EXPECT_FALSE(math::isOdd(u));
  EXPECT_FALSE(math::isOdd(ui));

  i = -2;
  s = -2;
  si = -2;
  u = -2;
  ui = -2;

  EXPECT_FALSE(math::isOdd(i));
  EXPECT_FALSE(math::isOdd(s));
  EXPECT_FALSE(math::isOdd(si));
  EXPECT_FALSE(math::isOdd(u));
  EXPECT_FALSE(math::isOdd(ui));

  i = 0;
  s = 0;
  si = 0;
  u = 0;
  ui = 0;

  EXPECT_FALSE(math::isOdd(i));
  EXPECT_FALSE(math::isOdd(s));
  EXPECT_FALSE(math::isOdd(si));
  EXPECT_FALSE(math::isOdd(u));
  EXPECT_FALSE(math::isOdd(ui));
}

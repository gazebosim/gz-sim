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

#include "ignition/math/Rand.hh"
#include "ignition/math/Vector3.hh"
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
  EXPECT_EQ(23, math::parseInt("23ab67"));

  EXPECT_EQ(math::NAN_I, math::parseInt(""));
  EXPECT_EQ(math::NAN_I, math::parseInt("?"));
  EXPECT_EQ(math::NAN_I, math::parseInt("ab23ab67"));

  EXPECT_DOUBLE_EQ(12.345, math::parseFloat("12.345"));
  EXPECT_DOUBLE_EQ(-12.345, math::parseFloat("-12.345"));
  EXPECT_DOUBLE_EQ(-12.345, math::parseFloat("    -12.345"));
  EXPECT_DOUBLE_EQ(0.0, math::parseFloat("    "));
  EXPECT_TRUE(math::equal(123.45, math::parseFloat("1.2345e2"), 1e-2));
  EXPECT_TRUE(math::equal(123.45, math::parseFloat("1.2345e+2"), 1e-2));
  EXPECT_TRUE(math::equal(123.45, math::parseFloat("1.2345e+002"), 1e-2));
  EXPECT_TRUE(math::equal(.012345, math::parseFloat("1.2345e-2"), 1e-2));
  EXPECT_TRUE(math::equal(.012345, math::parseFloat("1.2345e-002"), 1e-2));
  EXPECT_TRUE(math::equal(1.2345, math::parseFloat("1.2345e+"), 1e-2));
  EXPECT_TRUE(math::equal(1.2345, math::parseFloat("1.2345e-"), 1e-2));
  EXPECT_TRUE(math::lessOrNearEqual(1.0, 2.0, 1e-2));
  EXPECT_TRUE(math::lessOrNearEqual(1.0, 1.0 - 9e-3, 1e-2));
  EXPECT_FALSE(math::lessOrNearEqual(1.0, 1.0 - 1.1e-2, 1e-2));
  EXPECT_TRUE(math::greaterOrNearEqual(1.0, 0.5, 1e-2));
  EXPECT_TRUE(math::greaterOrNearEqual(1.0, 1.0 + 9e-3, 1e-2));
  EXPECT_FALSE(math::greaterOrNearEqual(1.0, 1.0 + 1.1e-2, 1e-2));
  EXPECT_DOUBLE_EQ(1.2345, math::parseFloat("1.2345e+0"));
  EXPECT_DOUBLE_EQ(23.0, math::parseFloat("23ab67"));

  EXPECT_TRUE(math::isnan(math::parseFloat("")));
  EXPECT_TRUE(math::isnan(math::parseFloat("?")));
  EXPECT_TRUE(math::isnan(math::parseFloat("ab23ab67")));

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
  EXPECT_DOUBLE_EQ(math::fixnan(1.0 / 0.0), 0.0);

  EXPECT_DOUBLE_EQ(math::fixnan(42.0), 42.0);
  EXPECT_DOUBLE_EQ(math::fixnan(-42.0), -42.0);

  EXPECT_FLOAT_EQ(math::fixnan(1.0f / 0.0f), 0.0f);
  EXPECT_FLOAT_EQ(math::fixnan(-1.0f / 0.0f), 0.0f);
  EXPECT_FLOAT_EQ(math::fixnan(1.0f / 0.0f), 0.0f);

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

  EXPECT_FALSE(math::isEven(i));
  EXPECT_FALSE(math::isEven(s));
  EXPECT_FALSE(math::isEven(si));

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

  EXPECT_TRUE(math::isEven(i));
  EXPECT_TRUE(math::isEven(s));
  EXPECT_TRUE(math::isEven(si));

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

  EXPECT_TRUE(math::isOdd(i));
  EXPECT_TRUE(math::isOdd(s));
  EXPECT_TRUE(math::isOdd(si));

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

  EXPECT_FALSE(math::isOdd(i));
  EXPECT_FALSE(math::isOdd(s));
  EXPECT_FALSE(math::isOdd(si));

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

/////////////////////////////////////////////////
// Signum test
TEST(HelpersTest, Signum)
{
  int i = 1;
  signed s = 1;
  signed int si = 1;
  unsigned u = 1;
  unsigned int ui = 1;
  float f = 1.f;
  double d = 1.;

  EXPECT_EQ(1, math::signum(i));
  EXPECT_EQ(1, math::signum(s));
  EXPECT_EQ(1, math::signum(si));
  EXPECT_EQ(1, math::signum(u));
  EXPECT_EQ(1, math::signum(ui));
  EXPECT_EQ(1, math::signum(f));
  EXPECT_EQ(1, math::signum(d));

  i = s = si = u = ui = 2;
  f = 2.f;
  d = 2.;

  EXPECT_EQ(1, math::signum(i));
  EXPECT_EQ(1, math::signum(s));
  EXPECT_EQ(1, math::signum(si));
  EXPECT_EQ(1, math::signum(u));
  EXPECT_EQ(1, math::signum(ui));
  EXPECT_EQ(1, math::signum(f));
  EXPECT_EQ(1, math::signum(d));

  i = s = si = u = ui = 0;
  f = 0.f;
  d = 0.;

  EXPECT_EQ(0, math::signum(i));
  EXPECT_EQ(0, math::signum(s));
  EXPECT_EQ(0, math::signum(si));
  EXPECT_EQ(0, math::signum(u));
  EXPECT_EQ(0, math::signum(ui));
  EXPECT_EQ(0, math::signum(f));
  EXPECT_EQ(0, math::signum(d));

  i = s = si = -1;
  f = -1.f;
  d = -1.;

  EXPECT_EQ(-1, math::signum(i));
  EXPECT_EQ(-1, math::signum(s));
  EXPECT_EQ(-1, math::signum(si));
  EXPECT_EQ(-1, math::signum(f));
  EXPECT_EQ(-1, math::signum(d));

  i = s = si = -2;
  f = -2.f;
  d = -2.;

  EXPECT_EQ(-1, math::signum(i));
  EXPECT_EQ(-1, math::signum(s));
  EXPECT_EQ(-1, math::signum(si));
  EXPECT_EQ(-1, math::signum(f));
  EXPECT_EQ(-1, math::signum(d));

  f = -2.5f;
  d = -2.5;

  EXPECT_EQ(-1, math::signum(f));
  EXPECT_EQ(-1, math::signum(d));

  f = 2.5f;
  d = 2.5;

  EXPECT_EQ(1, math::signum(f));
  EXPECT_EQ(1, math::signum(d));

  f = 1e-10f;
  d = 1e-10;

  EXPECT_EQ(1, math::signum(f));
  EXPECT_EQ(1, math::signum(d));
}

/////////////////////////////////////////////////
TEST(HelpersTest, Stats)
{
  {
    const std::vector<int> valuesI{0, 10, 20};
    EXPECT_EQ(20, math::max(valuesI));
    EXPECT_EQ(0, math::min(valuesI));
    EXPECT_EQ(10, math::mean(valuesI));
    EXPECT_EQ(66, math::variance(valuesI));
  }

  {
    const std::vector<double> valuesD{0., 10., 20.};
    EXPECT_DOUBLE_EQ(20., math::max(valuesD));
    EXPECT_DOUBLE_EQ(0., math::min(valuesD));
    EXPECT_DOUBLE_EQ(10., math::mean(valuesD));
    EXPECT_DOUBLE_EQ(66.666666666666671, math::variance(valuesD));
  }
}

/////////////////////////////////////////////////
TEST(HelpersTest, Sort)
{
  {
    int a = 2;
    int b = -1;
    math::sort2(a, b);
    EXPECT_LE(a, b);
  }

  {
    int a = 0;
    int b = 1;
    math::sort2(a, b);
    EXPECT_LE(a, b);
  }

  {
    int a = 2;
    int b = -1;
    int c = 0;
    math::sort3(a, b, c);
    EXPECT_LE(a, b);
    EXPECT_LE(b, c);
  }

  {
    unsigned int a = 2;
    unsigned int b = 1;
    math::sort2(a, b);
    EXPECT_LE(a, b);
  }

  {
    unsigned int a = 2;
    unsigned int b = 1;
    unsigned int c = 0;
    math::sort3(a, b, c);
    EXPECT_LE(a, b);
    EXPECT_LE(b, c);
  }
  {
    unsigned int a = 0;
    unsigned int b = 1;
    unsigned int c = 2;
    math::sort3(a, b, c);
    EXPECT_LE(a, b);
    EXPECT_LE(b, c);
  }


  {
    float a = 2.1f;
    float b = -1.1e-1f;
    math::sort2(a, b);
    EXPECT_LE(a, b);
  }

  {
    float a = 34.5f;
    float b = -1.34f;
    float c = 0.194f;
    math::sort3(a, b, c);
    EXPECT_LE(a, b);
    EXPECT_LE(b, c);
  }

  {
    double a = 2.1;
    double b = -1.1e-1;
    math::sort2(a, b);
    EXPECT_LE(a, b);
  }

  {
    double a = 34.5;
    double b = -1.34;
    double c = 0.194;
    math::sort3(a, b, c);
    EXPECT_LE(a, b);
    EXPECT_LE(b, c);
  }
}

/////////////////////////////////////////////////
TEST(HelpersTest, Volume)
{
  EXPECT_DOUBLE_EQ(IGN_SPHERE_VOLUME(1.0), 4.0*IGN_PI*std::pow(1, 3)/3.0);
  EXPECT_DOUBLE_EQ(IGN_SPHERE_VOLUME(0.1), 4.0*IGN_PI*std::pow(.1, 3)/3.0);
  EXPECT_DOUBLE_EQ(IGN_SPHERE_VOLUME(-1.1), 4.0*IGN_PI*std::pow(-1.1, 3)/3.0);

  EXPECT_DOUBLE_EQ(IGN_CYLINDER_VOLUME(0.5, 2.0), 2 * IGN_PI * std::pow(.5, 2));
  EXPECT_DOUBLE_EQ(IGN_CYLINDER_VOLUME(1, -1), -1 * IGN_PI * std::pow(1, 2));

  EXPECT_DOUBLE_EQ(IGN_BOX_VOLUME(1, 2, 3), 1 * 2 * 3);
  EXPECT_DOUBLE_EQ(IGN_BOX_VOLUME(.1, .2, .3),
                   IGN_BOX_VOLUME_V(math::Vector3d(0.1, 0.2, 0.3)));
}

/////////////////////////////////////////////////
TEST(HelpersTest, Pair)
{
#if defined _MSC_VER || defined __arm__
  math::PairInput maxA = math::MAX_UI16;
  math::PairInput maxB = math::MAX_UI16;
#else
  math::PairInput maxA = math::MAX_UI32;
  math::PairInput maxB = math::MAX_UI32;
#endif

  math::PairInput maxC, maxD;

  // Maximum parameters should generate a maximum key
  math::PairOutput maxKey = math::Pair(maxA, maxB);
#if defined _MSC_VER || defined __arm__
  EXPECT_EQ(maxKey, math::MAX_UI32);
#else
  EXPECT_EQ(maxKey, math::MAX_UI64);
#endif

  std::tie(maxC, maxD) = math::Unpair(maxKey);
  EXPECT_EQ(maxC, maxA);
  EXPECT_EQ(maxD, maxB);

#if defined _MSC_VER || defined __arm__
  math::PairInput minA = math::MIN_UI16;
  math::PairInput minB = math::MIN_UI16;
#else
  math::PairInput minA = math::MIN_UI32;
  math::PairInput minB = math::MIN_UI32;
#endif
  math::PairInput minC, minD;

  // Minimum parameters should generate a minimum key
  math::PairOutput minKey = math::Pair(minA, minB);
#if defined _MSC_VER || defined __arm__
  EXPECT_EQ(minKey, math::MIN_UI32);
#else
  EXPECT_EQ(minKey, math::MIN_UI64);
#endif

  std::tie(minC, minD) = math::Unpair(minKey);
  EXPECT_EQ(minC, minA);
  EXPECT_EQ(minD, minB);

  // Max key != min key
  EXPECT_TRUE(minKey != maxKey);

  // Just a simple test case
  {
    unsigned int a = 10;
    unsigned int b = 20;
    math::PairInput c, d;

    auto key = math::Pair(static_cast<math::PairInput>(a),
                          static_cast<math::PairInput>(b));
    EXPECT_EQ(key, 410u);
    EXPECT_TRUE(key != maxKey);
    EXPECT_TRUE(key != minKey);

    std::tie(c, d) = math::Unpair(key);
    EXPECT_EQ(c, a);
    EXPECT_EQ(d, b);
  }

  {
    math::PairInput c, d;
    std::set<math::PairOutput> set;

    // Iterate over range of pairs, and check for unique keys.
    for (uint16_t a = math::MIN_UI16; a < math::MAX_UI16 - 500;
         a += static_cast<uint16_t>(math::Rand::IntUniform(100, 500)))
    {
      for (uint16_t b = math::MIN_UI16; b < math::MAX_UI16 - 500;
         b += static_cast<uint16_t>(math::Rand::IntUniform(100, 500)))
      {
        math::PairOutput key = math::Pair(a, b);
        std::tie(c, d) = math::Unpair(key);
        EXPECT_EQ(a, c);
        EXPECT_EQ(b, d);
        EXPECT_TRUE(set.find(key) == set.end());
        EXPECT_TRUE(key != maxKey);
        set.insert(key);
      }
    }

#if !defined _MSC_VER && !defined __arm__
    // Iterate over large numbers, and check for unique keys.
    for (math::PairInput a = math::MAX_UI32-5000; a < math::MAX_UI32; a++)
    {
      for (math::PairInput b = math::MAX_UI32-5000; b < math::MAX_UI32; b++)
      {
        math::PairOutput key = math::Pair(a, b);
        std::tie(c, d) = math::Unpair(key);
        EXPECT_EQ(a, c);
        EXPECT_EQ(b, d);
        EXPECT_TRUE(set.find(key) == set.end());
        EXPECT_TRUE(key != minKey);
        set.insert(key);
      }
    }
#endif
  }
}

/////////////////////////////////////////////////
TEST(HelpersTest, timePointToSecNsec)
{
  std::pair<int64_t, int64_t> parts = math::timePointToSecNsec(
      math::secNsecToTimePoint(0, 0));
  EXPECT_EQ(parts.first, 0);
  EXPECT_EQ(parts.second, 0);

  std::chrono::steady_clock::time_point point;
  point += std::chrono::nanoseconds(1000);
  parts = math::timePointToSecNsec(point);

  EXPECT_EQ(parts.first, 0);
  EXPECT_EQ(parts.second, 1000);

  point = math::secNsecToTimePoint(0, 0);
  point += std::chrono::seconds(60);
  point += std::chrono::nanoseconds(57989);
  parts = math::timePointToSecNsec(point);

  EXPECT_EQ(parts.first, 60);
  EXPECT_EQ(parts.second, 57989);
}

/////////////////////////////////////////////////
TEST(HelpersTest, secNsecToTimePoint)
{
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  using std::chrono::steady_clock;

  std::chrono::steady_clock::time_point point =
    math::secNsecToTimePoint(0, 0);
  point += std::chrono::hours(24);

  std::chrono::steady_clock::time_point s =
    math::secNsecToTimePoint(24*60*60, 0);
  EXPECT_EQ(s, point);

  point = math::secNsecToTimePoint(0, 0);
  point += std::chrono::nanoseconds(1000);
  s = math::secNsecToTimePoint(0, 1000);
  EXPECT_EQ(s, point);
}

/////////////////////////////////////////////////
TEST(HelpersTest, timePointToString)
{
  std::chrono::steady_clock::time_point time_clock =
    math::secNsecToTimePoint(0, 0);
  std::string s = math::timePointToString(time_clock);

  EXPECT_STREQ(s.c_str(), std::string("00 00:00:00.000").c_str());

  std::chrono::steady_clock::time_point point;
  point += std::chrono::hours(24);

  s = math::timePointToString(point);
  EXPECT_STREQ(s.c_str(), std::string("01 00:00:00.000").c_str());

  point = math::secNsecToTimePoint(0, 0);
  point += std::chrono::minutes(1);
  point += std::chrono::seconds(23);
  point += std::chrono::milliseconds(125);
  s = math::timePointToString(point);
  EXPECT_STREQ(s.c_str(), std::string("00 00:01:23.125").c_str());
}

/////////////////////////////////////////////////
TEST(HelpersTest, durationToString)
{
  std::chrono::steady_clock::duration duration =
    std::chrono::steady_clock::duration::zero();
  std::string s = math::durationToString(duration);

  EXPECT_STREQ(s.c_str(), std::string("00 00:00:00.000").c_str());

  std::chrono::steady_clock::duration duration1 =
    std::chrono::steady_clock::duration::zero();
  duration1 += std::chrono::hours(24);

  s = math::durationToString(duration1);
  EXPECT_STREQ(s.c_str(), std::string("01 00:00:00.000").c_str());

  duration1 = std::chrono::steady_clock::duration::zero();;
  duration1 += std::chrono::minutes(1);
  duration1 += std::chrono::seconds(23);
  duration1 += std::chrono::milliseconds(125);
  s = math::durationToString(duration1);
  EXPECT_STREQ(s.c_str(), std::string("00 00:01:23.125").c_str());
}

/////////////////////////////////////////////////
TEST(HelpersTest, stringToDuration)
{
  std::string time = "0 00:00:00.000";
  std::chrono::steady_clock::duration resultTime =
    math::stringToDuration(time);
  std::chrono::steady_clock::duration duration =
    std::chrono::steady_clock::duration::zero();

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = "10 0";
  resultTime = math::stringToDuration(time);
  duration = std::chrono::steady_clock::duration::zero();
  duration += std::chrono::hours(10 * 24);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = "7";
  resultTime = math::stringToDuration(time);
  duration = std::chrono::steady_clock::duration::zero();
  duration += std::chrono::seconds(7);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = "7:10";
  resultTime = math::stringToDuration(time);
  duration = std::chrono::steady_clock::duration::zero();
  duration += std::chrono::minutes(7);
  duration += std::chrono::seconds(10);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = "17:10";
  resultTime = math::stringToDuration(time);
  duration = std::chrono::steady_clock::duration::zero();
  duration += std::chrono::minutes(17);
  duration += std::chrono::seconds(10);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = "7:10.4";
  resultTime = math::stringToDuration(time);
  duration = std::chrono::steady_clock::duration::zero();
  duration += std::chrono::minutes(7);
  duration += std::chrono::seconds(10);
  duration += std::chrono::milliseconds(400);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = "7:10.45";
  resultTime = math::stringToDuration(time);
  duration = std::chrono::steady_clock::duration::zero();
  duration += std::chrono::minutes(7);
  duration += std::chrono::seconds(10);
  duration += std::chrono::milliseconds(450);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = "7:10.456";
  resultTime = math::stringToDuration(time);
  duration = std::chrono::steady_clock::duration::zero();
  duration += std::chrono::minutes(7);
  duration += std::chrono::seconds(10);
  duration += std::chrono::milliseconds(456);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = "2 23:18:25.902";
  resultTime = math::stringToDuration(time);
  duration = std::chrono::steady_clock::duration::zero();
  duration += std::chrono::hours(2 * 24);
  duration += std::chrono::hours(23);
  duration += std::chrono::minutes(18);
  duration += std::chrono::seconds(25);
  duration += std::chrono::milliseconds(902);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = ".9";
  resultTime = math::stringToDuration(time);
  duration = std::chrono::steady_clock::duration::zero();
  duration += std::chrono::milliseconds(900);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, duration);

  time = "bad time";
  resultTime = math::stringToDuration(time);

  EXPECT_FALSE(math::isTimeString(time));
  EXPECT_EQ(resultTime, std::chrono::steady_clock::duration::zero());

  time = "";
  resultTime = math::stringToDuration(time);

  EXPECT_TRUE(math::isTimeString(time));
  EXPECT_EQ(resultTime, std::chrono::steady_clock::duration::zero());

  time = "60";
  resultTime = math::stringToDuration(time);

  EXPECT_FALSE(math::isTimeString(time));
  EXPECT_EQ(resultTime, std::chrono::steady_clock::duration::zero());

  time = "60:12";
  resultTime = math::stringToDuration(time);

  EXPECT_FALSE(math::isTimeString(time));
  EXPECT_EQ(resultTime, std::chrono::steady_clock::duration::zero());

  time = "12:12.9999";
  resultTime = math::stringToDuration(time);

  EXPECT_FALSE(math::isTimeString(time));
  EXPECT_EQ(resultTime, std::chrono::steady_clock::duration::zero());

  time = "25:12:12.99";
  resultTime = math::stringToDuration(time);

  EXPECT_FALSE(math::isTimeString(time));
  EXPECT_EQ(resultTime, std::chrono::steady_clock::duration::zero());

  time = "999999999999999 5:12:12.5";
  resultTime = math::stringToDuration(time);

  EXPECT_FALSE(math::isTimeString(time));
  EXPECT_EQ(resultTime, std::chrono::steady_clock::duration::zero());
}

/////////////////////////////////////////////////
TEST(HelpersTest, secNsecToDuration)
{
  std::chrono::steady_clock::duration point =
    std::chrono::steady_clock::duration::zero();
  point += std::chrono::hours(24);

  std::chrono::steady_clock::duration s =
    math::secNsecToDuration(24*60*60, 0);
  EXPECT_EQ(s, point);

  point = std::chrono::steady_clock::duration::zero();
  point += std::chrono::nanoseconds(1000);
  s = math::secNsecToDuration(0, 1000);
  EXPECT_EQ(s, point);
}

/////////////////////////////////////////////////
TEST(HelpersTest, stringToTimePoint)
{
  using namespace std::chrono_literals;

  std::chrono::steady_clock::time_point zeroTime{0s};
  std::chrono::steady_clock::time_point negTime{-1s};

  std::string time = "0 00:00:00.000";
  std::chrono::steady_clock::time_point resultTime =
    math::stringToTimePoint(time);
  std::chrono::steady_clock::time_point point = zeroTime;

  EXPECT_EQ(resultTime, point);

  time = "10 0";
  resultTime = math::stringToTimePoint(time);
  point = zeroTime;
  point += std::chrono::hours(10 * 24);

  EXPECT_EQ(resultTime, point);

  time = "7";
  resultTime = math::stringToTimePoint(time);
  point = zeroTime;
  point += std::chrono::seconds(7);

  EXPECT_EQ(resultTime, point);

  time = "7:10";
  resultTime = math::stringToTimePoint(time);
  point = zeroTime;
  point += std::chrono::minutes(7);
  point += std::chrono::seconds(10);

  EXPECT_EQ(resultTime, point);

  time = "17:10";
  resultTime = math::stringToTimePoint(time);
  point = zeroTime;
  point += std::chrono::minutes(17);
  point += std::chrono::seconds(10);

  EXPECT_EQ(resultTime, point);

  time = "7:10.4";
  resultTime = math::stringToTimePoint(time);
  point = zeroTime;
  point += std::chrono::minutes(7);
  point += std::chrono::seconds(10);
  point += std::chrono::milliseconds(400);

  EXPECT_EQ(resultTime, point);

  time = "7:10.45";
  resultTime = math::stringToTimePoint(time);
  point = zeroTime;
  point += std::chrono::minutes(7);
  point += std::chrono::seconds(10);
  point += std::chrono::milliseconds(450);

  EXPECT_EQ(resultTime, point);

  time = "7:10.456";
  resultTime = math::stringToTimePoint(time);
  point = zeroTime;
  point += std::chrono::minutes(7);
  point += std::chrono::seconds(10);
  point += std::chrono::milliseconds(456);

  EXPECT_EQ(resultTime, point);

  time = "2 23:18:25.902";
  resultTime = math::stringToTimePoint(time);
  point = zeroTime;
  point += std::chrono::hours(2 * 24);
  point += std::chrono::hours(23);
  point += std::chrono::minutes(18);
  point += std::chrono::seconds(25);
  point += std::chrono::milliseconds(902);

  EXPECT_EQ(resultTime, point);

  time = ".9";
  resultTime = math::stringToTimePoint(time);
  point = zeroTime;
  point += std::chrono::milliseconds(900);

  EXPECT_EQ(resultTime, point);

  time = "bad time";
  resultTime = math::stringToTimePoint(time);

  EXPECT_EQ(resultTime, negTime);

  time = "";
  resultTime = math::stringToTimePoint(time);

  EXPECT_EQ(resultTime, negTime);

  time = "60";
  resultTime = math::stringToTimePoint(time);

  EXPECT_EQ(resultTime, negTime);

  time = "60:12";
  resultTime = math::stringToTimePoint(time);

  EXPECT_EQ(resultTime, negTime);

  time = "12:12.9999";
  resultTime = math::stringToTimePoint(time);

  EXPECT_EQ(resultTime, negTime);

  time = "25:12:12.99";
  resultTime = math::stringToTimePoint(time);

  EXPECT_EQ(resultTime, negTime);

  time = "999999999999999 5:12:12.5";
  resultTime = math::stringToTimePoint(time);

  EXPECT_EQ(resultTime, negTime);
}

/////////////////////////////////////////////////
TEST(HelpersTest, durationToSecNsec)
{
  std::pair<int64_t, int64_t> parts;

  parts = math::durationToSecNsec(std::chrono::nanoseconds(9834249021));
  EXPECT_EQ(9, parts.first);
  EXPECT_EQ(834249021, parts.second);

  parts = math::durationToSecNsec(std::chrono::nanoseconds(-9834249021));
  EXPECT_EQ(-9, parts.first);
  EXPECT_EQ(-834249021, parts.second);

  parts = math::durationToSecNsec(std::chrono::nanoseconds());
  EXPECT_EQ(0, parts.first);
  EXPECT_EQ(0, parts.second);

  parts = math::durationToSecNsec(std::chrono::nanoseconds(1234));
  EXPECT_EQ(0, parts.first);
  EXPECT_EQ(1234, parts.second);

  parts = math::durationToSecNsec(std::chrono::seconds(89743));
  EXPECT_EQ(89743, parts.first);
  EXPECT_EQ(0, parts.second);

  parts = math::durationToSecNsec(std::chrono::seconds(-1));
  EXPECT_EQ(-1, parts.first);
  EXPECT_EQ(0, parts.second);

  parts = math::durationToSecNsec(std::chrono::milliseconds(3487));
  EXPECT_EQ(3, parts.first);
  EXPECT_EQ(487000000, parts.second);
}

/////////////////////////////////////////////////
TEST(HelpersTest, roundUpMultiple)
{
  EXPECT_EQ(0, math::roundUpMultiple(0, 0));
  EXPECT_EQ(12, math::roundUpMultiple(12, 0));

  EXPECT_EQ(1, math::roundUpMultiple(1, 1));
  EXPECT_EQ(100, math::roundUpMultiple(100, 10));
  EXPECT_EQ(48, math::roundUpMultiple(48, 12));

  EXPECT_EQ(4, math::roundUpMultiple(3, 2));
  EXPECT_EQ(23, math::roundUpMultiple(3, 23));
  EXPECT_EQ(6, math::roundUpMultiple(6, 3));
  EXPECT_EQ(9, math::roundUpMultiple(7, 3));

  EXPECT_EQ(-8, math::roundUpMultiple(-9, 2));
  EXPECT_EQ(-6, math::roundUpMultiple(-7, 3));

  EXPECT_EQ(0, math::roundUpMultiple(-1, 2));

  EXPECT_EQ(2, math::roundUpMultiple(2, -2));
  EXPECT_EQ(0, math::roundUpMultiple(0, -2));
  EXPECT_EQ(-2, math::roundUpMultiple(-2, -2));
}

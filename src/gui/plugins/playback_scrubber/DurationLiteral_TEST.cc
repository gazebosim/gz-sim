/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "DurationLiteral.hh"

using namespace gz;

TEST(DurationLiteral, Valid)
{
  int64_t seconds = 0;

  EXPECT_TRUE(sim::detail::ParseDurationLiteral("200s", seconds));
  EXPECT_EQ(200, seconds);

  EXPECT_TRUE(sim::detail::ParseDurationLiteral("15m", seconds));
  EXPECT_EQ(15 * 60, seconds);

  EXPECT_TRUE(sim::detail::ParseDurationLiteral("2h", seconds));
  EXPECT_EQ(2 * 60 * 60, seconds);

  EXPECT_TRUE(sim::detail::ParseDurationLiteral("2h 19m 27s", seconds));
  EXPECT_EQ(2 * 60 * 60 + 19 * 60 + 27, seconds);

  EXPECT_TRUE(sim::detail::ParseDurationLiteral("1d 2h 3m 4s", seconds));
  EXPECT_EQ(24 * 60 * 60 + 2 * 60 * 60 + 3 * 60 + 4, seconds);

  EXPECT_TRUE(sim::detail::ParseDurationLiteral("  5m  2s  ", seconds));
  EXPECT_EQ(5 * 60 + 2, seconds);
}

TEST(DurationLiteral, Invalid)
{
  int64_t seconds = 123;

  EXPECT_FALSE(sim::detail::ParseDurationLiteral("", seconds));
  EXPECT_FALSE(sim::detail::ParseDurationLiteral("20", seconds));
  EXPECT_FALSE(sim::detail::ParseDurationLiteral("-1s", seconds));
  EXPECT_FALSE(sim::detail::ParseDurationLiteral("2H", seconds));
  EXPECT_FALSE(sim::detail::ParseDurationLiteral("2h garbage", seconds));
  EXPECT_FALSE(sim::detail::ParseDurationLiteral("garbage 2h", seconds));
  EXPECT_FALSE(sim::detail::ParseDurationLiteral(
      "9223372036854775807d", seconds));
}

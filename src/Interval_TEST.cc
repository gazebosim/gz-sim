/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <ostream>

#include "gz/math/Interval.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(IntervalTest, DefaultConstructor)
{
  const math::Intervald interval;
  EXPECT_DOUBLE_EQ(
      interval.LeftValue(),
      interval.RightValue());
}

/////////////////////////////////////////////////
TEST(IntervalTest, Constructor)
{
  constexpr bool kClosed = true;

  const math::Intervald interval(
      0., kClosed, 1., !kClosed);
  EXPECT_DOUBLE_EQ(interval.LeftValue(), 0.);
  EXPECT_TRUE(interval.IsLeftClosed());
  EXPECT_DOUBLE_EQ(interval.RightValue(), 1.);
  EXPECT_FALSE(interval.IsRightClosed());
}

/////////////////////////////////////////////////
TEST(IntervalTest, ConstructionHelpers)
{
  const math::Intervald openInterval =
      math::Intervald::Open(0., 1.);
  EXPECT_DOUBLE_EQ(openInterval.LeftValue(), 0.);
  EXPECT_FALSE(openInterval.IsLeftClosed());
  EXPECT_DOUBLE_EQ(openInterval.RightValue(), 1.);
  EXPECT_FALSE(openInterval.IsRightClosed());

  const math::Intervald leftClosedInterval =
      math::Intervald::LeftClosed(0., 1.);
  EXPECT_DOUBLE_EQ(leftClosedInterval.LeftValue(), 0.);
  EXPECT_TRUE(leftClosedInterval.IsLeftClosed());
  EXPECT_DOUBLE_EQ(leftClosedInterval.RightValue(), 1.);
  EXPECT_FALSE(leftClosedInterval.IsRightClosed());

  const math::Intervald rightClosedInterval =
      math::Intervald::RightClosed(0., 1.);
  EXPECT_DOUBLE_EQ(rightClosedInterval.LeftValue(), 0.);
  EXPECT_FALSE(rightClosedInterval.IsLeftClosed());
  EXPECT_DOUBLE_EQ(rightClosedInterval.RightValue(), 1.);
  EXPECT_TRUE(rightClosedInterval.IsRightClosed());

  const math::Intervald closedInterval =
      math::Intervald::Closed(0., 1.);
  EXPECT_DOUBLE_EQ(closedInterval.LeftValue(), 0.);
  EXPECT_TRUE(closedInterval.IsLeftClosed());
  EXPECT_DOUBLE_EQ(closedInterval.RightValue(), 1.);
  EXPECT_TRUE(closedInterval.IsRightClosed());
}

/////////////////////////////////////////////////
TEST(IntervalTest, UnboundedInterval)
{
  EXPECT_FALSE(math::Intervald::Unbounded.Empty());
  EXPECT_FALSE(math::Intervald::Unbounded.IsLeftClosed());
  EXPECT_FALSE(math::Intervald::Unbounded.IsRightClosed());
}

/////////////////////////////////////////////////
TEST(IntervalTest, EmptyInterval)
{
  EXPECT_FALSE(math::Intervald::Open(0., 1.).Empty());
  EXPECT_TRUE(math::Intervald::Open(0., 0.).Empty());
  EXPECT_TRUE(math::Intervald::LeftClosed(0., 0.).Empty());
  EXPECT_TRUE(math::Intervald::RightClosed(0., 0.).Empty());
  EXPECT_FALSE(math::Intervald::Closed(0., 0.).Empty());
  EXPECT_TRUE(math::Intervald::Closed(1., 0.).Empty());
}

/////////////////////////////////////////////////
TEST(IntervalTest, IntervalMembership)
{
  const math::Intervald emptyInterval =
      math::Intervald::Open(0., 0.);
  EXPECT_FALSE(emptyInterval.Contains(0.));

  const math::Intervald openInterval =
      math::Intervald::Open(0., 1.);
  EXPECT_FALSE(openInterval.Contains(0.));
  EXPECT_TRUE(openInterval.Contains(0.5));
  EXPECT_FALSE(openInterval.Contains(1));

  const math::Intervald leftClosedInterval =
      math::Intervald::LeftClosed(0., 1.);
  EXPECT_TRUE(leftClosedInterval.Contains(0.));
  EXPECT_TRUE(leftClosedInterval.Contains(0.5));
  EXPECT_FALSE(leftClosedInterval.Contains(1));

  const math::Intervald rightClosedInterval =
      math::Intervald::RightClosed(0., 1.);
  EXPECT_FALSE(rightClosedInterval.Contains(0.));
  EXPECT_TRUE(rightClosedInterval.Contains(0.5));
  EXPECT_TRUE(rightClosedInterval.Contains(1));

  const math::Intervald closedInterval =
      math::Intervald::Closed(0., 1.);
  EXPECT_TRUE(closedInterval.Contains(0.));
  EXPECT_TRUE(closedInterval.Contains(0.5));
  EXPECT_TRUE(closedInterval.Contains(1));

  const math::Intervald degenerateInterval =
      math::Intervald::Closed(0., 0.);
  EXPECT_TRUE(degenerateInterval.Contains(0.));
}

/////////////////////////////////////////////////
TEST(IntervalTest, IntervalSubset)
{
  const math::Intervald openInterval = math::Intervald::Open(0., 1.);
  EXPECT_FALSE(openInterval.Contains(math::Intervald::Open(0., 0.)));
  EXPECT_FALSE(openInterval.Contains(math::Intervald::Closed(-1., 0.)));
  EXPECT_FALSE(openInterval.Contains(math::Intervald::Closed(1., 2.)));
  EXPECT_FALSE(openInterval.Contains(math::Intervald::Open(0.5, 1.5)));
  EXPECT_FALSE(openInterval.Contains(math::Intervald::Open(-0.5, 0.5)));
  EXPECT_TRUE(openInterval.Contains(math::Intervald::Open(0.25, 0.75)));
  EXPECT_FALSE(openInterval.Contains(math::Intervald::Closed(0., 1.)));

  const math::Intervald closedInterval = math::Intervald::Closed(0., 1.);
  EXPECT_FALSE(closedInterval.Contains(math::Intervald::Open(0., 0.)));
  EXPECT_TRUE(closedInterval.Contains(math::Intervald::Closed(0., 0.)));
  EXPECT_FALSE(closedInterval.Contains(math::Intervald::Closed(0.5, 1.5)));
  EXPECT_FALSE(closedInterval.Contains(math::Intervald::Closed(-0.5, 0.5)));
  EXPECT_TRUE(closedInterval.Contains(math::Intervald::Closed(0.25, 0.75)));
  EXPECT_TRUE(closedInterval.Contains(math::Intervald::Open(0., 1.)));

  EXPECT_TRUE(math::Intervald::Unbounded.Contains(openInterval));
  EXPECT_TRUE(math::Intervald::Unbounded.Contains(closedInterval));
}

/////////////////////////////////////////////////
TEST(IntervalTest, IntervalEquality)
{
  EXPECT_NE(math::Intervald::Open(0., 0.), math::Intervald::Open(0., 0.));
  EXPECT_EQ(math::Intervald::Closed(0., 0.), math::Intervald::Closed(0., 0.));
  EXPECT_NE(math::Intervald::Open(0., 1.), math::Intervald::Closed(0., 1.));
  EXPECT_NE(
      math::Intervald::Open(0., 1.), math::Intervald::LeftClosed(0., 1.));
  EXPECT_NE(
      math::Intervald::Open(0., 1.), math::Intervald::RightClosed(0., 1.));
  EXPECT_NE(
      math::Intervald::Closed(0., 1.), math::Intervald::LeftClosed(0., 1.));
  EXPECT_NE(
      math::Intervald::Closed(0., 1.), math::Intervald::RightClosed(0., 1.));
}

/////////////////////////////////////////////////
TEST(IntervalTest, IntervalIntersection)
{
  const math::Intervald openInterval = math::Intervald::Open(0., 1.);
  EXPECT_FALSE(openInterval.Intersects(math::Intervald::Open(0.5, 0.5)));
  EXPECT_TRUE(openInterval.Intersects(math::Intervald::Open(0.5, 1.5)));
  EXPECT_TRUE(openInterval.Intersects(math::Intervald::Open(-0.5, 0.5)));
  EXPECT_FALSE(openInterval.Intersects(math::Intervald::Closed(1., 1.)));
  EXPECT_TRUE(openInterval.Intersects(math::Intervald::Closed(0.5, 0.5)));
  EXPECT_FALSE(openInterval.Intersects(math::Intervald::Open(1., 2.)));
  EXPECT_FALSE(openInterval.Intersects(math::Intervald::Open(-1., 0.)));
  EXPECT_FALSE(openInterval.Intersects(math::Intervald::LeftClosed(1., 2.)));
  EXPECT_FALSE(openInterval.Intersects(math::Intervald::RightClosed(-1., 0.)));

  const math::Intervald closedInterval = math::Intervald::Closed(0., 1.);
  EXPECT_FALSE(closedInterval.Intersects(math::Intervald::Open(1., 1.)));
  EXPECT_TRUE(closedInterval.Intersects(math::Intervald::Closed(0.5, 1.5)));
  EXPECT_TRUE(closedInterval.Intersects(math::Intervald::Closed(-0.5, 0.5)));
  EXPECT_FALSE(closedInterval.Intersects(math::Intervald::Closed(1.5, 2.5)));
  EXPECT_FALSE(closedInterval.Intersects(math::Intervald::Closed(-1.5, -0.5)));
  EXPECT_FALSE(closedInterval.Intersects(math::Intervald::Open(1., 2.)));
  EXPECT_FALSE(closedInterval.Intersects(math::Intervald::Open(-1., 0.)));
  EXPECT_TRUE(closedInterval.Intersects(math::Intervald::LeftClosed(1., 2.)));
  EXPECT_TRUE(closedInterval.Intersects(math::Intervald::RightClosed(-1., 0.)));
}

/////////////////////////////////////////////////
TEST(IntervalTest, IntervalStreaming)
{
  {
    std::ostringstream os;
    os << math::Intervald::Open(0., 1.);
    EXPECT_EQ(os.str(), "(0, 1)");
  }
  {
    std::ostringstream os;
    os << math::Intervald::LeftClosed(0., 1.);
    EXPECT_EQ(os.str(), "[0, 1)");
  }
  {
    std::ostringstream os;
    os << math::Intervald::RightClosed(0., 1.);
    EXPECT_EQ(os.str(), "(0, 1]");
  }
  {
    std::ostringstream os;
    os << math::Intervald::Closed(0., 1.);
    EXPECT_EQ(os.str(), "[0, 1]");
  }
}

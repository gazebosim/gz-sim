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

#include "gz/math/Region3.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(Region3Test, DefaultConstructor)
{
  const math::Region3d region;
  EXPECT_TRUE(region.Ix().Empty());
  EXPECT_TRUE(region.Iy().Empty());
  EXPECT_TRUE(region.Iz().Empty());
}

/////////////////////////////////////////////////
TEST(Region3Test, Constructor)
{
  const math::Region3d region(
      math::Intervald::Open(0., 1.),
      math::Intervald::Closed(-1., 1.),
      math::Intervald::Open(-1., 0.));
  EXPECT_EQ(region.Ix(), math::Intervald::Open(0., 1.));
  EXPECT_EQ(region.Iy(), math::Intervald::Closed(-1., 1.));
  EXPECT_EQ(region.Iz(), math::Intervald::Open(-1., 0.));
}

/////////////////////////////////////////////////
TEST(Region3Test, ConstructionHelpers)
{
  const math::Region3d openRegion =
      math::Region3d::Open(0., 0., 0., 1., 1., 1.);
  EXPECT_EQ(openRegion.Ix(), math::Intervald::Open(0., 1.));
  EXPECT_EQ(openRegion.Iy(), math::Intervald::Open(0., 1.));
  EXPECT_EQ(openRegion.Iz(), math::Intervald::Open(0., 1.));
  const math::Region3d closedRegion =
      math::Region3d::Closed(0., 0., 0., 1., 1., 1.);
  EXPECT_EQ(closedRegion.Ix(), math::Intervald::Closed(0., 1.));
  EXPECT_EQ(closedRegion.Iy(), math::Intervald::Closed(0., 1.));
  EXPECT_EQ(closedRegion.Iz(), math::Intervald::Closed(0., 1.));
}

/////////////////////////////////////////////////
TEST(Region3Test, UnboundedRegion)
{
  EXPECT_FALSE(math::Region3d::Unbounded.Empty());
  EXPECT_EQ(math::Region3d::Unbounded.Ix(), math::Intervald::Unbounded);
  EXPECT_EQ(math::Region3d::Unbounded.Iy(), math::Intervald::Unbounded);
  EXPECT_EQ(math::Region3d::Unbounded.Iz(), math::Intervald::Unbounded);
}

/////////////////////////////////////////////////
TEST(Region3Test, EmptyRegion)
{
  EXPECT_FALSE(math::Region3d::Open(0., 0., 0., 1., 1., 1.).Empty());
  EXPECT_TRUE(math::Region3d::Open(0., 0., 0., 0., 0., 0.).Empty());
  EXPECT_TRUE(math::Region3d::Open(0., 0., 0., 0., 1., 1.).Empty());
  EXPECT_TRUE(math::Region3d::Open(0., 0., 0., 1., 0., 1.).Empty());
  EXPECT_TRUE(math::Region3d::Open(0., 0., 0., 1., 1., 0.).Empty());
  EXPECT_FALSE(math::Region3d::Closed(0., 0., 0., 0., 0., 0.).Empty());
  EXPECT_TRUE(math::Region3d::Closed(1., 1., 1., 0., 0., 0.).Empty());
}

/////////////////////////////////////////////////
TEST(Region3Test, RegionMembership)
{
  const math::Region3d openRegion =
      math::Region3d::Open(0., 0., 0., 1., 1., 1.);
  EXPECT_FALSE(openRegion.Contains(math::Vector3d(0., 0., 0.)));
  EXPECT_TRUE(openRegion.Contains(math::Vector3d(0.5, 0.5, 0.5)));
  EXPECT_FALSE(openRegion.Contains(math::Vector3d(1., 1., 1.)));

  const math::Region3d closedRegion =
      math::Region3d::Closed(0., 0., 0., 1., 1., 1.);
  EXPECT_TRUE(closedRegion.Contains(math::Vector3d(0., 0., 0.)));
  EXPECT_TRUE(closedRegion.Contains(math::Vector3d(0.5, 0.5, 0.5)));
  EXPECT_TRUE(closedRegion.Contains(math::Vector3d(1., 1., 1.)));
}

/////////////////////////////////////////////////
TEST(Region3Test, RegionSubset)
{
  const math::Region3d openRegion =
      math::Region3d::Open(0., 0., 0., 1., 1., 1.);
  EXPECT_TRUE(openRegion.Contains(
      math::Region3d::Open(0.25, 0.25, 0.25,
                           0.75, 0.75, 0.75)));
  EXPECT_FALSE(openRegion.Contains(
      math::Region3d::Open(-1., 0.25, 0.25,
                            0., 0.75, 0.75)));
  EXPECT_FALSE(openRegion.Contains(
      math::Region3d::Open(0.25, -1., 0.25,
                           0.75,  0., 0.75)));
  EXPECT_FALSE(openRegion.Contains(
      math::Region3d::Open(0.25, 0.25, -1.,
                           0.75, 0.75,  0.)));
  EXPECT_FALSE(openRegion.Contains(
      math::Region3d::Closed(0., 0., 0.,
                             1., 1., 1.)));

  const math::Region3d closedRegion =
      math::Region3d::Closed(0., 0., 0., 1., 1., 1.);
  EXPECT_TRUE(closedRegion.Contains(
      math::Region3d::Closed(0., 0., 0., 1., 1., 1.)));
  EXPECT_TRUE(closedRegion.Contains(
      math::Region3d::Closed(0., 0., 0., 0., 0., 0.)));
}

/////////////////////////////////////////////////
TEST(Region3Test, RegionEquality)
{
  EXPECT_NE(math::Region3d::Open(0., 0., 0., 0., 0., 0.),
            math::Region3d::Open(0., 0., 0., 0., 0., 0.));
  EXPECT_EQ(math::Region3d::Closed(0., 0., 0., 0., 0., 0.),
            math::Region3d::Closed(0., 0., 0., 0., 0., 0.));
  EXPECT_NE(math::Region3d::Open(0., 0., 0., 1., 1., 1.),
            math::Region3d::Closed(0., 0., 0., 1., 1., 1.));
}

/////////////////////////////////////////////////
TEST(Region3Test, RegionIntersection)
{
  const math::Region3d region =
      math::Region3d::Open(0., 0., 0., 1., 1., 1.);
  EXPECT_TRUE(region.Intersects(
      math::Region3d::Open(0.5, 0.5, 0.5, 1.5, 1.5, 1.5)));
  EXPECT_TRUE(region.Intersects(
      math::Region3d::Open(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5)));
  EXPECT_FALSE(region.Intersects(
      math::Region3d::Open(1., 1., 1., 2., 2., 2.)));
  EXPECT_FALSE(region.Intersects(
      math::Region3d::Open(-1., -1., -1., 0., 0., 0.)));
}

/////////////////////////////////////////////////
TEST(IntervalTest, RegionStreaming)
{
  {
    std::ostringstream os;
    os << math::Region3d::Open(0., 1., 2.,
                               3., 4., 5.);
    EXPECT_EQ(os.str(), "(0, 3) x (1, 4) x (2, 5)");
  }
  {
    std::ostringstream os;
    os << math::Region3d::Closed(-1., 0., -1.,
                                 0., 1., 0.);
    EXPECT_EQ(os.str(), "[-1, 0] x [0, 1] x [-1, 0]");
  }
  {
    std::ostringstream os;
    os << math::Region3d(
        math::Intervald::LeftClosed(0., 100.),
        math::Intervald::RightClosed(-100., 0.),
        math::Intervald::Closed(0., 1.));
    EXPECT_EQ(os.str(), "[0, 100) x (-100, 0] x [0, 1]");
  }
}

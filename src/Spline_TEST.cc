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

#include "ignition/math/Vector3.hh"
#include "ignition/math/Spline.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(SplineTest, Spline)
{
  math::Spline s;

  s.AddPoint(math::Vector3d(0, 0, 0));
  EXPECT_EQ(static_cast<unsigned int>(1), s.PointCount());

  s.Clear();
  EXPECT_EQ(static_cast<unsigned int>(0), s.PointCount());

  s.AddPoint(math::Vector3d(0, 0, 0));
  EXPECT_TRUE(s.Point(0) == math::Vector3d(0, 0, 0));
  s.AddPoint(math::Vector3d(1, 1, 1));
  EXPECT_TRUE(s.Point(1) == math::Vector3d(1, 1, 1));

  // ::UpdatePoint
  EXPECT_THROW(s.UpdatePoint(2, math::Vector3d(2, 2, 2)), math::IndexException);
  s.UpdatePoint(1, math::Vector3d(2, 2, 2));
  s.AutoCalculate(false);
  s.UpdatePoint(0, math::Vector3d(-1, -1, -1));
  s.AutoCalculate(true);

  // ::Interpolate
  EXPECT_TRUE(s.Interpolate(0.5) == math::Vector3d(0.5, 0.5, 0.5));

  // ::Interpolate
  s.AddPoint(math::Vector3d(4, 4, 4));
  EXPECT_TRUE(s.Interpolate(1, 0.2) == math::Vector3d(2.496, 2.496, 2.496));
}

/////////////////////////////////////////////////
TEST(SplineTest, Tension)
{
  math::Spline s;
  s.Tension(0.1);

  EXPECT_DOUBLE_EQ(s.Tension(), 0.1);
}

/////////////////////////////////////////////////
TEST(SplineTest, Interpolate)
{
  math::Spline s;
  EXPECT_THROW(s.Interpolate(0, 0.1), math::IndexException);

  s.AddPoint(math::Vector3d(0, 0, 0));
  EXPECT_EQ(s.Interpolate(0, 0.1), math::Vector3d(0, 0, 0));

  s.AddPoint(math::Vector3d(1, 2, 3));
  EXPECT_EQ(s.Interpolate(0, 0.0), s.Point(0));

  EXPECT_EQ(s.Interpolate(0, 1.0), s.Point(1));
}

/////////////////////////////////////////////////
TEST(SplineTest, Point)
{
  math::Spline s;
  EXPECT_THROW(s.Point(0), math::IndexException);
}

/////////////////////////////////////////////////
TEST(SplineTest, Tangent)
{
  math::Spline s;
  EXPECT_THROW(s.Tangent(0), math::IndexException);

  s.AddPoint(math::Vector3d(0, 0, 0));
  EXPECT_THROW(s.Tangent(0), math::IndexException);
  s.AddPoint(math::Vector3d(1, 0, 0));
  EXPECT_EQ(s.Tangent(0), math::Vector3d(0.5, 0, 0));
}

/////////////////////////////////////////////////
TEST(SplineTest, RecalcTangents)
{
  math::Spline s;
  s.AddPoint(math::Vector3d(0, 0, 0));
  s.AddPoint(math::Vector3d(.4, .4, .4));
  s.AddPoint(math::Vector3d(0, 0, 0));

  s.RecalcTangents();

  EXPECT_EQ(s.Interpolate(0, 0.5), math::Vector3d(0.2, 0.2, 0.2));
  EXPECT_EQ(s.Interpolate(1, 0.5), math::Vector3d(0.2, 0.2, 0.2));
}

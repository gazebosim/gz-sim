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
  EXPECT_FALSE(s.UpdatePoint(2, math::Vector3d(2, 2, 2)));

  EXPECT_TRUE(s.UpdatePoint(1, math::Vector3d(2, 2, 2)));
  s.AutoCalculate(false);
  EXPECT_TRUE(s.UpdatePoint(0, math::Vector3d(-1, -1, -1)));
  s.AutoCalculate(true);

  // ::Interpolate
  EXPECT_EQ(s.Interpolate(0.0), math::Vector3d(-1, -1, -1));
  EXPECT_EQ(s.Interpolate(0.5), math::Vector3d(0.5, 0.5, 0.5));
  EXPECT_EQ(s.Interpolate(1.0), math::Vector3d(2, 2, 2));

  // ::Interpolate
  s.AddPoint(math::Vector3d(4, 4, 4));
  EXPECT_EQ(s.Interpolate(1, 0.2), math::Vector3d(2.496, 2.496, 2.496));
}

/////////////////////////////////////////////////
TEST(SplineTest, FixedTangentSpline)
{
  math::Spline s;

  // ::AddPoint
  s.AutoCalculate(false);
  s.AddPoint(math::Vector3d(0, 0, 0));
  s.AddPoint(math::Vector3d(0, 0.5, 0), math::Vector3d(0, 1, 0));
  s.AddPoint(math::Vector3d(0.5, 1, 0), math::Vector3d(1, 0, 0));
  s.AddPoint(math::Vector3d(1, 1, 0), math::Vector3d(1, 0, 0));

  // ::UpdatePoint
  s.UpdatePoint(0, math::Vector3d(0, 0, 0), math::Vector3d(0, 1, 0));

  s.AutoCalculate(true);

  s.RecalcTangents();

  // ::Interpolate
  EXPECT_EQ(s.Interpolate(0, 0.5), math::Vector3d(0, 0.25, 0));
  EXPECT_EQ(s.InterpolateTangent(0, 0.5), math::Vector3d(0, 0.25, 0));
  EXPECT_EQ(s.Interpolate(1, 0.5), math::Vector3d(0.125, 0.875, 0));
  EXPECT_EQ(s.Interpolate(2, 0.5), math::Vector3d(0.75, 1, 0));
  EXPECT_EQ(s.InterpolateTangent(2, 0.5), math::Vector3d(0.25, 0, 0));
}

/////////////////////////////////////////////////
TEST(SplineTest, ArcLength)
{
  math::Spline s;
  EXPECT_FALSE(std::isfinite(s.ArcLength()));
  s.AddPoint(math::Vector3d(1, 1, 1), math::Vector3d(1, 1, 1));
  EXPECT_FALSE(std::isfinite(s.ArcLength(0)));
  s.AddPoint(math::Vector3d(3, 3, 3), math::Vector3d(1, 1, 1));
  s.AddPoint(math::Vector3d(4, 4, 4), math::Vector3d(1, 1, 1));
  EXPECT_NEAR(s.ArcLength(0, 1.0), 3.46410161513775, 1e-14);
  EXPECT_NEAR(s.ArcLength(), 5.19615242270663, 1e-14);
  EXPECT_DOUBLE_EQ(s.ArcLength(), s.ArcLength(1.0));
  EXPECT_FALSE(std::isfinite(s.ArcLength(-1.0)));
  EXPECT_FALSE(std::isfinite(s.ArcLength(4, 0.0)));
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
  EXPECT_NO_THROW(s.Interpolate(0, 0.1));
  EXPECT_FALSE(s.Interpolate(0, 0.1).IsFinite());

  s.AddPoint(math::Vector3d(0, 0, 0));
  EXPECT_EQ(s.Interpolate(0, 0.1), math::Vector3d(0, 0, 0));
  EXPECT_FALSE(s.InterpolateTangent(0.1).IsFinite());

  s.AddPoint(math::Vector3d(1, 2, 3));
  EXPECT_EQ(s.Interpolate(0, 0.0), s.Point(0));
  EXPECT_FALSE(s.Interpolate(0, -0.1).IsFinite());
  EXPECT_EQ(s.InterpolateTangent(0, 0.0), s.Tangent(0));

  // Fast and slow call variations
  EXPECT_EQ(s.Interpolate(0, 0.5), math::Vector3d(0.5, 1.0, 1.5));
  EXPECT_EQ(s.Interpolate(0, 1.0), s.Point(1));
  EXPECT_EQ(s.InterpolateTangent(0, 0.5), math::Vector3d(1.25, 2.5, 3.75));
  EXPECT_EQ(s.InterpolateTangent(0, 1.0), s.Tangent(1));
  EXPECT_EQ(s.InterpolateMthDerivative(2, 0.5), math::Vector3d(0, 0, 0));
  EXPECT_EQ(s.InterpolateMthDerivative(2, 1.0), math::Vector3d(-3, -6, -9));
  EXPECT_EQ(s.InterpolateMthDerivative(3, 0.5), math::Vector3d(-6, -12, -18));
  EXPECT_EQ(s.InterpolateMthDerivative(3, 1.0), math::Vector3d(-6, -12, -18));
  EXPECT_EQ(s.InterpolateMthDerivative(4, 0.5), math::Vector3d(0, 0, 0));
  EXPECT_EQ(s.InterpolateMthDerivative(4, 1.0), math::Vector3d(0, 0, 0));
}

/////////////////////////////////////////////////
TEST(SplineTest, Point)
{
  math::Spline s;
  EXPECT_NO_THROW(s.Point(0));
  EXPECT_FALSE(s.Point(0).IsFinite());
}

/////////////////////////////////////////////////
TEST(SplineTest, Tangent)
{
  math::Spline s;
  EXPECT_NO_THROW(s.Tangent(0));
  EXPECT_FALSE(s.Tangent(0).IsFinite());

  s.AddPoint(math::Vector3d(0, 0, 0));
  EXPECT_NO_THROW(s.Tangent(0));
  EXPECT_FALSE(s.Tangent(0).IsFinite());

  s.AddPoint(math::Vector3d(1, 0, 0));
  EXPECT_EQ(s.Tangent(0), math::Vector3d(0.5, 0, 0));

  s.AddPoint(math::Vector3d(1, 1, 0), math::Vector3d(-1, 1, 0));
  EXPECT_EQ(s.Tangent(1), math::Vector3d(0.5, 0.5, 0));
  EXPECT_EQ(s.Tangent(2), math::Vector3d(-1, 1, 0));
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

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

#include "ignition/math/Helpers.hh"
#include "ignition/math/Vector3.hh"
#include "ignition/math/Quaternion.hh"
#include "ignition/math/RotationSpline.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(RotationSplineTest, RotationSpline)
{
  math::RotationSpline s;

  s.AddPoint(math::Quaterniond(0, 0, 0));
  EXPECT_EQ(static_cast<unsigned int>(1), s.PointCount());

  s.Clear();
  EXPECT_EQ(static_cast<unsigned int>(0), s.PointCount());

  s.AddPoint(math::Quaterniond(0, 0, 0));
  EXPECT_TRUE(s.Point(0) == math::Quaterniond(0, 0, 0));
  s.AddPoint(math::Quaterniond(.1, .1, .1));
  EXPECT_TRUE(s.Point(1) == math::Quaterniond(.1, .1, .1));

  // ::UpdatePoint
  EXPECT_NO_THROW(s.UpdatePoint(2, math::Quaterniond(.2, .2, .2)));
  EXPECT_FALSE(s.UpdatePoint(2, math::Quaterniond(.2, .2, .2)));

  EXPECT_TRUE(s.UpdatePoint(1, math::Quaterniond(.2, .2, .2)));
  s.AutoCalculate(false);
  EXPECT_TRUE(s.UpdatePoint(0, math::Quaterniond(
    math::Vector3d(-.1, -.1, -.1))));
  s.AutoCalculate(true);

  // ::Interpolate
  EXPECT_TRUE(s.Interpolate(0.5) ==
      math::Quaterniond(0.998089, 0.0315333, 0.0427683, 0.0315333));

  // ::Interpolate
  s.AddPoint(math::Quaterniond(.4, .4, .4));
  EXPECT_NO_THROW(s.Interpolate(4, 0.2));
  EXPECT_FALSE(s.Interpolate(4, 0.2).IsFinite());

  EXPECT_EQ(s.Interpolate(s.PointCount()-1, 0.2),
            s.Point(s.PointCount()-1));
  EXPECT_TRUE(s.Interpolate(1, 0.2) ==
      math::Quaterniond(0.978787, 0.107618, 0.137159, 0.107618));
  EXPECT_EQ(s.Interpolate(1, 0.0), s.Point(1));
  EXPECT_EQ(s.Interpolate(1, 1.0), s.Point(2));
}

/////////////////////////////////////////////////
TEST(RotationSplineTest, GetPoint)
{
  math::RotationSpline s;
  EXPECT_NO_THROW(s.Point(0));
  EXPECT_FALSE(s.Point(0).IsFinite());
  EXPECT_NO_THROW(s.Point(1));
  EXPECT_FALSE(s.Point(1).IsFinite());

  s.AddPoint(math::Quaterniond(0, 0, 0));
  EXPECT_NO_THROW(s.Point(0));
  EXPECT_NO_THROW(s.Point(1));
  EXPECT_TRUE(s.Point(1).IsFinite());
}

/////////////////////////////////////////////////
TEST(RotationSplineTest, RecalcTangents)
{
  math::RotationSpline s;
  s.AddPoint(math::Quaterniond(0, 0, 0));
  s.AddPoint(math::Quaterniond(.4, .4, .4));
  s.AddPoint(math::Quaterniond(0, 0, 0));

  s.RecalcTangents();
  EXPECT_EQ(s.Interpolate(0, 0.5),
      math::Quaterniond(0.987225, 0.077057, 0.11624, 0.077057));

  math::Quaterniond q = s.Interpolate(1, 0.5);
  EXPECT_EQ(s.Interpolate(1, 0.5),
      math::Quaterniond(0.987225, 0.077057, 0.11624, 0.077057));
}

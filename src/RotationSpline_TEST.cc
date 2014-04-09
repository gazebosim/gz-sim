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
#include "ignition/math/Vector3d.hh"
#include "ignition/math/Quaterniond.hh"
#include "ignition/math/RotationSpline.hh"

using namespace ignition;

TEST(RotationSplineTest, RotationSpline)
{
  math::RotationSpline s;

  s.AddPoint(math::Quaterniond(0, 0, 0));
  EXPECT_EQ(static_cast<unsigned int>(1), s.GetNumPoints());

  s.Clear();
  EXPECT_EQ(static_cast<unsigned int>(0), s.GetNumPoints());

  s.AddPoint(math::Quaterniond(0, 0, 0));
  EXPECT_TRUE(s.GetPoint(0) == math::Quaterniond(0, 0, 0));
  s.AddPoint(math::Quaterniond(.1, .1, .1));
  EXPECT_TRUE(s.GetPoint(1) == math::Quaterniond(.1, .1, .1));

  // ::UpdatePoint
  s.UpdatePoint(1, math::Quaterniond(.2, .2, .2));
  s.SetAutoCalculate(false);
  s.UpdatePoint(0, math::Vector3d(-.1, -.1, -.1));
  s.SetAutoCalculate(true);

  // ::Interpolate
  EXPECT_TRUE(s.Interpolate(0.5) ==
      math::Quaterniond(0.998089, 0.0315333, 0.0427683, 0.0315333));

  // ::Interpolate
  s.AddPoint(math::Quaterniond(.4, .4, .4));
  EXPECT_TRUE(s.Interpolate(1, 0.2) ==
      math::Quaterniond(0.978787, 0.107618, 0.137159, 0.107618));
}

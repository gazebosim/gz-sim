/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/sim/Primitives.hh>
#include <sdf/Root.hh>

using PrimitiveShape = gz::sim::PrimitiveShape;
using PrimitiveLight = gz::sim::PrimitiveLight;

/////////////////////////////////////////////////
TEST(Primitives, shapes)
{
  auto primitives  = {
    PrimitiveShape::kBox,
    PrimitiveShape::kSphere,
    PrimitiveShape::kCylinder,
    PrimitiveShape::kCapsule,
    PrimitiveShape::kEllipsoid
  };

  for (auto prim : primitives)
  {
    auto sdfString = gz::sim::getPrimitiveShape(prim);
    ASSERT_FALSE(sdfString.empty());

    /// Verify that string contains valid SDF
    sdf::Root root;
    auto errors = root.LoadSdfString(sdfString);
    EXPECT_TRUE(errors.empty()) << sdfString;
  }
}

/////////////////////////////////////////////////
TEST(Primitives, lights)
{
  auto primitives  = {
    PrimitiveLight::kDirectional,
    PrimitiveLight::kPoint,
    PrimitiveLight::kSpot,
  };

  for (auto prim : primitives)
  {
    auto sdfString = gz::sim::getPrimitiveLight(prim);
    ASSERT_FALSE(sdfString.empty());

    /// Verify that string contains valid SDF
    sdf::Root root;
    auto errors = root.LoadSdfString(sdfString);
    EXPECT_TRUE(errors.empty()) << sdfString;
  }
}

/////////////////////////////////////////////////
TEST(Primitives, invalid)
{
  auto sdfString = gz::sim::getPrimitive("foobar");
  ASSERT_TRUE(sdfString.empty());
}

/////////////////////////////////////////////////
TEST(Primitives, strings)
{
  auto primitives = {
    "box", "sphere", "cylinder", "capsule", "ellipsoid",
    "point", "directional", "spot"
  };

  for (auto prim : primitives)
  {
    auto sdfString = gz::sim::getPrimitive(prim);
    ASSERT_FALSE(sdfString.empty());

    /// Verify that string contains valid SDF
    sdf::Root root;
    auto errors = root.LoadSdfString(sdfString);
    EXPECT_TRUE(errors.empty()) << sdfString;
  }
}

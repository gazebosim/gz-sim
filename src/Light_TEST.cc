/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <cstddef>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Light.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"

/////////////////////////////////////////////////
TEST(LightTest, Constructor)
{
  gz::sim::Light lightNull;
  EXPECT_EQ(gz::sim::kNullEntity, lightNull.Entity());

  gz::sim::Entity id(3);
  gz::sim::Light light(id);

  EXPECT_EQ(id, light.Entity());
}

/////////////////////////////////////////////////
TEST(LightTest, CopyConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Light light(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  gz::sim::Light lightCopy(light); // NOLINT
  EXPECT_EQ(light.Entity(), lightCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LightTest, CopyAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Light light(id);

  gz::sim::Light lightCopy;
  lightCopy = light;
  EXPECT_EQ(light.Entity(), lightCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LightTest, MoveConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Light light(id);

  gz::sim::Light lightMoved(std::move(light));
  EXPECT_EQ(id, lightMoved.Entity());
}

/////////////////////////////////////////////////
TEST(LightTest, MoveAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Light light(id);

  gz::sim::Light lightMoved;
  lightMoved = std::move(light);
  EXPECT_EQ(id, lightMoved.Entity());
}

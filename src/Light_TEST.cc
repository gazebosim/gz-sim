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

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Light.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Sensor.hh"

/////////////////////////////////////////////////
TEST(LightTest, Constructor)
{
  ignition::gazebo::Light lightNull;
  EXPECT_EQ(ignition::gazebo::kNullEntity, lightNull.Entity());

  ignition::gazebo::Entity id(3);
  ignition::gazebo::Light light(id);

  EXPECT_EQ(id, light.Entity());
}

/////////////////////////////////////////////////
TEST(LightTest, CopyConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Light light(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  ignition::gazebo::Light lightCopy(light); // NOLINT
  EXPECT_EQ(light.Entity(), lightCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LightTest, CopyAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Light light(id);

  ignition::gazebo::Light lightCopy;
  lightCopy = light;
  EXPECT_EQ(light.Entity(), lightCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LightTest, MoveConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Light light(id);

  ignition::gazebo::Light lightMoved(std::move(light));
  EXPECT_EQ(id, lightMoved.Entity());
}

/////////////////////////////////////////////////
TEST(LightTest, MoveAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Light light(id);

  ignition::gazebo::Light lightMoved;
  lightMoved = std::move(light);
  EXPECT_EQ(id, lightMoved.Entity());
}

/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace ignition::gazebo;

void ComponentDefaultConstructor()
{
  EntityComponentManager ecm;
  Entity entity = ecm.CreateEntity();

  // Use a default a components constructor
  ecm.CreateComponent(entity, components::Name());

  //Get the existing component and assign it a new value
  auto *comp = ecm.Component<components::Name>(entity);
  ASSERT_NE(nullptr, comp);

  *comp = components::Name("test");

  // Exit with exit code 0 so the test passes
  exit(0);
}
/////////////////////////////////////////////////
TEST(ComponentDeathTest, ComponentDefaultConstructor)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_EXIT(ComponentDefaultConstructor(), ::testing::ExitedWithCode(0), "");
}

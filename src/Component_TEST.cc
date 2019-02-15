/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <memory>

#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace ignition::gazebo;

/////////////////////////////////////////////////
/// Test that using the default constructor of Component doesn't cause
/// problems when copying
TEST(ComponentTest, ComponentCanBeCopiedAfterDefaultCtor)
{
  // Use Component's default constructor
  auto comp = components::Name();

  // Test copy constructor and assignment
  components::Name compCopy(comp);
  comp = components::Name("test");

  // If it got here we have succeeded
  SUCCEED();
}

TEST(ComponentTest, DataByMove)
{
  // Create a custom component with shared_ptr data
  using CustomComponent =
      components::Component<std::shared_ptr<int>, class CustomComponentTag>;

  EntityComponentManager ecm;
  Entity entity = ecm.CreateEntity();

  auto data = std::make_shared<int>(1);
  // Copy data so we can check the use count after it has been moved
  auto dataCopy = data;

  EXPECT_EQ(2u, dataCopy.use_count());

  ecm.CreateComponent(entity, CustomComponent(std::move(data)));

  // If "data" was moved, the use cound should still be 2.
  EXPECT_EQ(2u, dataCopy.use_count());
}

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
#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/TagWrapper.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
TEST(ComponentFactoryTest, Register)
{
  // auto factory = components::Factory::Instance();

  // Create a custom component.
  using MyCustom = components::TagWrapper<class MyCustomTag>;
  components::Factory::Register<MyCustom>("ign_gazebo_components.MyCustom");
  auto components = components::Factory::Components();
  EXPECT_NE(components.end(),
      std::find(components.begin(), components.end(),
          "ign_gazebo_components.MyCustom"));
}

/////////////////////////////////////////////////
TEST(ComponentFactoryTest, New)
{
  // auto factory = components::Factory::Instance();

  {
    auto comp = components::Factory::New<components::Pose>("__unknown_component__");
    ASSERT_TRUE(comp == nullptr);
  }

  {
    auto comp = components::Factory::New<components::Pose>("ign_gazebo_components.Pose");
    ASSERT_TRUE(comp != nullptr);
    EXPECT_EQ("ign_gazebo_components.Pose", comp->name);
    EXPECT_EQ("ign_gazebo_components.Pose", components::Pose::name);
    EXPECT_NE(0u, comp->id);
    EXPECT_EQ(comp->id, components::Pose::id);
  }

  {
    auto comp = components::Factory::New("ign_gazebo_components.Pose");
    ASSERT_TRUE(comp != nullptr);
  }

  {
    auto id = EntityComponentManager::ComponentType<components::Pose>();
    auto comp = components::Factory::New(id);
    ASSERT_TRUE(comp != nullptr);
  }
}

///////////////////////////////////////////////
TEST(ComponentFactoryTest, Components)
{
  // auto factory = components::Factory::Instance();

  std::vector<std::string> comps = components::Factory::Components();
  EXPECT_FALSE(comps.empty());
  EXPECT_TRUE(std::find(comps.begin(), comps.end(),
      std::string("ign_gazebo_components.Altimeter")) != comps.end());
}

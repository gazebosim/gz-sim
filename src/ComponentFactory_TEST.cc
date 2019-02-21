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
#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Pose.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
TEST(ComponentFactoryTest, Register)
{
  auto factory = components::Factory::Instance();

  // Create a custom component.
  using MyCustom = components::Component<components::NoData, class MyCustomTag>;

  // Check it has no type name or id yet
  EXPECT_EQ(0u, MyCustom::typeId);
  EXPECT_TRUE(MyCustom::typeName.empty());

  // Store number of registered component types
  EXPECT_EQ(factory->TypeNames().size(), factory->TypeIds().size());
  auto registeredCount = factory->TypeNames().size();

  factory->Register<MyCustom>("ign_gazebo_components.MyCustom",
      new components::ComponentDescriptor<MyCustom>());

  // Check now it has type name and id
  EXPECT_NE(0u, MyCustom::typeId);
  EXPECT_EQ("ign_gazebo_components.MyCustom", MyCustom::typeName);

  // Check factory knows name and id
  auto names = factory->TypeNames();
  EXPECT_EQ(registeredCount + 1, names.size());
  EXPECT_NE(names.end(), std::find(names.begin(), names.end(),
          "ign_gazebo_components.MyCustom"));

  auto ids = factory->TypeIds();
  EXPECT_EQ(registeredCount + 1, ids.size());
  EXPECT_NE(ids.end(), std::find(ids.begin(), ids.end(), MyCustom::typeId));
}

/////////////////////////////////////////////////
TEST(ComponentFactoryTest, New)
{
  auto factory = components::Factory::Instance();

  {
    auto comp = factory->New("__unknown_component__");
    ASSERT_TRUE(comp == nullptr);
  }

  {
    auto comp = factory->New<components::Pose>();
    ASSERT_TRUE(comp != nullptr);
    EXPECT_EQ("ign_gazebo_components.Pose", comp->typeName);
    EXPECT_EQ("ign_gazebo_components.Pose", components::Pose::typeName);
    EXPECT_NE(0u, comp->typeId);
    EXPECT_EQ(comp->typeId, components::Pose::typeId);
  }

  {
    auto comp = factory->New("ign_gazebo_components.Pose");
    EXPECT_EQ("ign_gazebo_components.Pose", comp->TypeName());
    EXPECT_NE(0u, comp->TypeId());
    ASSERT_TRUE(comp != nullptr);
  }

  {
    auto comp = factory->New(components::Pose::typeId);
    EXPECT_EQ("ign_gazebo_components.Pose", comp->TypeName());
    EXPECT_NE(0u, comp->TypeId());
    ASSERT_TRUE(comp != nullptr);
  }
}

///////////////////////////////////////////////
TEST(ComponentFactoryTest, TypeNames)
{
  auto factory = components::Factory::Instance();

  std::vector<std::string> comps = factory->TypeNames();
  EXPECT_FALSE(comps.empty());
  EXPECT_TRUE(std::find(comps.begin(), comps.end(),
      std::string("ign_gazebo_components.Altimeter")) != comps.end());
}

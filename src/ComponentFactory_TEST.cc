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
#include "gz/sim/test_config.hh"
#include "gz/sim/components/Component.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"

#include "../test/helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
class ComponentFactoryTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture::SetUp();
    common::setenv("IGN_DEBUG_COMPONENT_FACTORY", "true");
  }
};

/////////////////////////////////////////////////
TEST_F(ComponentFactoryTest, Register)
{
  auto factory = components::Factory::Instance();

  // Create a custom component.
  using MyCustom = components::Component<components::NoData, class MyCustomTag>;

  // Check it has no type id yet
  EXPECT_EQ(0u, MyCustom::typeId);
  EXPECT_EQ("", MyCustom::typeName);
  EXPECT_EQ("", factory->Name(MyCustom::typeId));

  // Store number of registered component types
  auto registeredCount = factory->TypeIds().size();

  factory->Register<MyCustom>("ign_gazebo_components.MyCustom",
      new components::ComponentDescriptor<MyCustom>());

  // Check now it has type id
  EXPECT_NE(0u, MyCustom::typeId);
  EXPECT_EQ("ign_gazebo_components.MyCustom", MyCustom::typeName);
  EXPECT_EQ("ign_gazebo_components.MyCustom",
      factory->Name(MyCustom::typeId));

  // Check factory knows id
  auto ids = factory->TypeIds();
  EXPECT_EQ(registeredCount + 1, ids.size());
  EXPECT_NE(ids.end(), std::find(ids.begin(), ids.end(), MyCustom::typeId));

  // Fail to register same component twice
  factory->Register<MyCustom>("ign_gazebo_components.MyCustom",
      new components::ComponentDescriptor<MyCustom>());

  EXPECT_EQ(registeredCount + 1, factory->TypeIds().size());

  // Fail to register 2 components with same name
  using Duplicate = components::Component<components::NoData,
      class DuplicateTag>;
  factory->Register<Duplicate>("ign_gazebo_components.MyCustom",
      new components::ComponentDescriptor<Duplicate>());

  EXPECT_EQ(registeredCount + 1, factory->TypeIds().size());

  // Unregister
  factory->Unregister<MyCustom>();

  // Check it has no type id yet
  ids = factory->TypeIds();
  EXPECT_EQ(registeredCount, ids.size());
  EXPECT_EQ(0u, MyCustom::typeId);
  EXPECT_EQ("", factory->Name(MyCustom::typeId));
}

/////////////////////////////////////////////////
TEST_F(ComponentFactoryTest, New)
{
  auto factory = components::Factory::Instance();

  {
    auto comp = factory->New(123456789);
    ASSERT_EQ(nullptr, comp);
  }

  {
    auto comp = factory->New<components::Pose>();
    ASSERT_NE(nullptr, comp);

    EXPECT_NE(0u, comp->typeId);
    EXPECT_EQ(comp->typeId, components::Pose::typeId);
  }

  {
    auto comp = factory->New(components::Pose::typeId);
    ASSERT_NE(nullptr, comp);

    EXPECT_NE(0u, comp->TypeId());

    EXPECT_NE(nullptr, static_cast<components::Pose *>(comp.get()));
  }

  {
    // Test constructing a component with pre-defined data

    // Test a valid pre-defined component
    gz::math::Pose3d pose(1, 2, 3, 4, 5, 6);
    components::Pose poseComp(pose);
    auto comp = factory->New(components::Pose::typeId, &poseComp);
    ASSERT_NE(nullptr, comp);
    EXPECT_NE(0u, comp->TypeId());
    auto derivedComp = static_cast<components::Pose *>(comp.get());
    ASSERT_NE(nullptr, derivedComp);
    EXPECT_EQ(pose, derivedComp->Data());

    // Test an invalid pre-defined component
    comp = factory->New(components::Pose::typeId, nullptr);
    ASSERT_EQ(nullptr, comp);

    // Test mistmatching component types
    comp = factory->New(components::Name::typeId, &poseComp);
    ASSERT_EQ(nullptr, comp);
  }
}


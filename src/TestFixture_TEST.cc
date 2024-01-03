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

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/ServerConfig.hh"
#include "test_config.hh"
#include "../test/helpers/EnvTestFixture.hh"
#include "gz/sim/TestFixture.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
class TestFixtureTest : public InternalFixture<::testing::Test>
{
  /// \brief Configure expectations
  public: void Configure(const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        EventManager &)
  {
    auto worldEntity = _ecm.EntityByComponents(components::Name("default"));
    EXPECT_EQ(worldEntity, _entity);

    ASSERT_NE(nullptr, _sdf);
    EXPECT_EQ("plugin", _sdf->GetName());

    EXPECT_NE(kNullEntity, _ecm.EntityByComponents(components::Name("box")));
    EXPECT_NE(kNullEntity, _ecm.EntityByComponents(components::Name("sphere")));
    EXPECT_NE(kNullEntity, _ecm.EntityByComponents(components::Name(
        "cylinder")));
  }

  /// \brief PreUpdate, Update and PostUpdate expectations
  public: void Updates(const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    EXPECT_NE(kNullEntity, _ecm.EntityByComponents(components::Name("box")));
    EXPECT_NE(kNullEntity, _ecm.EntityByComponents(components::Name("sphere")));
    EXPECT_NE(kNullEntity, _ecm.EntityByComponents(components::Name(
        "cylinder")));
  }
};

/////////////////////////////////////////////////
TEST_F(TestFixtureTest, Callbacks)
{
  TestFixture testFixture(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "shapes.sdf"));
  ASSERT_NE(nullptr, testFixture.Server());

  unsigned int configures{0u};
  unsigned int preUpdates{0u};
  unsigned int updates{0u};
  unsigned int postUpdates{0u};
  testFixture.
    OnConfigure([&](const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        EventManager &_eventMgr)
    {
      this->Configure(_entity, _sdf, _ecm, _eventMgr);
      configures++;
    }).
    OnPreUpdate([&](const UpdateInfo &_info, EntityComponentManager &_ecm)
    {
      this->Updates(_info, _ecm);
      preUpdates++;
      EXPECT_EQ(preUpdates, _info.iterations);
    }).
    OnUpdate([&](const UpdateInfo &_info, EntityComponentManager &_ecm)
    {
      this->Updates(_info, _ecm);
      updates++;
      EXPECT_EQ(updates, _info.iterations);
    }).
    OnPostUpdate([&](const UpdateInfo &_info,
        const EntityComponentManager &_ecm)
    {
      this->Updates(_info, _ecm);
      postUpdates++;
      EXPECT_EQ(postUpdates, _info.iterations);
    }).
    Finalize();

  unsigned int expectedIterations{10u};
  testFixture.Server()->Run(true, expectedIterations, false);

  EXPECT_EQ(1u, configures);
  EXPECT_EQ(expectedIterations, preUpdates);
  EXPECT_EQ(expectedIterations, updates);
  EXPECT_EQ(expectedIterations, postUpdates);
}

/////////////////////////////////////////////////
TEST_F(TestFixtureTest, LoadConfig)
{
  ServerConfig config;
  config.SetSdfFile(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "shapes.sdf"));

  TestFixture testFixture(config);
  ASSERT_NE(nullptr, testFixture.Server());

  // Check that world was loaded correctly (entities were created)
  unsigned int iterations{0u};
  testFixture.OnPostUpdate([&](const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
    {
      this->Updates(_info, _ecm);
      iterations++;
    }).Finalize();

  unsigned int expectedIterations{10u};
  testFixture.Server()->Run(true, expectedIterations, false);

  EXPECT_EQ(expectedIterations, iterations);
}

/////////////////////////////////////////////////
TEST_F(TestFixtureTest, NoFinalize)
{
  TestFixture testFixture(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "shapes.sdf"));
  ASSERT_NE(nullptr, testFixture.Server());

  unsigned int configures{0u};
  unsigned int preUpdates{0u};
  unsigned int updates{0u};
  unsigned int postUpdates{0u};
  testFixture.
    OnConfigure([&](const Entity &,
        const std::shared_ptr<const sdf::Element> &,
        EntityComponentManager &,
        EventManager &)
    {
      configures++;
    }).
    OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &)
    {
      preUpdates++;
    }).
    OnUpdate([&](const UpdateInfo &, EntityComponentManager &)
    {
      updates++;
    }).
    OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &)
    {
      postUpdates++;
    });

  unsigned int expectedIterations{10u};
  testFixture.Server()->Run(true, expectedIterations, false);

  // Callbacks not called without Finalize
  EXPECT_EQ(0u, configures);
  EXPECT_EQ(0u, preUpdates);
  EXPECT_EQ(0u, updates);
  EXPECT_EQ(0u, postUpdates);
}

/////////////////////////////////////////////////
TEST_F(TestFixtureTest, MultiFinalize)
{
  TestFixture testFixture(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "shapes.sdf"));
  ASSERT_NE(nullptr, testFixture.Server());

  unsigned int configures{0u};
  unsigned int preUpdates{0u};
  unsigned int updates{0u};
  unsigned int postUpdates{0u};
  testFixture.
    OnConfigure([&](const Entity &,
        const std::shared_ptr<const sdf::Element> &,
        EntityComponentManager &,
        EventManager &)
    {
      configures++;
    }).
    OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &)
    {
      preUpdates++;
    }).
    OnUpdate([&](const UpdateInfo &, EntityComponentManager &)
    {
      updates++;
    }).
    OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &)
    {
      postUpdates++;
    }).
    // Nothing is messed up if finalize is called multiple times
    Finalize().
    Finalize().
    Finalize();

  unsigned int expectedIterations{10u};
  testFixture.Server()->Run(true, expectedIterations, false);

  EXPECT_EQ(1u, configures);
  EXPECT_EQ(expectedIterations, preUpdates);
  EXPECT_EQ(expectedIterations, updates);
  EXPECT_EQ(expectedIterations, postUpdates);
}

/////////////////////////////////////////////////
TEST_F(TestFixtureTest, ChangeCallback)
{
  TestFixture testFixture(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "shapes.sdf"));
  ASSERT_NE(nullptr, testFixture.Server());

  unsigned int preUpdate1{0u};
  testFixture.
    OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &)
    {
      preUpdate1++;
    }).
    Finalize();

  unsigned int expectedIterations{10u};
  testFixture.Server()->Run(true, expectedIterations, false);

  EXPECT_EQ(expectedIterations, preUpdate1);

  unsigned int preUpdate2{0u};
  testFixture.
    OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &)
    {
      preUpdate2++;
    }).
    Finalize();

  testFixture.Server()->Run(true, expectedIterations, false);

  // First callback isn't called anymore
  EXPECT_EQ(expectedIterations, preUpdate1);

  // New callback is called
  EXPECT_EQ(expectedIterations, preUpdate2);
}

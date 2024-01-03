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

#include "EntityFeatureMap.hh"

#include <gtest/gtest.h>

#include <gz/physics/BoxShape.hh>
#include <gz/physics/CylinderShape.hh>
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/Entity.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/Implements.hh>
#include <gz/physics/InstallationDirectories.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/config.hh>
#include <gz/plugin/Loader.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "../../../test/helpers/EnvTestFixture.hh"
#include "gz/sim/EntityComponentManager.hh"

using namespace gz;
using namespace gz::sim::systems::physics_system;

struct MinimumFeatureList
    : physics::FeatureList<physics::ConstructEmptyWorldFeature,
                           physics::ConstructEmptyModelFeature,
                           physics::ConstructEmptyLinkFeature>
{
};

using EnginePtrType =
    physics::EnginePtr<physics::FeaturePolicy3d, MinimumFeatureList>;


class EntityFeatureMapFixture: public InternalFixture<::testing::Test>
{
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    const std::string pluginLib = "gz-physics-dartsim-plugin";

    common::SystemPaths systemPaths;
    systemPaths.AddPluginPaths(gz::physics::getEngineInstallDir());

    auto pathToLib = systemPaths.FindSharedLibrary(pluginLib);
    ASSERT_FALSE(pathToLib.empty())
        << "Failed to find plugin [" << pluginLib << "]";

    // Load engine plugin
    plugin::Loader pluginLoader;
    auto plugins = pluginLoader.LoadLib(pathToLib);
    ASSERT_FALSE(plugins.empty())
        << "Unable to load the [" << pathToLib << "] library.";

    auto classNames = pluginLoader.AllPlugins();
    ASSERT_FALSE(classNames.empty())
        << "No plugins found in library [" << pathToLib << "].";

    for (const auto &className : classNames)
    {
      // Get the first plugin that works
      auto plugin = pluginLoader.Instantiate(className);

      ASSERT_TRUE(plugin) << "Class " << className << " not found in library ["
                          << pathToLib << "]";

      this->engine =
          physics::RequestEngine<physics::FeaturePolicy3d,
                                           MinimumFeatureList>::From(plugin);

      ASSERT_NE(nullptr, this->engine);
      break;
    }
  }

  public: EnginePtrType engine;
};

// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(EntityFeatureMapFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(AddCastRemoveEntity))
{
  struct TestOptionalFeatures1
      : physics::FeatureList<physics::LinkFrameSemantics>
  {
  };
  using TestOptionalFeatures2 = physics::FeatureList<physics::RemoveEntities>;

  using WorldEntityMap =
      EntityFeatureMap3d<physics::World, MinimumFeatureList,
                         TestOptionalFeatures1, TestOptionalFeatures2>;

  using WorldPtrType = physics::EntityPtr<
      physics::World<physics::FeaturePolicy3d, MinimumFeatureList>>;

  // Making these entities different from 1 and 2 ensures that the implicit
  // conversion in gz-physics between EntityPtr and std::size_t doesn't cause
  // false positive tests
  sim::Entity gazeboWorld1Entity = 123;
  sim::Entity gazeboWorld2Entity = 456;
  WorldPtrType testWorld1 = this->engine->ConstructEmptyWorld("world1");
  WorldEntityMap testMap;
  EXPECT_FALSE(testMap.HasEntity(gazeboWorld1Entity));
  EXPECT_EQ(nullptr, testMap.Get(gazeboWorld1Entity));
  EXPECT_EQ(sim::kNullEntity, testMap.Get(testWorld1));
  EXPECT_EQ(0u, testMap.TotalMapEntryCount());

  testMap.AddEntity(gazeboWorld1Entity, testWorld1);

  // After adding the entity, there should be one entry each in four maps
  EXPECT_EQ(4u, testMap.TotalMapEntryCount());
  EXPECT_EQ(testWorld1, testMap.Get(gazeboWorld1Entity));
  EXPECT_EQ(gazeboWorld1Entity, testMap.Get(testWorld1));
  EXPECT_EQ(gazeboWorld1Entity, testMap.GetByPhysicsId(testWorld1->EntityID()));

  // Cast to optional feature1
  auto testWorld1Feature1 =
      testMap.EntityCast<TestOptionalFeatures1>(gazeboWorld1Entity);
  ASSERT_NE(nullptr, testWorld1Feature1);
  // After the cast, there should be one more entry in the cache map.
  EXPECT_EQ(5u, testMap.TotalMapEntryCount());

  // Cast to optional feature2
  auto testWorld1Feature2 =
      testMap.EntityCast<TestOptionalFeatures2>(gazeboWorld1Entity);
  ASSERT_NE(nullptr, testWorld1Feature2);
  // After the cast, the number of entries should remain the same because we
  // have not added an entity.
  EXPECT_EQ(5u, testMap.TotalMapEntryCount());

  // Add another entity
  WorldPtrType testWorld2 = this->engine->ConstructEmptyWorld("world2");
  testMap.AddEntity(gazeboWorld2Entity, testWorld2);
  EXPECT_EQ(9u, testMap.TotalMapEntryCount());
  EXPECT_EQ(testWorld2, testMap.Get(gazeboWorld2Entity));
  EXPECT_EQ(gazeboWorld2Entity, testMap.Get(testWorld2));
  EXPECT_EQ(gazeboWorld2Entity, testMap.GetByPhysicsId(testWorld2->EntityID()));

  auto testWorld2Feature1 =
      testMap.EntityCast<TestOptionalFeatures1>(testWorld2);
  ASSERT_NE(nullptr, testWorld2Feature1);
  // After the cast, there should be one more entry in the cache map.
  EXPECT_EQ(10u, testMap.TotalMapEntryCount());

  auto testWorld2Feature2 =
      testMap.EntityCast<TestOptionalFeatures2>(testWorld2);
  ASSERT_NE(nullptr, testWorld2Feature2);
  // After the cast, the number of entries should remain the same because we
  // have not added an entity.
  EXPECT_EQ(10u, testMap.TotalMapEntryCount());

  // Remove entitites
  testMap.Remove(gazeboWorld1Entity);
  EXPECT_FALSE(testMap.HasEntity(gazeboWorld1Entity));
  EXPECT_EQ(nullptr, testMap.Get(gazeboWorld1Entity));
  EXPECT_EQ(sim::kNullEntity, testMap.Get(testWorld1));
  EXPECT_EQ(sim::kNullEntity,
      testMap.GetByPhysicsId(testWorld1->EntityID()));
  EXPECT_EQ(5u, testMap.TotalMapEntryCount());

  testMap.Remove(testWorld2);
  EXPECT_FALSE(testMap.HasEntity(gazeboWorld2Entity));
  EXPECT_EQ(nullptr, testMap.Get(gazeboWorld2Entity));
  EXPECT_EQ(sim::kNullEntity, testMap.Get(testWorld2));
  EXPECT_EQ(sim::kNullEntity, testMap.GetByPhysicsId(
      testWorld2->EntityID()));
  EXPECT_EQ(0u, testMap.TotalMapEntryCount());
}

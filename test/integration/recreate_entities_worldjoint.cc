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

#include <algorithm>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/utilities/ExtraTestMacros.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Recreate.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

using namespace std::chrono_literals;

class RecreateEntitiesFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(RecreateEntitiesFixture,
    IGN_UTILS_TEST_DISABLED_ON_WIN32(RecreateEntities_WorldJoint))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "fixed_world_joint.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  EntityComponentManager *ecm = nullptr;
  bool recreateEntities = false;

  // Create a system that adds a recreate component to models
  test::Relay testSystem;
  testSystem.OnUpdate([&](const gazebo::UpdateInfo &,
    gazebo::EntityComponentManager &_ecm)
    {
      // cppcheck-suppress knownConditionTrueFalse
      if (!ecm)
      {
        ecm = &_ecm;
        recreateEntities = true;
        return;
      }

      if (recreateEntities)
      {
        _ecm.Each<components::Model>(
            [&](const ignition::gazebo::Entity &_entity,
                const components::Model *) -> bool
            {
              // add a components::Recreate to indicate that this entity
              // needs to be recreated
              _ecm.CreateComponent(_entity, components::Recreate());
              return true;
            });

        // recreate entities only once so set it back to false
        recreateEntities = false;
        return;
      }
    });
  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  auto validateEntities = [&]()
  {
    // Check entities
    // 1 x world + 1 x (default) level + 1 x wind + 1 x model + 2 x link + 2 x
    // collision + 2 x visual + 2 x joint
    EXPECT_EQ(12u, ecm->EntityCount());

    Entity worldEntity =
        ecm->EntityByComponents(components::World());
    EXPECT_NE(kNullEntity, worldEntity);

    // Check models
    unsigned int modelCount{0};
    ecm->Each<components::Model,
                              components::Pose,
                              components::ParentEntity,
                              components::Name>(
      [&](const Entity &/*_entity*/,
          const components::Model *_model,
          const components::Pose *_pose,
          const components::ParentEntity *_parent,
          const components::Name *_name)->bool
      {
        EXPECT_NE(nullptr, _model);
        EXPECT_NE(nullptr, _pose);
        EXPECT_NE(nullptr, _parent);
        EXPECT_NE(nullptr, _name);

        modelCount++;

        EXPECT_EQ(worldEntity, _parent->Data());
        return true;
      });

    EXPECT_EQ(1u, modelCount);

    // Check links
    unsigned int linkCount{0};
    ecm->Each<components::Link,
              components::Pose,
              components::ParentEntity,
              components::Name>(
      [&](const Entity &/*_entity*/,
          const components::Link *_link,
          const components::Pose *_pose,
          const components::ParentEntity *_parent,
          const components::Name *_name)->bool
      {
        EXPECT_NE(nullptr, _link);
        EXPECT_NE(nullptr, _pose);
        EXPECT_NE(nullptr, _parent);
        EXPECT_NE(nullptr, _name);

        linkCount++;

        return true;
      });

    EXPECT_EQ(2u, linkCount);

    // Check joint
    unsigned int jointCount{0};
    ecm->Each<components::Joint,
              components::Pose,
              components::ParentEntity,
              components::Name,
              components::ParentLinkName>(
      [&](const Entity &/*_entity*/,
          const components::Joint *_joint,
          const components::Pose *_pose,
          const components::ParentEntity *_parent,
          const components::Name *_name,
          const components::ParentLinkName *_parentLinkName)->bool
      {
        EXPECT_NE(nullptr, _joint);
        EXPECT_NE(nullptr, _pose);
        EXPECT_NE(nullptr, _parent);
        EXPECT_NE(nullptr, _name);
        EXPECT_NE(nullptr, _parentLinkName);

        std::cout << "JointName[" << _name->Data() << "]\n";
        if (_name->Data() == "world_fixed")
        {
          EXPECT_EQ("world", _parentLinkName->Data());
        }
        jointCount++;

        return true;
      });

    EXPECT_EQ(2u, jointCount);
  };

  // validate initial state
  validateEntities();

  // Run once to let the test relay system creates the components::Recreate
  server.Run(true, 1, false);

  // Run again so that entities get recreated in the ECM
  server.Run(true, 1, false);

  // validate that the entities are recreated
  validateEntities();

  // Run again to make sure the recreate components are removed and no
  // entities to to be recreated
  server.Run(true, 1, false);

  auto entities = ecm->EntitiesByComponents(components::Model(),
      components::Recreate());
  EXPECT_TRUE(entities.empty());
}

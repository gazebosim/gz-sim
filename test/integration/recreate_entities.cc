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

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/utils/ExtraTestMacros.hh>
#include <sdf/World.hh>

#include "gz/sim/components/Factory.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Recreate.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace components;

using namespace std::chrono_literals;

class RecreateEntitiesFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(RecreateEntitiesFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(RecreateEntities))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "shapes.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  EntityComponentManager *ecm = nullptr;
  bool recreateEntities = false;

  // Create a system that adds a recreate component to models
  test::Relay testSystem;
  testSystem.OnUpdate([&](const sim::UpdateInfo &,
    sim::EntityComponentManager &_ecm)
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
            [&](const gz::sim::Entity &_entity,
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

  // model entity ids
  Entity boxModelEntity = kNullEntity;
  Entity cylModelEntity = kNullEntity;
  Entity sphModelEntity = kNullEntity;
  Entity capModelEntity = kNullEntity;
  Entity ellipModelEntity = kNullEntity;

  // link entity ids
  Entity boxLinkEntity = kNullEntity;
  Entity cylLinkEntity = kNullEntity;
  Entity sphLinkEntity = kNullEntity;
  Entity capLinkEntity = kNullEntity;
  Entity ellipLinkEntity = kNullEntity;

  auto validateEntities = [&]()
  {
    boxModelEntity = kNullEntity;
    cylModelEntity = kNullEntity;
    sphModelEntity = kNullEntity;
    capModelEntity = kNullEntity;
    ellipModelEntity = kNullEntity;

    boxLinkEntity = kNullEntity;
    cylLinkEntity = kNullEntity;
    sphLinkEntity = kNullEntity;
    capLinkEntity = kNullEntity;
    ellipLinkEntity = kNullEntity;

    // Check entities
    // 1 x world + 1 x (default) level + 1 x wind + 5 x model + 5 x link + 5 x
    // collision + 5 x visual + 1 x light (light + visual)
    EXPECT_EQ(25u, ecm->EntityCount());

    Entity worldEntity =
        ecm->EntityByComponents(components::World());
    EXPECT_NE(kNullEntity, worldEntity);

    // Check models
    unsigned int modelCount{0};
    ecm->Each<components::Model,
                              components::Pose,
                              components::ParentEntity,
                              components::Name>(
      [&](const Entity &_entity,
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
        if (modelCount == 1)
        {
          EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 1),
              _pose->Data());
          EXPECT_EQ("box", _name->Data());
          boxModelEntity = _entity;
        }
        else if (modelCount == 2)
        {
          EXPECT_EQ(gz::math::Pose3d(-1, -2, -3, 0, 0, 1),
              _pose->Data());
          EXPECT_EQ("cylinder", _name->Data());
          cylModelEntity = _entity;
        }
        else if (modelCount == 3)
        {
          EXPECT_EQ(gz::math::Pose3d(0, 0, 0, 0, 0, 1),
              _pose->Data());
          EXPECT_EQ("sphere", _name->Data());
          sphModelEntity = _entity;
        }
        else if (modelCount == 4)
        {
          EXPECT_EQ(gz::math::Pose3d(-4, -5, -6, 0, 0, 1),
              _pose->Data());
          EXPECT_EQ("capsule", _name->Data());
          capModelEntity = _entity;
        }
        else if (modelCount == 5)
        {
          EXPECT_EQ(gz::math::Pose3d(4, 5, 6, 0, 0, 1),
              _pose->Data());
          EXPECT_EQ("ellipsoid", _name->Data());
          ellipModelEntity = _entity;
        }
        return true;
      });

    EXPECT_EQ(5u, modelCount);
    EXPECT_NE(kNullEntity, boxModelEntity);
    EXPECT_NE(kNullEntity, cylModelEntity);
    EXPECT_NE(kNullEntity, sphModelEntity);
    EXPECT_NE(kNullEntity, capModelEntity);
    EXPECT_NE(kNullEntity, ellipModelEntity);

    // Check links
    unsigned int linkCount{0};
    ecm->Each<components::Link,
              components::Pose,
              components::ParentEntity,
              components::Name>(
      [&](const Entity &_entity,
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

        if (linkCount == 1)
        {
          EXPECT_EQ(gz::math::Pose3d(0.1, 0.1, 0.1, 0, 0, 0),
              _pose->Data());
          EXPECT_EQ("box_link", _name->Data());
          EXPECT_EQ(boxModelEntity, _parent->Data());
          boxLinkEntity = _entity;
        }
        else if (linkCount == 2)
        {
          EXPECT_EQ(gz::math::Pose3d(0.2, 0.2, 0.2, 0, 0, 0),
              _pose->Data());
          EXPECT_EQ("cylinder_link", _name->Data());
          EXPECT_EQ(cylModelEntity, _parent->Data());
          cylLinkEntity = _entity;
        }
        else if (linkCount == 3)
        {
          EXPECT_EQ(gz::math::Pose3d(0.3, 0.3, 0.3, 0, 0, 0),
              _pose->Data());
          EXPECT_EQ("sphere_link", _name->Data());
          EXPECT_EQ(sphModelEntity, _parent->Data());
          sphLinkEntity = _entity;
        }
        else if (linkCount == 4)
        {
          EXPECT_EQ(gz::math::Pose3d(0.5, 0.5, 0.5, 0, 0, 0),
              _pose->Data());
          EXPECT_EQ("capsule_link", _name->Data());
          EXPECT_EQ(capModelEntity, _parent->Data());
          capLinkEntity = _entity;
        }
        else if (linkCount == 5)
        {
          EXPECT_EQ(gz::math::Pose3d(0.8, 0.8, 0.8, 0, 0, 0),
              _pose->Data());
          EXPECT_EQ("ellipsoid_link", _name->Data());
          EXPECT_EQ(ellipModelEntity, _parent->Data());
          ellipLinkEntity = _entity;
        }
        return true;
      });

    EXPECT_EQ(5u, linkCount);
    EXPECT_NE(kNullEntity, boxLinkEntity);
    EXPECT_NE(kNullEntity, cylLinkEntity);
    EXPECT_NE(kNullEntity, sphLinkEntity);
    EXPECT_NE(kNullEntity, capLinkEntity);
    EXPECT_NE(kNullEntity, ellipLinkEntity);
  };

  // validate initial state
  validateEntities();

  // store the original entity ids
  Entity originalboxModelEntity = boxModelEntity;
  Entity originalcylModelEntity = cylModelEntity;
  Entity originalsphModelEntity = sphModelEntity;
  Entity originalcapModelEntity = capModelEntity;
  Entity originalellipModelEntity = ellipModelEntity;
  Entity originalboxLinkEntity = boxLinkEntity;
  Entity originalcylLinkEntity = cylLinkEntity;
  Entity originalsphLinkEntity = sphLinkEntity;
  Entity originalcapLinkEntity = capLinkEntity;
  Entity originalellipLinkEntity = ellipLinkEntity;

  // Run once to let the test relay system creates the components::Recreate
  server.Run(true, 1, false);

  // Run again so that entities get recreated in the ECM
  server.Run(true, 1, false);

  // validate that the entities are recreated
  validateEntities();

  // check that the newly recreated entities should have a different entity id
  EXPECT_NE(originalboxModelEntity, boxModelEntity);
  EXPECT_NE(originalcylModelEntity, cylModelEntity);
  EXPECT_NE(originalsphModelEntity, sphModelEntity);
  EXPECT_NE(originalcapModelEntity, capModelEntity);
  EXPECT_NE(originalellipModelEntity, ellipModelEntity);
  EXPECT_NE(originalboxLinkEntity, boxLinkEntity);
  EXPECT_NE(originalcylLinkEntity, cylLinkEntity);
  EXPECT_NE(originalsphLinkEntity, sphLinkEntity);
  EXPECT_NE(originalcapLinkEntity, capLinkEntity);
  EXPECT_NE(originalellipLinkEntity, ellipLinkEntity);

  // Run again to make sure the recreate components are removed and no
  // entities to to be recreated
  server.Run(true, 1, false);

  auto entities = ecm->EntitiesByComponents(components::Model(),
      components::Recreate());
  EXPECT_TRUE(entities.empty());
}

/////////////////////////////////////////////////
TEST_F(RecreateEntitiesFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(RecreateEntities_Joints))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "double_pendulum.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  EntityComponentManager *ecm = nullptr;
  bool recreateEntities = false;

  // Create a system that adds a recreate component to models
  test::Relay testSystem;
  testSystem.OnUpdate([&](const sim::UpdateInfo &,
    sim::EntityComponentManager &_ecm)
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
            [&](const gz::sim::Entity &_entity,
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
    // 1 x world + 1 x (default) level + 1 x wind + 5 x model + 5 x link + 5 x
    // collision + 5 x visual + 1 x light (light + visual)
    EXPECT_EQ(49u, ecm->EntityCount());

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

    EXPECT_EQ(3u, modelCount);

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

    EXPECT_EQ(7u, linkCount);

    // Check links
    unsigned int jointCount{0};
    ecm->Each<components::Joint,
              components::Pose,
              components::ParentEntity,
              components::Name>(
      [&](const Entity &/*_entity*/,
          const components::Joint *_joint,
          const components::Pose *_pose,
          const components::ParentEntity *_parent,
          const components::Name *_name)->bool
      {
        EXPECT_NE(nullptr, _joint);
        EXPECT_NE(nullptr, _pose);
        EXPECT_NE(nullptr, _parent);
        EXPECT_NE(nullptr, _name);

        jointCount++;

        return true;
      });

    EXPECT_EQ(4u, jointCount);
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

/////////////////////////////////////////////////
TEST_F(RecreateEntitiesFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(RecreateEntities_WorldJoint))
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
  testSystem.OnUpdate([&](const sim::UpdateInfo &,
    sim::EntityComponentManager &_ecm)
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
            [&](const gz::sim::Entity &_entity,
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

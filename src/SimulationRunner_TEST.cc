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
#include <ignition/common/Console.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>

#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/components/ChildEntity.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;

class SimulationRunnerTest : public ::testing::TestWithParam<int>
{
};

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, CreateEntities)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  std::vector<SystemPluginPtr> systems;
  SimulationRunner runner(root.WorldByIndex(0), systems);

  // Check component types
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::World>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Model>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Link>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Collision>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Visual>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Name>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::ParentEntity>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Geometry>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Material>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Inertial>()));

  // Check entities
  // 1 x world + 3 x model + 3 x link + 3 x collision + 3 x visual
  EXPECT_EQ(13u, runner.entityCompMgr.EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  EntityId worldEntity = kNullEntity;
  runner.entityCompMgr.Each<components::World,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::World *_world,
        const components::Name *_name)
    {
      ASSERT_NE(nullptr, _world);
      ASSERT_NE(nullptr, _name);

      EXPECT_EQ("default", _name->Data());

      worldCount++;

      worldEntity = _entity;
    });

  EXPECT_EQ(1u, worldCount);
  EXPECT_NE(kNullEntity, worldEntity);

  // Check models
  unsigned int modelCount{0};
  EntityId boxModelEntity = kNullEntity;
  EntityId cylModelEntity = kNullEntity;
  EntityId sphModelEntity = kNullEntity;
  runner.entityCompMgr.Each<components::Model,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::Model *_model,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)
    {
      ASSERT_NE(nullptr, _model);
      ASSERT_NE(nullptr, _pose);
      ASSERT_NE(nullptr, _parent);
      ASSERT_NE(nullptr, _name);

      modelCount++;

      EXPECT_EQ(worldEntity, _parent->Id());
      if (modelCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("box", _name->Data());
        boxModelEntity = _entity;
      }
      else if (modelCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(-1, -2, -3, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("cylinder", _name->Data());
        cylModelEntity = _entity;
      }
      else if (modelCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("sphere", _name->Data());
        sphModelEntity = _entity;
      }
    });

  EXPECT_EQ(3u, modelCount);
  EXPECT_NE(kNullEntity, boxModelEntity);
  EXPECT_NE(kNullEntity, cylModelEntity);
  EXPECT_NE(kNullEntity, sphModelEntity);

  // Check links
  unsigned int linkCount{0};
  EntityId boxLinkEntity = kNullEntity;
  EntityId cylLinkEntity = kNullEntity;
  EntityId sphLinkEntity = kNullEntity;
  runner.entityCompMgr.Each<components::Link,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::Link *_link,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)
    {
      ASSERT_NE(nullptr, _link);
      ASSERT_NE(nullptr, _pose);
      ASSERT_NE(nullptr, _parent);
      ASSERT_NE(nullptr, _name);

      linkCount++;

      if (linkCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.1, 0.1, 0.1, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("box_link", _name->Data());
        EXPECT_EQ(boxModelEntity, _parent->Id());
        boxLinkEntity = _entity;
      }
      else if (linkCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.2, 0.2, 0.2, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("cylinder_link", _name->Data());
        EXPECT_EQ(cylModelEntity, _parent->Id());
        cylLinkEntity = _entity;
      }
      else if (linkCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.3, 0.3, 0.3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("sphere_link", _name->Data());
        EXPECT_EQ(sphModelEntity, _parent->Id());
        sphLinkEntity = _entity;
      }
    });

  EXPECT_EQ(3u, linkCount);
  EXPECT_NE(kNullEntity, boxLinkEntity);
  EXPECT_NE(kNullEntity, cylLinkEntity);
  EXPECT_NE(kNullEntity, sphLinkEntity);

  // Check inertials
  unsigned int inertialCount{0};
  runner.entityCompMgr.Each<components::Link, components::Inertial>(
    [&](const EntityId & _entity,
        const components::Link *_link,
        const components::Inertial *_inertial)
    {
      ASSERT_NE(nullptr, _link);
      ASSERT_NE(nullptr, _inertial);

      inertialCount++;

      if (_entity == boxLinkEntity)
      {
        EXPECT_EQ(math::MassMatrix3d(1.0, math::Vector3d(1.0, 1.0, 1.0),
                                     math::Vector3d::Zero),
                  _inertial->Data().MassMatrix());
      }
      else if (_entity == cylLinkEntity)
      {
        EXPECT_EQ(math::MassMatrix3d(2.0, math::Vector3d(2.0, 2.0, 2.0),
                                     math::Vector3d::Zero),
                  _inertial->Data().MassMatrix());
      }
      else if (_entity == sphLinkEntity)
      {
        EXPECT_EQ(math::MassMatrix3d(3.0, math::Vector3d(3.0, 3.0, 3.0),
                                     math::Vector3d::Zero),
                  _inertial->Data().MassMatrix());
      }
    });

  EXPECT_EQ(3u, inertialCount);

  // Check collisions
  unsigned int collisionCount{0};
  runner.entityCompMgr.Each<components::Collision,
                            components::Geometry,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &/*_entity*/,
        const components::Collision *_collision,
        const components::Geometry *_geometry,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)
    {
      ASSERT_NE(nullptr, _collision);
      ASSERT_NE(nullptr, _geometry);
      ASSERT_NE(nullptr, _pose);
      ASSERT_NE(nullptr, _parent);
      ASSERT_NE(nullptr, _name);

      collisionCount++;

      if (collisionCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.11, 0.11, 0.11, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("box_collision", _name->Data());

        EXPECT_EQ(boxLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::BOX, _geometry->Data().Type());
        ASSERT_NE(nullptr, _geometry->Data().BoxShape());
        EXPECT_EQ(math::Vector3d(3, 4, 5),
                  _geometry->Data().BoxShape()->Size());
      }
      else if (collisionCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.21, 0.21, 0.21, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("cylinder_collision", _name->Data());

        EXPECT_EQ(cylLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::CYLINDER, _geometry->Data().Type());
        ASSERT_NE(nullptr, _geometry->Data().CylinderShape());
        EXPECT_DOUBLE_EQ(0.2, _geometry->Data().CylinderShape()->Radius());
        EXPECT_DOUBLE_EQ(0.1, _geometry->Data().CylinderShape()->Length());
      }
      else if (collisionCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.31, 0.31, 0.31, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("sphere_collision", _name->Data());

        EXPECT_EQ(sphLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
        ASSERT_NE(nullptr, _geometry->Data().SphereShape());
        EXPECT_DOUBLE_EQ(23.4, _geometry->Data().SphereShape()->Radius());
      }
    });

  EXPECT_EQ(3u, collisionCount);

  // Check visuals
  unsigned int visualCount{0};
  runner.entityCompMgr.Each<components::Visual,
                            components::Geometry,
                            components::Material,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &/*_entity*/,
        const components::Visual *_visual,
        const components::Geometry *_geometry,
        const components::Material *_material,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)
    {
      ASSERT_NE(nullptr, _visual);
      ASSERT_NE(nullptr, _geometry);
      ASSERT_NE(nullptr, _material);
      ASSERT_NE(nullptr, _pose);
      ASSERT_NE(nullptr, _parent);
      ASSERT_NE(nullptr, _name);

      visualCount++;

      if (visualCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.12, 0.12, 0.12, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("box_visual", _name->Data());

        EXPECT_EQ(boxLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::BOX, _geometry->Data().Type());
        ASSERT_NE(nullptr, _geometry->Data().BoxShape());
        EXPECT_EQ(math::Vector3d(1, 2, 3),
                  _geometry->Data().BoxShape()->Size());

        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Ambient());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Specular());
      }
      else if (visualCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.22, 0.22, 0.22, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("cylinder_visual", _name->Data());

        EXPECT_EQ(cylLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::CYLINDER, _geometry->Data().Type());
        ASSERT_NE(nullptr, _geometry->Data().CylinderShape());
        EXPECT_DOUBLE_EQ(2.1, _geometry->Data().CylinderShape()->Radius());
        EXPECT_DOUBLE_EQ(10.2, _geometry->Data().CylinderShape()->Length());

        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Specular());
      }
      else if (visualCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.32, 0.32, 0.32, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("sphere_visual", _name->Data());

        EXPECT_EQ(sphLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
        ASSERT_NE(nullptr, _geometry->Data().SphereShape());
        EXPECT_DOUBLE_EQ(1.2, _geometry->Data().SphereShape()->Radius());

        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Specular());
      }
    });

  EXPECT_EQ(3u, visualCount);
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, Time)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  std::vector<SystemPluginPtr> systems;
  SimulationRunner runner(root.WorldByIndex(0), systems);

  // Check state
  EXPECT_TRUE(runner.Paused());
  EXPECT_EQ(0u, runner.currentInfo.iterations);
  EXPECT_EQ(0ms, runner.currentInfo.simTime);
  EXPECT_EQ(0ms, runner.currentInfo.dt);
  EXPECT_EQ(1ms, runner.updatePeriod);
  EXPECT_EQ(1ms, runner.stepSize);

  runner.SetPaused(false);

  // Run
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(100u, runner.currentInfo.iterations);
  EXPECT_EQ(100ms, runner.currentInfo.simTime);
  EXPECT_EQ(1ms, runner.currentInfo.dt);
  EXPECT_EQ(1ms, runner.updatePeriod);
  EXPECT_EQ(1ms, runner.stepSize);

  // Change step size and run
  runner.stepSize = 2ms;
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(200u, runner.currentInfo.iterations);
  EXPECT_EQ(300ms, runner.currentInfo.simTime);
  EXPECT_EQ(2ms, runner.currentInfo.dt);
  EXPECT_EQ(1ms, runner.updatePeriod);
  EXPECT_EQ(2ms, runner.stepSize);

  // Set paused
  runner.SetPaused(true);
  EXPECT_TRUE(runner.Paused());
  runner.SetPaused(false);
  EXPECT_FALSE(runner.Paused());

  // Unpause and run
  runner.SetPaused(false);
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(300u, runner.currentInfo.iterations);
  EXPECT_EQ(500ms, runner.currentInfo.simTime)
    << runner.currentInfo.simTime.count();
  EXPECT_EQ(2ms, runner.currentInfo.dt);
  EXPECT_EQ(1ms, runner.updatePeriod);
  EXPECT_EQ(2ms, runner.stepSize);
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(ServerRepeat, SimulationRunnerTest,
    ::testing::Range(1, 2));

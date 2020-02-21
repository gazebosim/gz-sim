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
#include <tinyxml2.h>

#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>


#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/Wind.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/config.hh"
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
using IntComponent = components::Component<int, class IntComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.IntComponent",
    IntComponent)

using DoubleComponent = components::Component<double, class DoubleComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.DoubleComponent",
    DoubleComponent)
}
}
}
}

class SimulationRunnerTest : public ::testing::TestWithParam<int>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);

    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
      (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

std::vector<msgs::Clock> clockMsgs;
std::vector<msgs::Clock> rootClockMsgs;

/////////////////////////////////////////////////
void clockCb(const msgs::Clock &_msg)
{
  clockMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void rootClockCb(const msgs::Clock &_msg)
{
  rootClockMsgs.push_back(_msg);
}


/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, CreateEntities)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(root.WorldByIndex(0), systemLoader);

  // Check component types
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::World::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Model::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::CanonicalLink::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Link::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Collision::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Visual::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Light::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Name::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::ParentEntity::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Geometry::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Material::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Inertial::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Wind::typeId));

  // Check entities
  // 1 x world + 1 x (default) level + 1 x wind + 3 x model + 3 x link + 3 x
  // collision + 3 x visual + 1 x light
  EXPECT_EQ(16u, runner.EntityCompMgr().EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  Entity worldEntity = kNullEntity;
  runner.EntityCompMgr().Each<components::World,
                            components::Name>(
    [&](const Entity &_entity,
        const components::World *_world,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _world);
      EXPECT_NE(nullptr, _name);

      EXPECT_EQ("default", _name->Data());

      worldCount++;

      worldEntity = _entity;
      return true;
    });

  EXPECT_EQ(1u, worldCount);
  EXPECT_NE(kNullEntity, worldEntity);

  // Check models
  unsigned int modelCount{0};
  Entity boxModelEntity = kNullEntity;
  Entity cylModelEntity = kNullEntity;
  Entity sphModelEntity = kNullEntity;
  runner.EntityCompMgr().Each<components::Model,
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
      return true;
    });

  EXPECT_EQ(3u, modelCount);
  EXPECT_NE(kNullEntity, boxModelEntity);
  EXPECT_NE(kNullEntity, cylModelEntity);
  EXPECT_NE(kNullEntity, sphModelEntity);

  // Check links
  unsigned int linkCount{0};
  Entity boxLinkEntity = kNullEntity;
  Entity cylLinkEntity = kNullEntity;
  Entity sphLinkEntity = kNullEntity;
  runner.EntityCompMgr().Each<components::Link,
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
        EXPECT_EQ(ignition::math::Pose3d(0.1, 0.1, 0.1, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("box_link", _name->Data());
        EXPECT_EQ(boxModelEntity, _parent->Data());
        boxLinkEntity = _entity;
      }
      else if (linkCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.2, 0.2, 0.2, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("cylinder_link", _name->Data());
        EXPECT_EQ(cylModelEntity, _parent->Data());
        cylLinkEntity = _entity;
      }
      else if (linkCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.3, 0.3, 0.3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("sphere_link", _name->Data());
        EXPECT_EQ(sphModelEntity, _parent->Data());
        sphLinkEntity = _entity;
      }
      return true;
    });

  EXPECT_EQ(3u, linkCount);
  EXPECT_NE(kNullEntity, boxLinkEntity);
  EXPECT_NE(kNullEntity, cylLinkEntity);
  EXPECT_NE(kNullEntity, sphLinkEntity);

  // Check inertials
  unsigned int inertialCount{0};
  runner.EntityCompMgr().Each<components::Link, components::Inertial>(
    [&](const Entity & _entity,
        const components::Link *_link,
        const components::Inertial *_inertial)->bool
    {
      EXPECT_NE(nullptr, _link);
      EXPECT_NE(nullptr, _inertial);

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
      return true;
    });

  EXPECT_EQ(3u, inertialCount);

  // Check collisions
  unsigned int collisionCount{0};
  runner.EntityCompMgr().Each<components::Collision,
                            components::Geometry,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const components::Collision *_collision,
        const components::Geometry *_geometry,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _collision);
      EXPECT_NE(nullptr, _geometry);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      collisionCount++;

      if (collisionCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.11, 0.11, 0.11, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("box_collision", _name->Data());

        EXPECT_EQ(boxLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::BOX, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().BoxShape());
        EXPECT_EQ(math::Vector3d(3, 4, 5),
                  _geometry->Data().BoxShape()->Size());
      }
      else if (collisionCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.21, 0.21, 0.21, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("cylinder_collision", _name->Data());

        EXPECT_EQ(cylLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::CYLINDER, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().CylinderShape());
        EXPECT_DOUBLE_EQ(0.2, _geometry->Data().CylinderShape()->Radius());
        EXPECT_DOUBLE_EQ(0.1, _geometry->Data().CylinderShape()->Length());
      }
      else if (collisionCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.31, 0.31, 0.31, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("sphere_collision", _name->Data());

        EXPECT_EQ(sphLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().SphereShape());
        EXPECT_DOUBLE_EQ(23.4, _geometry->Data().SphereShape()->Radius());
      }
      return true;
    });

  EXPECT_EQ(3u, collisionCount);

  // Check visuals
  unsigned int visualCount{0};
  runner.EntityCompMgr().Each<components::Visual,
                            components::Geometry,
                            components::Material,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const components::Visual *_visual,
        const components::Geometry *_geometry,
        const components::Material *_material,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _visual);
      EXPECT_NE(nullptr, _geometry);
      EXPECT_NE(nullptr, _material);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      visualCount++;

      if (visualCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.12, 0.12, 0.12, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("box_visual", _name->Data());

        EXPECT_EQ(boxLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::BOX, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().BoxShape());
        EXPECT_EQ(math::Vector3d(1, 2, 3),
                  _geometry->Data().BoxShape()->Size());

        EXPECT_EQ(math::Color(0, 0, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Ambient());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Specular());
      }
      else if (visualCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.22, 0.22, 0.22, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("cylinder_visual", _name->Data());

        EXPECT_EQ(cylLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::CYLINDER, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().CylinderShape());
        EXPECT_DOUBLE_EQ(2.1, _geometry->Data().CylinderShape()->Radius());
        EXPECT_DOUBLE_EQ(10.2, _geometry->Data().CylinderShape()->Length());

        EXPECT_EQ(math::Color(0, 0, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Specular());
      }
      else if (visualCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.32, 0.32, 0.32, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("sphere_visual", _name->Data());

        EXPECT_EQ(sphLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().SphereShape());
        EXPECT_DOUBLE_EQ(1.2, _geometry->Data().SphereShape()->Radius());

        EXPECT_EQ(math::Color(0, 0, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Specular());
      }
      return true;
    });

  EXPECT_EQ(3u, visualCount);

  // Check lights
  unsigned int lightCount{0};
  runner.EntityCompMgr().Each<components::Light,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const components::Light *_light,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _light);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      lightCount++;

      EXPECT_EQ(ignition::math::Pose3d(0.0, 0.0, 10, 0, 0, 0),
          _pose->Data());

      EXPECT_EQ("sun", _name->Data());

      EXPECT_EQ(worldEntity, _parent->Data());

      EXPECT_EQ("sun", _light->Data().Name());
      EXPECT_EQ(sdf::LightType::DIRECTIONAL, _light->Data().Type());
      EXPECT_EQ(ignition::math::Pose3d(0, 0, 10, 0, 0, 0),
          _light->Data().RawPose());
      EXPECT_EQ("", _light->Data().PoseRelativeTo());
      EXPECT_TRUE(_light->Data().CastShadows());
      EXPECT_EQ(ignition::math::Color(0.8f, 0.8f, 0.8f, 1),
          _light->Data().Diffuse());
      EXPECT_EQ(ignition::math::Color(0.2f, 0.2f, 0.2f, 1),
          _light->Data().Specular());
      EXPECT_DOUBLE_EQ(1000, _light->Data().AttenuationRange());
      EXPECT_DOUBLE_EQ(0.9, _light->Data().ConstantAttenuationFactor());
      EXPECT_DOUBLE_EQ(0.01, _light->Data().LinearAttenuationFactor());
      EXPECT_DOUBLE_EQ(0.001, _light->Data().QuadraticAttenuationFactor());
      EXPECT_EQ(ignition::math::Vector3d(-0.5, 0.1, -0.9),
          _light->Data().Direction());
      return true;
    });

  EXPECT_EQ(1u, lightCount);
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, CreateLights)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/lights.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(root.WorldByIndex(0), systemLoader);

  // Check entities
  // 1 x world + 1 x (default) level + 1 x wind + 1 x model + 1 x link + 1 x
  // visual + 4 x light
  EXPECT_EQ(10u, runner.EntityCompMgr().EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  Entity worldEntity = kNullEntity;
  runner.EntityCompMgr().Each<components::World,
                            components::Name>(
    [&](const Entity &_entity,
        const components::World *_world,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _world);
      EXPECT_NE(nullptr, _name);

      EXPECT_EQ("lights", _name->Data());

      worldCount++;

      worldEntity = _entity;
      return true;
    });

  EXPECT_EQ(1u, worldCount);
  EXPECT_NE(kNullEntity, worldEntity);

  // Check model
  unsigned int modelCount{0};
  Entity sphModelEntity = kNullEntity;
  runner.EntityCompMgr().Each<components::Model,
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
      EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0),
          _pose->Data());
      EXPECT_EQ("sphere", _name->Data());
      sphModelEntity = _entity;

      return true;
    });

  EXPECT_EQ(1u, modelCount);
  EXPECT_NE(kNullEntity, sphModelEntity);

  // Check link
  unsigned int linkCount{0};
  Entity sphLinkEntity = kNullEntity;
  runner.EntityCompMgr().Each<components::Link,
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

      EXPECT_EQ(ignition::math::Pose3d(0.0, 0.0, 0.0, 0, 0, 0),
          _pose->Data());
      EXPECT_EQ("sphere_link", _name->Data());
      EXPECT_EQ(sphModelEntity, _parent->Data());
      sphLinkEntity = _entity;

      return true;
    });

  EXPECT_EQ(1u, linkCount);
  EXPECT_NE(kNullEntity, sphLinkEntity);

  // Check visuals
  unsigned int visualCount{0};
  runner.EntityCompMgr().Each<components::Visual,
                            components::Geometry,
                            components::Material,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const components::Visual *_visual,
        const components::Geometry *_geometry,
        const components::Material *_material,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _visual);
      EXPECT_NE(nullptr, _geometry);
      EXPECT_NE(nullptr, _material);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      visualCount++;

      EXPECT_EQ(ignition::math::Pose3d(0.0, 0.0, 0.0, 0, 0, 0),
          _pose->Data());

      EXPECT_EQ("sphere_visual", _name->Data());

      EXPECT_EQ(sphLinkEntity, _parent->Data());

      EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
      EXPECT_NE(nullptr, _geometry->Data().SphereShape());
      EXPECT_DOUBLE_EQ(0.5, _geometry->Data().SphereShape()->Radius());

      EXPECT_EQ(math::Color(0.3, 0.3, 0.3), _material->Data().Ambient());
      EXPECT_EQ(math::Color(0.3, 0.3, 0.3), _material->Data().Diffuse());
      EXPECT_EQ(math::Color(0.3, 0.3, 0.3), _material->Data().Specular());
      return true;
    });

  EXPECT_EQ(1u, visualCount);

  // Check lights
  unsigned int lightCount{0};
  runner.EntityCompMgr().Each<components::Light,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const components::Light *_light,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _light);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      lightCount++;

      // light attached to link
      if (lightCount == 1u)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.0, 0.0, 1.0, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("link_light_point", _name->Data());
        EXPECT_EQ(sphLinkEntity, _parent->Data());
        EXPECT_EQ("link_light_point", _light->Data().Name());
        EXPECT_EQ(sdf::LightType::POINT, _light->Data().Type());
        EXPECT_EQ(ignition::math::Pose3d(0, 0, 1, 0, 0, 0),
            _light->Data().RawPose());
        EXPECT_EQ(std::string(), _light->Data().PoseRelativeTo());
        EXPECT_FALSE(_light->Data().CastShadows());
        EXPECT_EQ(ignition::math::Color(0.0f, 0.0f, 1.0f, 1),
            _light->Data().Diffuse());
        EXPECT_EQ(ignition::math::Color(0.1f, 0.1f, 0.1f, 1),
            _light->Data().Specular());
        EXPECT_DOUBLE_EQ(2, _light->Data().AttenuationRange());
        EXPECT_DOUBLE_EQ(0.05, _light->Data().ConstantAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.02, _light->Data().LinearAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.01, _light->Data().QuadraticAttenuationFactor());
      }
      // directional light in the world
      else if (lightCount == 2u)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.0, 0.0, 10, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("directional", _name->Data());
        EXPECT_EQ(worldEntity, _parent->Data());
        EXPECT_EQ("directional", _light->Data().Name());
        EXPECT_EQ(sdf::LightType::DIRECTIONAL, _light->Data().Type());
        EXPECT_EQ(ignition::math::Pose3d(0, 0, 10, 0, 0, 0),
            _light->Data().RawPose());
        EXPECT_EQ(std::string(), _light->Data().PoseRelativeTo());
        EXPECT_TRUE(_light->Data().CastShadows());
        EXPECT_EQ(ignition::math::Color(0.8f, 0.8f, 0.8f, 1),
            _light->Data().Diffuse());
        EXPECT_EQ(ignition::math::Color(0.2f, 0.2f, 0.2f, 1),
            _light->Data().Specular());
        EXPECT_DOUBLE_EQ(100, _light->Data().AttenuationRange());
        EXPECT_DOUBLE_EQ(0.9, _light->Data().ConstantAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.01, _light->Data().LinearAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.001, _light->Data().QuadraticAttenuationFactor());
        EXPECT_EQ(ignition::math::Vector3d(0.5, 0.2, -0.9),
            _light->Data().Direction());
      }
      // point light in the world
      else if (lightCount == 3u)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.0, -1.5, 3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("point", _name->Data());
        EXPECT_EQ(worldEntity, _parent->Data());
        EXPECT_EQ("point", _light->Data().Name());
        EXPECT_EQ(sdf::LightType::POINT, _light->Data().Type());
        EXPECT_EQ(ignition::math::Pose3d(0, -1.5, 3, 0, 0, 0),
            _light->Data().RawPose());
        EXPECT_EQ(std::string(), _light->Data().PoseRelativeTo());
        EXPECT_FALSE(_light->Data().CastShadows());
        EXPECT_EQ(ignition::math::Color(1.0f, 0.0f, 0.0f, 1),
            _light->Data().Diffuse());
        EXPECT_EQ(ignition::math::Color(0.1f, 0.1f, 0.1f, 1),
            _light->Data().Specular());
        EXPECT_DOUBLE_EQ(4, _light->Data().AttenuationRange());
        EXPECT_DOUBLE_EQ(0.2, _light->Data().ConstantAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.5, _light->Data().LinearAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.01, _light->Data().QuadraticAttenuationFactor());
      }
      // spot light in the world
      else if (lightCount == 4u)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.0, 1.5, 3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("spot", _name->Data());
        EXPECT_EQ(worldEntity, _parent->Data());
        EXPECT_EQ("spot", _light->Data().Name());
        EXPECT_EQ(sdf::LightType::SPOT, _light->Data().Type());
        EXPECT_EQ(ignition::math::Pose3d(0, 1.5, 3, 0, 0, 0),
            _light->Data().RawPose());
        EXPECT_EQ(std::string(), _light->Data().PoseRelativeTo());
        EXPECT_FALSE(_light->Data().CastShadows());
        EXPECT_EQ(ignition::math::Color(0.0f, 1.0f, 0.0f, 1),
            _light->Data().Diffuse());
        EXPECT_EQ(ignition::math::Color(0.2f, 0.2f, 0.2f, 1),
            _light->Data().Specular());
        EXPECT_DOUBLE_EQ(5, _light->Data().AttenuationRange());
        EXPECT_DOUBLE_EQ(0.3, _light->Data().ConstantAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.4, _light->Data().LinearAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.001, _light->Data().QuadraticAttenuationFactor());
        EXPECT_EQ(ignition::math::Vector3d(0.0, 0.0, -1.0),
            _light->Data().Direction());
        EXPECT_DOUBLE_EQ(0.1, _light->Data().SpotInnerAngle().Radian());
        EXPECT_DOUBLE_EQ(0.5, _light->Data().SpotOuterAngle().Radian());
        EXPECT_DOUBLE_EQ(0.8, _light->Data().SpotFalloff());
      }
      return true;
    });

  EXPECT_EQ(4u, lightCount);
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, CreateJointEntities)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/demo_joint_types.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(root.WorldByIndex(0), systemLoader);

  // Check component types
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::World::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::CanonicalLink::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Link::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Joint::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::JointAxis::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::JointType::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::ChildLinkName::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::ParentLinkName::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::ParentEntity::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Pose::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Name::typeId));

  const sdf::Model *model = root.WorldByIndex(0)->ModelByIndex(1);

  // Check canonical links
  unsigned int canonicalLinkCount{0};
  runner.EntityCompMgr().Each<components::CanonicalLink>(
    [&](const Entity &, const components::CanonicalLink *)->bool
    {
      canonicalLinkCount++;
      return true;
    });
  // one canonical link per model
  EXPECT_EQ(root.WorldByIndex(0)->ModelCount(), canonicalLinkCount);

  auto testAxis = [](const std::string &_jointName,
                      const sdf::JointAxis *_jointAxis,
                      const auto *_axisComp)
  {
    ASSERT_TRUE(nullptr != _axisComp) << "Axis not found for " << _jointName;
    EXPECT_EQ(_jointAxis->Xyz(), _axisComp->Data().Xyz());
    EXPECT_DOUBLE_EQ(_jointAxis->Lower(), _axisComp->Data().Lower());
    EXPECT_DOUBLE_EQ(_jointAxis->Upper(), _axisComp->Data().Upper());
    EXPECT_DOUBLE_EQ(_jointAxis->Effort(), _axisComp->Data().Effort());
  };

  auto testJoint = [&testAxis](const sdf::Joint *_joint,
      const components::JointAxis *_axis,
      const components::JointAxis2 *_axis2,
      const components::ParentLinkName *_parentLinkName,
      const components::ChildLinkName *_childLinkName,
      const components::Pose *_pose,
      const components::Name *_name,
      bool _checkAxis = true,
      bool _checkAxis2 = false)
  {
    ASSERT_TRUE(nullptr != _joint);

    // Even if the pose element isn't explicitly set in the sdf, a default one
    // is created during parsing, so it's safe to compare here.
    EXPECT_EQ(_joint->RawPose(), _pose->Data());

    if (_checkAxis)
      testAxis(_joint->Name(), _joint->Axis(0), _axis);

    if (_checkAxis2)
      testAxis(_joint->Name(), _joint->Axis(1), _axis2);

    EXPECT_EQ(_joint->ParentLinkName(), _parentLinkName->Data());
    EXPECT_EQ(_joint->ChildLinkName(), _childLinkName->Data());
    EXPECT_EQ(_joint->Name(), _name->Data());
  };

  std::set<std::string> jointsToCheck {
    "revolute_demo",
    "gearbox_demo",
    "revolute2_demo",
    "prismatic_demo",
    "ball_demo",
    "screw_demo",
    "universal_demo",
    "prismatic_demo",
    "fixed_demo"
  };

  std::set<sdf::JointType> jointTypes;
  runner.EntityCompMgr().Each<components::Joint,
                            components::JointType,
                            components::ParentLinkName,
                            components::ChildLinkName,
                            components::Pose,
                            components::Name>(
    [&](const Entity &_entity,
        const components::Joint * /*_joint*/,
        const components::JointType *_jointType,
        const components::ParentLinkName *_parentLinkName,
        const components::ChildLinkName *_childLinkName,
        const components::Pose *_pose,
        const components::Name *_name)->bool
    {
      jointTypes.insert(_jointType->Data());
      auto axis =
          runner.EntityCompMgr().Component<components::JointAxis>(_entity);
      auto axis2 =
          runner.EntityCompMgr().Component<components::JointAxis2>(_entity);

      const sdf::Joint *joint = model->JointByName(_name->Data());

      if (jointsToCheck.find(_name->Data()) != jointsToCheck.end())
      {
        bool checkAxis = ("fixed_demo" != _name->Data()) &&
                         ("ball_demo" != _name->Data());
        bool checkAxis2 = ("gearbox_demo" == _name->Data()) ||
                          ("revolute2_demo" == _name->Data()) ||
                          ("universal_demo" == _name->Data());
        testJoint(joint, axis, axis2, _parentLinkName, _childLinkName, _pose,
            _name, checkAxis, checkAxis2);
      }

      return true;
    });

  EXPECT_EQ(8u, jointTypes.size());
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, Time)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  transport::Node node;
  node.Subscribe("/world/default/clock", &clockCb);
  node.Subscribe("/clock", &rootClockCb);

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(root.WorldByIndex(0), systemLoader);

  // Check state
  EXPECT_TRUE(runner.Paused());
  EXPECT_EQ(0u, runner.CurrentInfo().iterations);
  EXPECT_EQ(0ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(0ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(1ms, runner.StepSize());

  runner.SetPaused(false);

  // Run
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(100u, runner.CurrentInfo().iterations);
  EXPECT_EQ(100ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(1ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(1ms, runner.StepSize());

  int sleep = 0;
  while (clockMsgs.size() < 100 && sleep++ < 100)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Verify info published to /clock topic
  auto simTime = math::durationToSecNsec(runner.CurrentInfo().simTime);
  EXPECT_EQ(clockMsgs.back().mutable_sim()->sec(), simTime.first);
  EXPECT_EQ(clockMsgs.back().mutable_sim()->nsec(), simTime.second);

  // Change step size and run
  runner.SetStepSize(2ms);
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(200u, runner.CurrentInfo().iterations);
  EXPECT_EQ(300ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(2ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(2ms, runner.StepSize());

  sleep = 0;
  while (clockMsgs.size() < 200 && sleep++ < 100)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Verify info published to /clock topic
  simTime = math::durationToSecNsec(runner.CurrentInfo().simTime);
  EXPECT_EQ(clockMsgs.back().mutable_sim()->sec(), simTime.first);
  EXPECT_EQ(clockMsgs.back().mutable_sim()->nsec(), simTime.second);

  // Set paused
  runner.SetPaused(true);
  EXPECT_TRUE(runner.Paused());
  runner.SetPaused(false);
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(200u, runner.CurrentInfo().iterations);
  EXPECT_EQ(300ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(2ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(2ms, runner.StepSize());

  // Verify info published to /clock topic
  simTime = math::durationToSecNsec(runner.CurrentInfo().simTime);
  EXPECT_EQ(clockMsgs.back().mutable_sim()->sec(), simTime.first);
  EXPECT_EQ(clockMsgs.back().mutable_sim()->nsec(), simTime.second);

  // Unpause and run
  runner.SetPaused(false);
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(300u, runner.CurrentInfo().iterations);
  EXPECT_EQ(500ms, runner.CurrentInfo().simTime)
    << runner.CurrentInfo().simTime.count();
  EXPECT_EQ(2ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(2ms, runner.StepSize());

  sleep = 0;
  while (clockMsgs.size() < 300 && sleep++ < 100)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Verify info published to /clock topic
  simTime = math::durationToSecNsec(runner.CurrentInfo().simTime);
  EXPECT_EQ(clockMsgs.back().mutable_sim()->sec(), simTime.first);
  EXPECT_EQ(clockMsgs.back().mutable_sim()->nsec(), simTime.second);

  sleep = 0;
  while (clockMsgs.size() != rootClockMsgs.size() && sleep++ < 100)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // verify root clock
  EXPECT_EQ(clockMsgs.size(), rootClockMsgs.size());
  for (unsigned int i = 0; i < clockMsgs.size(); ++i)
  {
    EXPECT_EQ(clockMsgs[i].mutable_sim()->sec(),
        rootClockMsgs[i].mutable_sim()->sec());
    EXPECT_EQ(clockMsgs[i].mutable_sim()->nsec(),
        rootClockMsgs[i].mutable_sim()->nsec());
  }
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, LoadPlugins)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/plugins.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(root.WorldByIndex(0), systemLoader);

  // Get world entity
  Entity worldId{kNullEntity};
  runner.EntityCompMgr().Each<ignition::gazebo::components::World>([&](
      const ignition::gazebo::Entity &_entity,
      const ignition::gazebo::components::World *_world)->bool
      {
        EXPECT_NE(nullptr, _world);
        worldId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, worldId);

  // Get model entity
  Entity modelId{kNullEntity};
  runner.EntityCompMgr().Each<ignition::gazebo::components::Model>([&](
      const ignition::gazebo::Entity &_entity,
      const ignition::gazebo::components::Model *_model)->bool
      {
        EXPECT_NE(nullptr, _model);
        modelId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, modelId);

  // Get sensor entity
  Entity sensorId{kNullEntity};
  runner.EntityCompMgr().Each<ignition::gazebo::components::Sensor>([&](
      const ignition::gazebo::Entity &_entity,
      const ignition::gazebo::components::Sensor *_sensor)->bool
      {
        EXPECT_NE(nullptr, _sensor);
        sensorId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, sensorId);

  // Check component registered by world plugin
  std::string worldComponentName{"WorldPluginComponent"};
  auto worldComponentId = ignition::common::hash64(worldComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(worldComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(worldId,
      worldComponentId));

  // Check component registered by model plugin
  std::string modelComponentName{"ModelPluginComponent"};
  auto modelComponentId = ignition::common::hash64(modelComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(modelComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(modelId,
      modelComponentId));

  // Check component registered by sensor plugin
  std::string sensorComponentName{"SensorPluginComponent"};
  auto sensorComponentId = ignition::common::hash64(sensorComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(sensorComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(sensorId,
      sensorComponentId));

  // Clang re-registers components between tests. If we don't unregister them
  // beforehand, the new plugin tries to create a storage type from a previous
  // plugin, causing a crash.
  // Is this only a problem with GTest, or also during simulation? How to
  // reproduce? Maybe we need to test unloading plugins, but we have no API for
  // it yet.
  #if defined (__clang__)
    components::Factory::Instance()->Unregister(worldComponentId);
    components::Factory::Instance()->Unregister(modelComponentId);
    components::Factory::Instance()->Unregister(sensorComponentId);
  #endif
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, LoadServerConfigPlugins)
{
  sdf::Root rootWithout;
  rootWithout.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/plugins_empty.sdf");
  ASSERT_EQ(1u, rootWithout.WorldCount());

  std::string plugins = R"(
  <root>
    <plugin
      entity_name="default"
      entity_type="world"
      filename="libTestWorldSystem.so"
      name="ignition::gazebo::TestWorldSystem">
      <world_key>0.123</world_key>
    </plugin>
    <plugin
      entity_name="box"
      entity_type="model"
      filename="libTestModelSystem.so"
      name="ignition::gazebo::TestModelSystem">
      <model_key>987</model_key>
    </plugin>
    <plugin
      entity_name="default::box::link_1::camera"
      entity_type="sensor"
      filename="libTestSensorSystem.so"
      name="ignition::gazebo::TestSensorSystem">
      <sensor_key>456</sensor_key>
    </plugin>
  </root>)";

  // Create a server configuration with plugins
  ServerConfig serverConfig;
  for (auto plugin : ParsePluginsFromString(plugins))
  {
    serverConfig.AddPlugin(plugin);
  }

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(rootWithout.WorldByIndex(0), systemLoader,
      serverConfig);

  // Get world entity
  Entity worldId{kNullEntity};
  runner.EntityCompMgr().Each<ignition::gazebo::components::World>([&](
      const ignition::gazebo::Entity &_entity,
      const ignition::gazebo::components::World *_world)->bool
      {
        EXPECT_NE(nullptr, _world);
        worldId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, worldId);

  // Get model entity
  Entity modelId{kNullEntity};
  runner.EntityCompMgr().Each<ignition::gazebo::components::Model>([&](
      const ignition::gazebo::Entity &_entity,
      const ignition::gazebo::components::Model *_model)->bool
      {
        EXPECT_NE(nullptr, _model);
        modelId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, modelId);

  // Get sensor entity
  Entity sensorId{kNullEntity};
  runner.EntityCompMgr().Each<ignition::gazebo::components::Sensor>([&](
      const ignition::gazebo::Entity &_entity,
      const ignition::gazebo::components::Sensor *_sensor)->bool
      {
        EXPECT_NE(nullptr, _sensor);
        sensorId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, sensorId);

  // Check component registered by world plugin
  std::string worldComponentName{"WorldPluginComponent"};
  auto worldComponentId = ignition::common::hash64(worldComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(worldComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(worldId,
      worldComponentId));

  // Check component registered by model plugin
  std::string modelComponentName{"ModelPluginComponent"};
  auto modelComponentId = ignition::common::hash64(modelComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(modelComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(modelId,
      modelComponentId));

  // Check component registered by sensor plugin
  std::string sensorComponentName{"SensorPluginComponent"};
  auto sensorComponentId = ignition::common::hash64(sensorComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(sensorComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(sensorId,
      sensorComponentId));

  // Clang re-registers components between tests. If we don't unregister them
  // beforehand, the new plugin tries to create a storage type from a previous
  // plugin, causing a crash.
  // Is this only a problem with GTest, or also during simulation? How to
  // reproduce? Maybe we need to test unloading plugins, but we have no API for
  // it yet.
  #if defined (__clang__)
    components::Factory::Instance()->Unregister(worldComponentId);
    components::Factory::Instance()->Unregister(modelComponentId);
    components::Factory::Instance()->Unregister(sensorComponentId);
  #endif
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, LoadPluginsDefault)
{
  sdf::Root rootWithout;
  rootWithout.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/plugins_empty.sdf");
  ASSERT_EQ(1u, rootWithout.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(rootWithout.WorldByIndex(0), systemLoader);
  ASSERT_EQ(3u, runner.SystemCount());
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, LoadPluginsEvent)
{
  // Load SDF file without plugins
  sdf::Root rootWithout;
  rootWithout.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");
  ASSERT_EQ(1u, rootWithout.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(rootWithout.WorldByIndex(0), systemLoader);
  runner.SetPaused(false);

  // Get model entities
  auto boxEntity = runner.EntityCompMgr().EntityByComponents(
      ignition::gazebo::components::Model(),
      ignition::gazebo::components::Name("box"));
  EXPECT_NE(kNullEntity, boxEntity);

  auto sphereEntity = runner.EntityCompMgr().EntityByComponents(
      ignition::gazebo::components::Model(),
      ignition::gazebo::components::Name("sphere"));
  EXPECT_NE(kNullEntity, sphereEntity);

  auto cylinderEntity = runner.EntityCompMgr().EntityByComponents(
      ignition::gazebo::components::Model(),
      ignition::gazebo::components::Name("cylinder"));
  EXPECT_NE(kNullEntity, cylinderEntity);

  // We can't access the type registered by the plugin unless we link against
  // it, but we know its name to check
  std::string componentName{"ModelPluginComponent"};
  auto componentId = ignition::common::hash64(componentName);

  // Check there's no double component
  EXPECT_FALSE(runner.EntityCompMgr().HasComponentType(componentId));

  // Load SDF file with plugins
  sdf::Root rootWith;
  rootWith.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/plugins.sdf");
  ASSERT_EQ(1u, rootWith.WorldCount());

  // Emit plugin loading event
  runner.EventMgr().Emit<events::LoadPlugins>(boxEntity,
      rootWith.WorldByIndex(0)->ModelByIndex(0)->Element());

  // Check component registered by model plugin
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(componentId))
      << componentId;
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(boxEntity,
      componentId)) << componentId;

  // Emit plugin loading event again
  runner.EventMgr().Emit<events::LoadPlugins>(sphereEntity,
      rootWith.WorldByIndex(0)->ModelByIndex(0)->Element());

  // Check component for the other model
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(componentId))
      << componentId;
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(sphereEntity,
      componentId)) << componentId;

  // Remove entities that have plugin - this is not unloading or destroying
  // the plugin though!
  auto entityCount = runner.EntityCompMgr().EntityCount();
  const_cast<EntityComponentManager &>(
      runner.EntityCompMgr()).RequestRemoveEntity(boxEntity);
  const_cast<EntityComponentManager &>(
      runner.EntityCompMgr()).RequestRemoveEntity(sphereEntity);
  EXPECT_TRUE(runner.Run(100));
  EXPECT_GT(entityCount, runner.EntityCompMgr().EntityCount());

  // Check component is still registered
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(componentId))
      << componentId;

  // Entities no longer exist
  EXPECT_FALSE(runner.EntityCompMgr().HasEntity(boxEntity));
  EXPECT_FALSE(runner.EntityCompMgr().HasEntity(sphereEntity));

  // Emit plugin loading event after all previous instances have been removed
  runner.EventMgr().Emit<events::LoadPlugins>(cylinderEntity,
      rootWith.WorldByIndex(0)->ModelByIndex(0)->Element());

  // Check component for the other model
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(componentId))
      << componentId;
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(cylinderEntity,
      componentId)) << componentId;
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, GuiInfo)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(root.WorldByIndex(0), systemLoader);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  msgs::GUI res;

  EXPECT_TRUE(node.Request("/world/default/gui/info", timeout, res, result));
  EXPECT_TRUE(result);

  ASSERT_EQ(1, res.plugin_size());

  auto plugin = res.plugin(0);
  EXPECT_EQ("3D View", plugin.name());
  EXPECT_EQ("GzScene3D", plugin.filename());
  EXPECT_NE(plugin.innerxml().find("<ignition-gui>"), std::string::npos);
  EXPECT_NE(plugin.innerxml().find("<ambient_light>"), std::string::npos);
  EXPECT_EQ(plugin.innerxml().find("<service>"), std::string::npos);
  EXPECT_EQ(plugin.innerxml().find("<pose_topic>"), std::string::npos);
  EXPECT_EQ(plugin.innerxml().find("<scene_topic>"), std::string::npos);
  EXPECT_EQ(plugin.innerxml().find("<deletion_topic>"), std::string::npos);
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, GenerateWorldSdf)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(root.WorldByIndex(0), systemLoader);

  msgs::SdfGeneratorConfig req;
  msgs::StringMsg genWorldSdf;
  EXPECT_TRUE(runner.GenerateWorldSdf(req, genWorldSdf));
  EXPECT_FALSE(genWorldSdf.data().empty());

  sdf::Root newRoot;
  newRoot.LoadSdfString(genWorldSdf.data());
  ASSERT_EQ(1u, newRoot.WorldCount());

  const auto* world = newRoot.WorldByIndex(0);
  EXPECT_EQ(3u, world->ModelCount());
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(ServerRepeat, SimulationRunnerTest,
    ::testing::Range(1, 2));

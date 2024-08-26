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

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/gui.pb.h>
#include <gz/msgs/sdf_generator_config.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>

#include "test_config.hh"

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Material.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/config.hh"

#include "../test/helpers/EnvTestFixture.hh"
#include "SimulationRunner.hh"

using namespace gz;
using namespace sim;
using namespace components;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
using IntComponent = Component<int, class IntComponentTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.IntComponent",
    IntComponent)

using DoubleComponent = Component<double, class DoubleComponentTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.DoubleComponent",
    DoubleComponent)
}
}
}
}

/////////////////////////////////////////////////
class SimulationRunnerTest
  : public InternalFixture<::testing::TestWithParam<int>>
{
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
  root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*root.WorldByIndex(0), systemLoader);

  // Check component types
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      World::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Model::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      CanonicalLink::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Link::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Collision::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Visual::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Light::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Name::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      ParentEntity::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Geometry::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      components::Material::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Inertial::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Wind::typeId));

  // Check entities
  // 1 x world + 1 x (default) level + 1 x wind + 5 x model + 5 x link + 5 x
  // collision + 5 x visual + 1 x light (light + visual)
  EXPECT_EQ(25u, runner.EntityCompMgr().EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  Entity worldEntity = kNullEntity;
  runner.EntityCompMgr().Each<World,
                            components::Name>(
    [&](const Entity &_entity,
        const World *_world,
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

  // Test step size, real time factor is not testable since it has no public API
  auto stepSize = std::chrono::duration<double>(runner.StepSize()).count();
  EXPECT_DOUBLE_EQ(0.001, stepSize);

  // Check models
  unsigned int modelCount{0};
  Entity boxModelEntity = kNullEntity;
  Entity cylModelEntity = kNullEntity;
  Entity sphModelEntity = kNullEntity;
  Entity capModelEntity = kNullEntity;
  Entity ellipModelEntity = kNullEntity;
  runner.EntityCompMgr().Each<Model,
                            Pose,
                            ParentEntity,
                            components::Name>(
    [&](const Entity &_entity,
        const Model *_model,
        const Pose *_pose,
        const ParentEntity *_parent,
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
        EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("box", _name->Data());
        boxModelEntity = _entity;
      }
      else if (modelCount == 2)
      {
        EXPECT_EQ(math::Pose3d(-1, -2, -3, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("cylinder", _name->Data());
        cylModelEntity = _entity;
      }
      else if (modelCount == 3)
      {
        EXPECT_EQ(math::Pose3d(0, 0, 0, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("sphere", _name->Data());
        sphModelEntity = _entity;
      }
      else if (modelCount == 4)
      {
        EXPECT_EQ(math::Pose3d(-4, -5, -6, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("capsule", _name->Data());
        capModelEntity = _entity;
      }
      else if (modelCount == 5)
      {
        EXPECT_EQ(math::Pose3d(4, 5, 6, 0, 0, 1),
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
  Entity boxLinkEntity = kNullEntity;
  Entity cylLinkEntity = kNullEntity;
  Entity sphLinkEntity = kNullEntity;
  Entity capLinkEntity = kNullEntity;
  Entity ellipLinkEntity = kNullEntity;
  runner.EntityCompMgr().Each<Link,
                            Pose,
                            ParentEntity,
                            components::Name>(
    [&](const Entity &_entity,
        const Link *_link,
        const Pose *_pose,
        const ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _link);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      linkCount++;

      if (linkCount == 1)
      {
        EXPECT_EQ(math::Pose3d(0.1, 0.1, 0.1, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("box_link", _name->Data());
        EXPECT_EQ(boxModelEntity, _parent->Data());
        boxLinkEntity = _entity;
      }
      else if (linkCount == 2)
      {
        EXPECT_EQ(math::Pose3d(0.2, 0.2, 0.2, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("cylinder_link", _name->Data());
        EXPECT_EQ(cylModelEntity, _parent->Data());
        cylLinkEntity = _entity;
      }
      else if (linkCount == 3)
      {
        EXPECT_EQ(math::Pose3d(0.3, 0.3, 0.3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("sphere_link", _name->Data());
        EXPECT_EQ(sphModelEntity, _parent->Data());
        sphLinkEntity = _entity;
      }
      else if (linkCount == 4)
      {
        EXPECT_EQ(math::Pose3d(0.5, 0.5, 0.5, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("capsule_link", _name->Data());
        EXPECT_EQ(capModelEntity, _parent->Data());
        capLinkEntity = _entity;
      }
      else if (linkCount == 5)
      {
        EXPECT_EQ(math::Pose3d(0.8, 0.8, 0.8, 0, 0, 0),
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

  // Check inertials
  unsigned int inertialCount{0};
  runner.EntityCompMgr().Each<Link, Inertial>(
    [&](const Entity & _entity,
        const Link *_link,
        const Inertial *_inertial)->bool
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
      else if (_entity == capLinkEntity)
      {
        EXPECT_EQ(math::MassMatrix3d(2.0, math::Vector3d(2.0, 2.0, 2.0),
                                     math::Vector3d::Zero),
                  _inertial->Data().MassMatrix());
      }
      else if (_entity == ellipLinkEntity)
      {
        EXPECT_EQ(math::MassMatrix3d(3.0, math::Vector3d(3.0, 3.0, 3.0),
                                     math::Vector3d::Zero),
                  _inertial->Data().MassMatrix());
      }
      return true;
    });

  EXPECT_EQ(5u, inertialCount);

  // Check collisions
  unsigned int collisionCount{0};
  runner.EntityCompMgr().Each<Collision,
                            Geometry,
                            Pose,
                            ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const Collision *_collision,
        const Geometry *_geometry,
        const Pose *_pose,
        const ParentEntity *_parent,
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
        EXPECT_EQ(math::Pose3d(0.11, 0.11, 0.11, 0, 0, 0),
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
        EXPECT_EQ(math::Pose3d(0.21, 0.21, 0.21, 0, 0, 0),
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
        EXPECT_EQ(math::Pose3d(0.31, 0.31, 0.31, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("sphere_collision", _name->Data());

        EXPECT_EQ(sphLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().SphereShape());
        EXPECT_DOUBLE_EQ(23.4, _geometry->Data().SphereShape()->Radius());
      }
      else if (collisionCount == 4)
      {
        EXPECT_EQ(math::Pose3d(0.51, 0.51, 0.51, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("capsule_collision", _name->Data());

        EXPECT_EQ(capLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::CAPSULE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().CapsuleShape());
        EXPECT_DOUBLE_EQ(0.23, _geometry->Data().CapsuleShape()->Radius());
        EXPECT_DOUBLE_EQ(0.14, _geometry->Data().CapsuleShape()->Length());
      }
      else if (collisionCount == 5)
      {
        EXPECT_EQ(math::Pose3d(0.81, 0.81, 0.81, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("ellipsoid_collision", _name->Data());

        EXPECT_EQ(ellipLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::ELLIPSOID, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().EllipsoidShape());
        EXPECT_EQ(math::Vector3d(0.4, 0.6, 1.6),
        _geometry->Data().EllipsoidShape()->Radii());
      }
      return true;
    });

  EXPECT_EQ(5u, collisionCount);

  // Check visuals
  unsigned int visualCount{0};
  runner.EntityCompMgr().Each<Visual,
                            Geometry,
                            components::Material,
                            Pose,
                            ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const Visual *_visual,
        const Geometry *_geometry,
        const components::Material *_material,
        const Pose *_pose,
        const ParentEntity *_parent,
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
        EXPECT_EQ(math::Pose3d(0.12, 0.12, 0.12, 0, 0, 0),
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
        EXPECT_EQ(math::Pose3d(0.22, 0.22, 0.22, 0, 0, 0),
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
        EXPECT_EQ(math::Pose3d(0.32, 0.32, 0.32, 0, 0, 0),
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
      else if (visualCount == 4)
      {
        EXPECT_EQ(math::Pose3d(0.52, 0.52, 0.52, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("capsule_visual", _name->Data());

        EXPECT_EQ(capLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::CAPSULE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().CapsuleShape());
        EXPECT_DOUBLE_EQ(2.12, _geometry->Data().CapsuleShape()->Radius());
        EXPECT_DOUBLE_EQ(1.23, _geometry->Data().CapsuleShape()->Length());

        EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0.0f, 0.0f, 1.0f), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0.0f, 0.0f, 1.0f), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0.0f, 1.0f, 0.0f), _material->Data().Specular());
      }
      else if (visualCount == 5)
      {
        EXPECT_EQ(math::Pose3d(0.82, 0.82, 0.82, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("ellipsoid_visual", _name->Data());

        EXPECT_EQ(ellipLinkEntity, _parent->Data());

        EXPECT_EQ(sdf::GeometryType::ELLIPSOID, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().EllipsoidShape());
        EXPECT_EQ(math::Vector3d(0.4, 0.6, 1.6),
          _geometry->Data().EllipsoidShape()->Radii());

        EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f), _material->Data().Emissive());
        EXPECT_EQ(math::Color(1.0f, 0.0f, 1.0f), _material->Data().Ambient());
        EXPECT_EQ(math::Color(1.0f, 0.0f, 1.0f), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(1.0f, 0.0f, 1.0f), _material->Data().Specular());
      }
      return true;
    });

  EXPECT_EQ(5u, visualCount);

  // Check lights
  unsigned int lightCount{0};
  runner.EntityCompMgr().Each<Light,
                            Pose,
                            ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const Light *_light,
        const Pose *_pose,
        const ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _light);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      lightCount++;

      EXPECT_EQ(math::Pose3d(0.0, 0.0, 10, 0, 0, 0),
          _pose->Data());

      EXPECT_EQ("sun", _name->Data());

      EXPECT_EQ(worldEntity, _parent->Data());

      EXPECT_EQ("sun", _light->Data().Name());
      EXPECT_EQ(sdf::LightType::DIRECTIONAL, _light->Data().Type());
      EXPECT_EQ(math::Pose3d(0, 0, 10, 0, 0, 0),
          _light->Data().RawPose());
      EXPECT_EQ("", _light->Data().PoseRelativeTo());
      EXPECT_TRUE(_light->Data().CastShadows());
      EXPECT_EQ(math::Color(0.8f, 0.8f, 0.8f, 1),
          _light->Data().Diffuse());
      EXPECT_EQ(math::Color(0.2f, 0.2f, 0.2f, 1),
          _light->Data().Specular());
      EXPECT_DOUBLE_EQ(1000, _light->Data().AttenuationRange());
      EXPECT_DOUBLE_EQ(0.9, _light->Data().ConstantAttenuationFactor());
      EXPECT_DOUBLE_EQ(0.01, _light->Data().LinearAttenuationFactor());
      EXPECT_DOUBLE_EQ(0.001, _light->Data().QuadraticAttenuationFactor());
      EXPECT_DOUBLE_EQ(1.0, _light->Data().Intensity());
      EXPECT_EQ(math::Vector3d(-0.5, 0.1, -0.9),
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
  root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "lights.sdf"));

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*root.WorldByIndex(0), systemLoader);

  // Check entities
  // 1 x world + 1 x (default) level + 1 x wind + 1 x model + 1 x link + 1 x
  // visual + 4 x light (light + visual)
  EXPECT_EQ(14u, runner.EntityCompMgr().EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  Entity worldEntity = kNullEntity;
  runner.EntityCompMgr().Each<World,
                            components::Name>(
    [&](const Entity &_entity,
        const World *_world,
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
  runner.EntityCompMgr().Each<Model,
                            Pose,
                            ParentEntity,
                            components::Name>(
    [&](const Entity &_entity,
        const Model *_model,
        const Pose *_pose,
        const ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _model);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      modelCount++;

      EXPECT_EQ(worldEntity, _parent->Data());
      EXPECT_EQ(math::Pose3d(0, 0, 0, 0, 0, 0),
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
  runner.EntityCompMgr().Each<Link,
                            Pose,
                            ParentEntity,
                            components::Name>(
    [&](const Entity &_entity,
        const Link *_link,
        const Pose *_pose,
        const ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _link);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      linkCount++;

      EXPECT_EQ(math::Pose3d(0.0, 0.0, 0.0, 0, 0, 0),
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
  runner.EntityCompMgr().Each<Visual,
                            Geometry,
                            components::Material,
                            Pose,
                            ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const Visual *_visual,
        const Geometry *_geometry,
        const components::Material *_material,
        const Pose *_pose,
        const ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _visual);
      EXPECT_NE(nullptr, _geometry);
      EXPECT_NE(nullptr, _material);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      visualCount++;

      EXPECT_EQ(math::Pose3d(0.0, 0.0, 0.0, 0, 0, 0),
          _pose->Data());

      EXPECT_EQ("sphere_visual", _name->Data());

      EXPECT_EQ(sphLinkEntity, _parent->Data());

      EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
      EXPECT_NE(nullptr, _geometry->Data().SphereShape());
      EXPECT_DOUBLE_EQ(0.5, _geometry->Data().SphereShape()->Radius());

      EXPECT_EQ(math::Color(0.3f, 0.3f, 0.3f), _material->Data().Ambient());
      EXPECT_EQ(math::Color(0.3f, 0.3f, 0.3f), _material->Data().Diffuse());
      EXPECT_EQ(math::Color(0.3f, 0.3f, 0.3f), _material->Data().Specular());
      return true;
    });

  EXPECT_EQ(1u, visualCount);

  // Check lights
  unsigned int lightCount{0};
  runner.EntityCompMgr().Each<Light,
                            Pose,
                            ParentEntity,
                            components::Name>(
    [&](const Entity &/*_entity*/,
        const Light *_light,
        const Pose *_pose,
        const ParentEntity *_parent,
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
        EXPECT_EQ(math::Pose3d(0.0, 0.0, 1.0, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("link_light_point", _name->Data());
        EXPECT_EQ(sphLinkEntity, _parent->Data());
        EXPECT_EQ("link_light_point", _light->Data().Name());
        EXPECT_EQ(sdf::LightType::POINT, _light->Data().Type());
        EXPECT_EQ(math::Pose3d(0, 0, 1, 0, 0, 0),
            _light->Data().RawPose());
        EXPECT_EQ(std::string(), _light->Data().PoseRelativeTo());
        EXPECT_FALSE(_light->Data().CastShadows());
        EXPECT_EQ(math::Color(0.0f, 0.0f, 1.0f, 1),
            _light->Data().Diffuse());
        EXPECT_EQ(math::Color(0.1f, 0.1f, 0.1f, 1),
            _light->Data().Specular());
        EXPECT_DOUBLE_EQ(2, _light->Data().AttenuationRange());
        EXPECT_DOUBLE_EQ(0.05, _light->Data().ConstantAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.02, _light->Data().LinearAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.01, _light->Data().QuadraticAttenuationFactor());
      }
      // directional light in the world
      else if (lightCount == 2u)
      {
        EXPECT_EQ(math::Pose3d(0.0, 0.0, 10, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("directional", _name->Data());
        EXPECT_EQ(worldEntity, _parent->Data());
        EXPECT_EQ("directional", _light->Data().Name());
        EXPECT_EQ(sdf::LightType::DIRECTIONAL, _light->Data().Type());
        EXPECT_EQ(math::Pose3d(0, 0, 10, 0, 0, 0),
            _light->Data().RawPose());
        EXPECT_EQ(std::string(), _light->Data().PoseRelativeTo());
        EXPECT_TRUE(_light->Data().CastShadows());
        EXPECT_EQ(math::Color(0.8f, 0.8f, 0.8f, 1),
            _light->Data().Diffuse());
        EXPECT_EQ(math::Color(0.2f, 0.2f, 0.2f, 1),
            _light->Data().Specular());
        EXPECT_DOUBLE_EQ(100, _light->Data().AttenuationRange());
        EXPECT_DOUBLE_EQ(0.9, _light->Data().ConstantAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.01, _light->Data().LinearAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.001, _light->Data().QuadraticAttenuationFactor());
        EXPECT_EQ(math::Vector3d(0.5, 0.2, -0.9),
            _light->Data().Direction());
      }
      // point light in the world
      else if (lightCount == 3u)
      {
        EXPECT_EQ(math::Pose3d(0.0, -1.5, 3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("point", _name->Data());
        EXPECT_EQ(worldEntity, _parent->Data());
        EXPECT_EQ("point", _light->Data().Name());
        EXPECT_EQ(sdf::LightType::POINT, _light->Data().Type());
        EXPECT_EQ(math::Pose3d(0, -1.5, 3, 0, 0, 0),
            _light->Data().RawPose());
        EXPECT_EQ(std::string(), _light->Data().PoseRelativeTo());
        EXPECT_FALSE(_light->Data().CastShadows());
        EXPECT_EQ(math::Color(1.0f, 0.0f, 0.0f, 1),
            _light->Data().Diffuse());
        EXPECT_EQ(math::Color(0.1f, 0.1f, 0.1f, 1),
            _light->Data().Specular());
        EXPECT_DOUBLE_EQ(4, _light->Data().AttenuationRange());
        EXPECT_DOUBLE_EQ(0.2, _light->Data().ConstantAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.5, _light->Data().LinearAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.01, _light->Data().QuadraticAttenuationFactor());
      }
      // spot light in the world
      else if (lightCount == 4u)
      {
        EXPECT_EQ(math::Pose3d(0.0, 1.5, 3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("spot", _name->Data());
        EXPECT_EQ(worldEntity, _parent->Data());
        EXPECT_EQ("spot", _light->Data().Name());
        EXPECT_EQ(sdf::LightType::SPOT, _light->Data().Type());
        EXPECT_EQ(math::Pose3d(0, 1.5, 3, 0, 0, 0),
            _light->Data().RawPose());
        EXPECT_EQ(std::string(), _light->Data().PoseRelativeTo());
        EXPECT_FALSE(_light->Data().CastShadows());
        EXPECT_EQ(math::Color(0.0f, 1.0f, 0.0f, 1),
            _light->Data().Diffuse());
        EXPECT_EQ(math::Color(0.2f, 0.2f, 0.2f, 1),
            _light->Data().Specular());
        EXPECT_DOUBLE_EQ(5, _light->Data().AttenuationRange());
        EXPECT_DOUBLE_EQ(0.3, _light->Data().ConstantAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.4, _light->Data().LinearAttenuationFactor());
        EXPECT_DOUBLE_EQ(0.001, _light->Data().QuadraticAttenuationFactor());
        EXPECT_EQ(math::Vector3d(0.0, 0.0, -1.0),
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
  root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "demo_joint_types.sdf"));

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*root.WorldByIndex(0), systemLoader);

  // Check component types
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      World::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      CanonicalLink::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Link::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Joint::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      JointAxis::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      JointType::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      ChildLinkName::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      ParentLinkName::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      ParentEntity::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Pose::typeId));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      Name::typeId));

  const sdf::Model *model = root.WorldByIndex(0)->ModelByIndex(1);

  // Check canonical links
  unsigned int canonicalLinkCount{0};
  runner.EntityCompMgr().Each<CanonicalLink>(
    [&](const Entity &, const CanonicalLink *)->bool
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
      const JointAxis *_axis,
      const JointAxis2 *_axis2,
      const ParentLinkName *_parentLinkName,
      const ChildLinkName *_childLinkName,
      const Pose *_pose,
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

    EXPECT_EQ(_joint->ParentName(), _parentLinkName->Data());
    EXPECT_EQ(_joint->ChildName(), _childLinkName->Data());
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
  runner.EntityCompMgr().Each<Joint,
                            JointType,
                            ParentLinkName,
                            ChildLinkName,
                            Pose,
                            components::Name>(
    [&](const Entity &_entity,
        const Joint * /*_joint*/,
        const JointType *_jointType,
        const ParentLinkName *_parentLinkName,
        const ChildLinkName *_childLinkName,
        const Pose *_pose,
        const components::Name *_name)->bool
    {
      jointTypes.insert(_jointType->Data());
      auto axis =
          runner.EntityCompMgr().Component<JointAxis>(_entity);
      auto axis2 =
          runner.EntityCompMgr().Component<JointAxis2>(_entity);

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
  root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  ASSERT_EQ(1u, root.WorldCount());

  transport::Node node;
  node.Subscribe("/world/default/clock", &clockCb);
  node.Subscribe("/clock", &rootClockCb);

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*root.WorldByIndex(0), systemLoader);

  // Check state
  EXPECT_TRUE(runner.Paused());
  EXPECT_EQ(0u, runner.CurrentInfo().iterations);
  EXPECT_EQ(0ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(0ms, runner.CurrentInfo().dt);
  EXPECT_EQ(0ms, runner.UpdatePeriod());
  EXPECT_EQ(1ms, runner.StepSize());

  runner.SetPaused(false);

  // Run
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(100u, runner.CurrentInfo().iterations);
  EXPECT_EQ(100ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(1ms, runner.CurrentInfo().dt);
  EXPECT_EQ(0ms, runner.UpdatePeriod());
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
  EXPECT_EQ(0ms, runner.UpdatePeriod());
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
  EXPECT_EQ(0ms, runner.UpdatePeriod());
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
  EXPECT_EQ(0ms, runner.UpdatePeriod());
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

  // Test the run to simulation time feature.
  runner.SetPaused(true);
  auto currentSimTime = runner.CurrentInfo().simTime;
  runner.SetRunToSimTime(currentSimTime + std::chrono::seconds(4));
  runner.SetPaused(false);
  runner.Run((std::chrono::seconds(4) / runner.CurrentInfo().dt));
  EXPECT_TRUE(runner.Paused());
  EXPECT_EQ((currentSimTime + std::chrono::seconds(4)).count(),
      runner.CurrentInfo().simTime.count());
}

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_P(SimulationRunnerTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LoadPlugins) )
{
  // Load SDF file
  sdf::Root root;
  root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "plugins.sdf"));

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*root.WorldByIndex(0), systemLoader);

  // Get world entity
  Entity worldId{kNullEntity};
  runner.EntityCompMgr().Each<World>([&](
      const Entity &_entity,
      const World *_world)->bool
      {
        EXPECT_NE(nullptr, _world);
        worldId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, worldId);

  // Get model entity
  Entity modelId{kNullEntity};
  runner.EntityCompMgr().Each<Model>([&](
      const Entity &_entity,
      const Model *_model)->bool
      {
        EXPECT_NE(nullptr, _model);
        modelId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, modelId);

  // Get sensor entity
  Entity sensorId{kNullEntity};
  runner.EntityCompMgr().Each<Sensor>([&](
      const Entity &_entity,
      const Sensor *_sensor)->bool
      {
        EXPECT_NE(nullptr, _sensor);
        sensorId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, sensorId);

  // Get visual entity
  Entity visualId{kNullEntity};
  runner.EntityCompMgr().Each<components::Visual>([&](
      const Entity &_entity,
      const components::Visual *_visual)->bool
      {
        EXPECT_NE(nullptr, _visual);
        visualId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, visualId);

  // Check component registered by world plugin
  std::string worldComponentName{"WorldPluginComponent"};
  auto worldComponentId = common::hash64(worldComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(worldComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(worldId,
      worldComponentId));

  // Check component registered by model plugin
  std::string modelComponentName{"ModelPluginComponent"};
  auto modelComponentId = common::hash64(modelComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(modelComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(modelId,
      modelComponentId));

  // Check component registered by sensor plugin
  std::string sensorComponentName{"SensorPluginComponent"};
  auto sensorComponentId = common::hash64(sensorComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(sensorComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(sensorId,
      sensorComponentId));

  // Check component registered by visual plugin
  std::string visualComponentName{"VisualPluginComponent"};
  auto visualComponentId = common::hash64(visualComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(visualComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(visualId,
      visualComponentId));
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LoadServerNoPlugins) )
{
  sdf::Root rootWithout;
  rootWithout.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "plugins_empty.sdf"));
  ASSERT_EQ(1u, rootWithout.WorldCount());

  // ServerConfig will fall back to environment variable
  auto config = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "server_valid2.config");
  ASSERT_EQ(true, common::setenv(kServerConfigPathEnv, config));
  ServerConfig serverConfig;

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*rootWithout.WorldByIndex(0), systemLoader,
      serverConfig);

  ASSERT_EQ(2u, runner.SystemCount());
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LoadServerConfigPlugins) )
{
  sdf::Root rootWithout;
  rootWithout.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "plugins_empty.sdf"));
  ASSERT_EQ(1u, rootWithout.WorldCount());

  // Create a server configuration with plugins
  // No fallback expected
  auto config = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "server_valid.config");

  auto plugins = parsePluginsFromFile(config);
  ASSERT_EQ(3u, plugins.size());

  ServerConfig serverConfig;
  for (auto plugin : plugins)
  {
    serverConfig.AddPlugin(plugin);
  }

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*rootWithout.WorldByIndex(0), systemLoader,
      serverConfig);

  // Get world entity
  Entity worldId{kNullEntity};
  runner.EntityCompMgr().Each<World>([&](
      const Entity &_entity,
      const World *_world)->bool
      {
        EXPECT_NE(nullptr, _world);
        worldId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, worldId);

  // Get model entity
  Entity modelId{kNullEntity};
  runner.EntityCompMgr().Each<Model>([&](
      const Entity &_entity,
      const Model *_model)->bool
      {
        EXPECT_NE(nullptr, _model);
        modelId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, modelId);

  // Get sensor entity
  Entity sensorId{kNullEntity};
  runner.EntityCompMgr().Each<Sensor>([&](
      const Entity &_entity,
      const Sensor *_sensor)->bool
      {
        EXPECT_NE(nullptr, _sensor);
        sensorId = _entity;
        return true;
      });
  EXPECT_NE(kNullEntity, sensorId);

  // Check component registered by world plugin
  std::string worldComponentName{"WorldPluginComponent"};
  auto worldComponentId = common::hash64(worldComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(worldComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(worldId,
      worldComponentId));

  // Check component registered by model plugin
  std::string modelComponentName{"ModelPluginComponent"};
  auto modelComponentId = common::hash64(modelComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(modelComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(modelId,
      modelComponentId));

  // Check component registered by sensor plugin
  std::string sensorComponentName{"SensorPluginComponent"};
  auto sensorComponentId = common::hash64(sensorComponentName);

  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(sensorComponentId));
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(sensorId,
      sensorComponentId));
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LoadPluginsDefault) )
{
  sdf::Root rootWithout;
  rootWithout.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "plugins_empty.sdf"));
  ASSERT_EQ(1u, rootWithout.WorldCount());

  // Load the default config, but not through the default code path.
  // The user may have modified their local config.
  auto config = common::joinPaths(PROJECT_SOURCE_PATH,
    "include", "gz", "sim", "server.config");
  ASSERT_TRUE(common::setenv(kServerConfigPathEnv, config));

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*rootWithout.WorldByIndex(0), systemLoader);
  ASSERT_EQ(3u, runner.SystemCount());
  common::unsetenv(kServerConfigPathEnv);
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LoadPluginsEvent) )
{
  // Load SDF file without plugins
  sdf::Root rootWithout;
  rootWithout.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));
  ASSERT_EQ(1u, rootWithout.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*rootWithout.WorldByIndex(0), systemLoader);
  runner.SetPaused(false);

  // Get model entities
  auto boxEntity = runner.EntityCompMgr().EntityByComponents(
      Model(),
      components::Name("box"));
  EXPECT_NE(kNullEntity, boxEntity);

  auto sphereEntity = runner.EntityCompMgr().EntityByComponents(
      Model(),
      components::Name("sphere"));
  EXPECT_NE(kNullEntity, sphereEntity);

  auto cylinderEntity = runner.EntityCompMgr().EntityByComponents(
      Model(),
      components::Name("cylinder"));
  EXPECT_NE(kNullEntity, cylinderEntity);

  // We can't access the type registered by the plugin unless we link against
  // it, but we know its name to check
  std::string componentName{"ModelPluginComponent"};
  auto componentId = common::hash64(componentName);

  // Check there's no double component
  EXPECT_FALSE(runner.EntityCompMgr().HasComponentType(componentId));

  // Load SDF file with plugins
  sdf::Root rootWith;
  rootWith.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "plugins.sdf"));
  ASSERT_EQ(1u, rootWith.WorldCount());

  // Emit plugin loading event
  runner.EventMgr().Emit<events::LoadSdfPlugins>(boxEntity,
      rootWith.WorldByIndex(0)->ModelByIndex(0)->Plugins());

  // Check component registered by model plugin
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(componentId))
      << componentId;
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(boxEntity,
      componentId)) << componentId;

  // Emit plugin loading event again
  runner.EventMgr().Emit<events::LoadSdfPlugins>(sphereEntity,
      rootWith.WorldByIndex(0)->ModelByIndex(0)->Plugins());

  // Check component for the other model
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(componentId))
      << componentId;
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(sphereEntity,
      componentId)) << componentId;

  // Remove entities that have plugin
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
  runner.EventMgr().Emit<events::LoadSdfPlugins>(cylinderEntity,
      rootWith.WorldByIndex(0)->ModelByIndex(0)->Plugins());

  // Check component for the other model
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(componentId))
      << componentId;
  EXPECT_TRUE(runner.EntityCompMgr().EntityHasComponentType(cylinderEntity,
      componentId)) << componentId;
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LoadOnlyModelPlugin) )
{
  sdf::Root rootWithout;
  rootWithout.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "model_plugin_only.sdf"));
  ASSERT_EQ(1u, rootWithout.WorldCount());

  // ServerConfig will fall back to environment variable
  auto config = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "server_valid2.config");
  ASSERT_EQ(true, common::setenv(kServerConfigPathEnv, config));
  ServerConfig serverConfig;

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*rootWithout.WorldByIndex(0), systemLoader,
      serverConfig);

  // 1 model plugin from SDF and 1 world plugin from config
  // and 1 model plugin from theconfig
  EXPECT_EQ(3u, runner.SystemCount());
  runner.SetPaused(false);
  runner.Run(1);

  // Remove the model. Only 1 world plugin should remain.
  EXPECT_TRUE(runner.RequestRemoveEntity("box"));
  runner.Run(2);
  EXPECT_EQ(1u, runner.SystemCount());
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, GuiInfo)
{
  // Load SDF file
  sdf::Root root;
  root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*root.WorldByIndex(0), systemLoader);

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
  EXPECT_EQ("MinimalScene", plugin.filename());
  EXPECT_NE(plugin.innerxml().find("<gz-gui>"), std::string::npos);
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
  root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*root.WorldByIndex(0), systemLoader);

  msgs::SdfGeneratorConfig req;
  msgs::StringMsg genWorldSdf;
  EXPECT_TRUE(runner.GenerateWorldSdf(req, genWorldSdf));
  EXPECT_FALSE(genWorldSdf.data().empty());

  sdf::Root newRoot;
  newRoot.LoadSdfString(genWorldSdf.data());
  ASSERT_EQ(1u, newRoot.WorldCount());

  const auto* world = newRoot.WorldByIndex(0);
  EXPECT_EQ(5u, world->ModelCount());
}

/////////////////////////////////////////////////
/// Helper function to recursively check for plugins with filename and name
/// attributes set to "__default__"
testing::AssertionResult checkForSpuriousPlugins(sdf::ElementPtr _elem)
{
  auto plugin = _elem->FindElement("plugin");
  if (nullptr != plugin &&
      plugin->Get<std::string>("filename") == "__default__" &&
      plugin->Get<std::string>("name") == "__default__")
  {
    return testing::AssertionFailure() << _elem->ToString("");
  }
  for (auto child = _elem->GetFirstElement(); child;
       child = child->GetNextElement())
  {
    auto result = checkForSpuriousPlugins(child);
    if (!result)
      return result;
  }
  return testing::AssertionSuccess();
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, GeneratedSdfHasNoSpuriousPlugins)
{
  // Load SDF file
  sdf::Root root;
  root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  auto systemLoader = std::make_shared<SystemLoader>();
  SimulationRunner runner(*root.WorldByIndex(0), systemLoader);

  msgs::SdfGeneratorConfig req;
  msgs::StringMsg genWorldSdf;
  EXPECT_TRUE(runner.GenerateWorldSdf(req, genWorldSdf));
  EXPECT_FALSE(genWorldSdf.data().empty());

  sdf::Root newRoot;
  newRoot.LoadSdfString(genWorldSdf.data());
  EXPECT_TRUE(checkForSpuriousPlugins(newRoot.Element()));
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_SUITE_P(ServerRepeat, SimulationRunnerTest,
    ::testing::Range(1, 2));

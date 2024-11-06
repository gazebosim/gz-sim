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
#include <gz/common/Console.hh>
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
#include "gz/sim/components/CastShadows.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/LaserRetro.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Material.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Transparency.hh"
#include "gz/sim/components/Visibility.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "../test/helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
class EntityCompMgrTest : public EntityComponentManager
{
  public: void ProcessEntityRemovals()
  {
    this->ProcessRemoveEntityRequests();
  }
};

/////////////////////////////////////////////////
class SdfEntityCreatorTest : public InternalFixture<::testing::Test>
{
  public: EntityCompMgrTest ecm;
  public: EventManager evm;
};

/////////////////////////////////////////////////
TEST_F(SdfEntityCreatorTest, CreateEntities)
{
  EXPECT_EQ(0u, this->ecm.EntityCount());

  // SdfEntityCreator
  SdfEntityCreator creator(this->ecm, evm);

  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");
  ASSERT_EQ(1u, root.WorldCount());

  // Create entities
  creator.CreateEntities(root.WorldByIndex(0));

  // Check component types
  EXPECT_TRUE(this->ecm.HasComponentType(components::World::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Model::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::CanonicalLink::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Link::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Collision::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Visual::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Light::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Name::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::ParentEntity::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Geometry::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Material::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Inertial::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::LaserRetro::typeId));

  // Check entities
  // 1 x world + 1 wind + 5 x model + 5 x link + 5 x collision + 5 x visual +
  // 1 x light (light + visual)
  EXPECT_EQ(24u, this->ecm.EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  Entity worldEntity = kNullEntity;
  this->ecm.Each<components::World,
           components::Name,
           components::Physics>(
    [&](const Entity &_entity,
        const components::World *_world,
        const components::Name *_name,
        const components::Physics *_physics)->bool
    {
      EXPECT_NE(nullptr, _world);
      EXPECT_NE(nullptr, _name);
      EXPECT_NE(nullptr, _physics);

      EXPECT_EQ("default", _name->Data());
      EXPECT_DOUBLE_EQ(0.001, _physics->Data().MaxStepSize());
      EXPECT_DOUBLE_EQ(0.0, _physics->Data().RealTimeFactor());

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
  Entity capModelEntity = kNullEntity;
  Entity ellipModelEntity = kNullEntity;
  this->ecm.Each<components::Model,
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
      EXPECT_EQ(worldEntity, this->ecm.ParentEntity(_entity));

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
  this->ecm.Each<components::Link,
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
        EXPECT_EQ(math::Pose3d(0.1, 0.1, 0.1, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("box_link", _name->Data());

        EXPECT_EQ(boxModelEntity, _parent->Data());
        EXPECT_EQ(boxModelEntity, this->ecm.ParentEntity(_entity));

        boxLinkEntity = _entity;
      }
      else if (linkCount == 2)
      {
        EXPECT_EQ(math::Pose3d(0.2, 0.2, 0.2, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("cylinder_link", _name->Data());

        EXPECT_EQ(cylModelEntity, _parent->Data());
        EXPECT_EQ(cylModelEntity, this->ecm.ParentEntity(_entity));

        cylLinkEntity = _entity;
      }
      else if (linkCount == 3)
      {
        EXPECT_EQ(math::Pose3d(0.3, 0.3, 0.3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("sphere_link", _name->Data());

        EXPECT_EQ(sphModelEntity, _parent->Data());
        EXPECT_EQ(sphModelEntity, this->ecm.ParentEntity(_entity));

        sphLinkEntity = _entity;
      }
      else if (linkCount == 4)
      {
        EXPECT_EQ(gz::math::Pose3d(0.5, 0.5, 0.5, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("capsule_link", _name->Data());

        EXPECT_EQ(capModelEntity, _parent->Data());
        EXPECT_EQ(capModelEntity, this->ecm.ParentEntity(_entity));

        capLinkEntity = _entity;
      }
      else if (linkCount == 5)
      {
        EXPECT_EQ(gz::math::Pose3d(0.8, 0.8, 0.8, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("ellipsoid_link", _name->Data());

        EXPECT_EQ(ellipModelEntity, _parent->Data());
        EXPECT_EQ(ellipModelEntity, this->ecm.ParentEntity(_entity));

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
  this->ecm.Each<components::Link, components::Inertial>(
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
  this->ecm.Each<components::Collision,
           components::Geometry,
           components::Pose,
           components::ParentEntity,
           components::Name>(
    [&](const Entity &_entity,
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
        EXPECT_EQ(math::Pose3d(0.11, 0.11, 0.11, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("box_collision", _name->Data());

        EXPECT_EQ(boxLinkEntity, _parent->Data());
        EXPECT_EQ(boxLinkEntity, this->ecm.ParentEntity(_entity));

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
        EXPECT_EQ(cylLinkEntity, this->ecm.ParentEntity(_entity));

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
        EXPECT_EQ(sphLinkEntity, this->ecm.ParentEntity(_entity));

        EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().SphereShape());
        EXPECT_DOUBLE_EQ(23.4, _geometry->Data().SphereShape()->Radius());
      }
      else if (collisionCount == 4)
      {
        EXPECT_EQ(gz::math::Pose3d(0.51, 0.51, 0.51, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("capsule_collision", _name->Data());

        EXPECT_EQ(capLinkEntity, _parent->Data());
        EXPECT_EQ(capLinkEntity, this->ecm.ParentEntity(_entity));

        EXPECT_EQ(sdf::GeometryType::CAPSULE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().CapsuleShape());
        EXPECT_DOUBLE_EQ(0.23, _geometry->Data().CapsuleShape()->Radius());
        EXPECT_DOUBLE_EQ(0.14, _geometry->Data().CapsuleShape()->Length());
        }
      else if (collisionCount == 5)
      {
        EXPECT_EQ(gz::math::Pose3d(0.81, 0.81, 0.81, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("ellipsoid_collision", _name->Data());

        EXPECT_EQ(ellipLinkEntity, _parent->Data());
        EXPECT_EQ(ellipLinkEntity, this->ecm.ParentEntity(_entity));

        EXPECT_EQ(sdf::GeometryType::ELLIPSOID, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().EllipsoidShape());
        EXPECT_EQ(gz::math::Vector3d(0.4, 0.6, 1.6),
          _geometry->Data().EllipsoidShape()->Radii());
      }
      return true;
    });

  EXPECT_EQ(5u, collisionCount);

  // Check visuals
  unsigned int visualCount{0};
  this->ecm.Each<components::Visual,
           components::Transparency,
           components::LaserRetro,
           components::CastShadows,
           components::Geometry,
           components::Material,
           components::VisibilityFlags,
           components::Pose,
           components::ParentEntity,
           components::Name>(
    [&](const Entity &_entity,
        const components::Visual *_visual,
        const components::Transparency *_transparency,
        const components::LaserRetro *_laserRetro,
        const components::CastShadows *_castShadows,
        const components::Geometry *_geometry,
        const components::Material *_material,
        const components::VisibilityFlags *_visibilityFlags,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _visual);
      EXPECT_NE(nullptr, _transparency);
      EXPECT_NE(nullptr, _castShadows);
      EXPECT_NE(nullptr, _geometry);
      EXPECT_NE(nullptr, _material);
      EXPECT_NE(nullptr, _visibilityFlags);
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
        EXPECT_EQ(boxLinkEntity, this->ecm.ParentEntity(_entity));

        EXPECT_DOUBLE_EQ(0.0, _transparency->Data());
        EXPECT_DOUBLE_EQ(1150.0, _laserRetro->Data());
        EXPECT_TRUE(_castShadows->Data());

        EXPECT_EQ(sdf::GeometryType::BOX, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().BoxShape());
        EXPECT_EQ(math::Vector3d(1, 2, 3),
                  _geometry->Data().BoxShape()->Size());

        EXPECT_EQ(math::Color(0, 0, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Ambient());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Specular());

        EXPECT_EQ(4294967295u, _visibilityFlags->Data());
      }
      else if (visualCount == 2)
      {
        EXPECT_EQ(math::Pose3d(0.22, 0.22, 0.22, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("cylinder_visual", _name->Data());

        EXPECT_EQ(cylLinkEntity, _parent->Data());
        EXPECT_EQ(cylLinkEntity, this->ecm.ParentEntity(_entity));

        EXPECT_DOUBLE_EQ(0.0, _transparency->Data());
        EXPECT_DOUBLE_EQ(1654.0, _laserRetro->Data());
        EXPECT_TRUE(_castShadows->Data());

        EXPECT_EQ(sdf::GeometryType::CYLINDER, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().CylinderShape());
        EXPECT_DOUBLE_EQ(2.1, _geometry->Data().CylinderShape()->Radius());
        EXPECT_DOUBLE_EQ(10.2, _geometry->Data().CylinderShape()->Length());

        EXPECT_EQ(math::Color(0, 0, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Specular());

        EXPECT_EQ(4294967295u, _visibilityFlags->Data());
      }
      else if (visualCount == 3)
      {
        EXPECT_EQ(math::Pose3d(0.32, 0.32, 0.32, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("sphere_visual", _name->Data());

        EXPECT_EQ(sphLinkEntity, _parent->Data());
        EXPECT_EQ(sphLinkEntity, this->ecm.ParentEntity(_entity));

        EXPECT_DOUBLE_EQ(0.5, _transparency->Data());
        EXPECT_DOUBLE_EQ(50.0, _laserRetro->Data());
        EXPECT_FALSE(_castShadows->Data());

        EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().SphereShape());
        EXPECT_DOUBLE_EQ(1.2, _geometry->Data().SphereShape()->Radius());

        EXPECT_EQ(math::Color(0, 0, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Specular());

        EXPECT_EQ(4294967295u, _visibilityFlags->Data());
      }
      else if (visualCount == 4)
      {
        EXPECT_EQ(gz::math::Pose3d(0.52, 0.52, 0.52, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("capsule_visual", _name->Data());

        EXPECT_EQ(capLinkEntity, _parent->Data());
        EXPECT_EQ(capLinkEntity, this->ecm.ParentEntity(_entity));

        EXPECT_DOUBLE_EQ(0.0, _transparency->Data());
        EXPECT_DOUBLE_EQ(6.54, _laserRetro->Data());
        EXPECT_TRUE(_castShadows->Data());

        EXPECT_EQ(sdf::GeometryType::CAPSULE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().CapsuleShape());
        EXPECT_DOUBLE_EQ(2.12, _geometry->Data().CapsuleShape()->Radius());
        EXPECT_DOUBLE_EQ(1.23, _geometry->Data().CapsuleShape()->Length());

        EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0.0f, 0.0f, 1.0f), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0.0f, 0.0f, 1.0f), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0.0f, 1.0f, 0.0f), _material->Data().Specular());

        EXPECT_EQ(4294967295u, _visibilityFlags->Data());
      }
      else if (visualCount == 5)
      {
        EXPECT_EQ(gz::math::Pose3d(0.82, 0.82, 0.82, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("ellipsoid_visual", _name->Data());

        EXPECT_EQ(ellipLinkEntity, _parent->Data());
        EXPECT_EQ(ellipLinkEntity, this->ecm.ParentEntity(_entity));

        EXPECT_DOUBLE_EQ(0.5, _transparency->Data());
        EXPECT_DOUBLE_EQ(3.21, _laserRetro->Data());
        EXPECT_FALSE(_castShadows->Data());

        EXPECT_EQ(sdf::GeometryType::ELLIPSOID, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().EllipsoidShape());
        EXPECT_EQ(gz::math::Vector3d(0.4, 0.6, 1.6),
          _geometry->Data().EllipsoidShape()->Radii());

        EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f), _material->Data().Emissive());
        EXPECT_EQ(math::Color(1.0f, 0.0f, 1.0f), _material->Data().Ambient());
        EXPECT_EQ(math::Color(1.0f, 0.0f, 1.0f), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(1.0f, 0.0f, 1.0f), _material->Data().Specular());

        EXPECT_EQ(4294967295u, _visibilityFlags->Data());
      }
      return true;
    });

  EXPECT_EQ(5u, visualCount);

  // Check lights
  unsigned int lightCount{0};
  this->ecm.Each<components::Light,
           components::Pose,
           components::ParentEntity,
           components::Name>(
    [&](const Entity &_entity,
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

      EXPECT_EQ(math::Pose3d(0.0, 0.0, 10, 0, 0, 0),
          _pose->Data());

      EXPECT_EQ("sun", _name->Data());

      EXPECT_EQ(worldEntity, _parent->Data());
      EXPECT_EQ(worldEntity, this->ecm.ParentEntity(_entity));

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
      EXPECT_EQ(math::Vector3d(-0.5, 0.1, -0.9),
          _light->Data().Direction());
      return true;
    });

  EXPECT_EQ(1u, lightCount);
}

/////////////////////////////////////////////////
TEST_F(SdfEntityCreatorTest, CreateLights)
{
  EXPECT_EQ(0u, this->ecm.EntityCount());

  // SdfEntityCreator
  SdfEntityCreator creator(this->ecm, evm);

  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/lights.sdf");
  ASSERT_EQ(1u, root.WorldCount());

  // Create entities
  creator.CreateEntities(root.WorldByIndex(0));

  // Check entities
  // 1 x world + 1 wind + 1 x model + 1 x link + 1 x visual +
  // 4 x light (light + visual)
  EXPECT_EQ(13u, this->ecm.EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  Entity worldEntity = kNullEntity;
  this->ecm.Each<components::World,
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
  this->ecm.Each<components::Model,
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
      EXPECT_EQ(worldEntity, this->ecm.ParentEntity(_entity));

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
  this->ecm.Each<components::Link,
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

      EXPECT_EQ(math::Pose3d(0.0, 0.0, 0.0, 0, 0, 0),
          _pose->Data());
      EXPECT_EQ("sphere_link", _name->Data());
      EXPECT_EQ(sphModelEntity, _parent->Data());
      EXPECT_EQ(sphModelEntity, this->ecm.ParentEntity(_entity));

      sphLinkEntity = _entity;

      return true;
    });

  EXPECT_EQ(1u, linkCount);
  EXPECT_NE(kNullEntity, sphLinkEntity);

  // Check visuals
  unsigned int visualCount{0};
  this->ecm.Each<components::Visual,
           components::Geometry,
           components::Material,
           components::Pose,
           components::ParentEntity,
           components::Name>(
    [&](const Entity &_entity,
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

      EXPECT_EQ(math::Pose3d(0.0, 0.0, 0.0, 0, 0, 0),
          _pose->Data());

      EXPECT_EQ("sphere_visual", _name->Data());

      EXPECT_EQ(sphLinkEntity, _parent->Data());
      EXPECT_EQ(sphLinkEntity, this->ecm.ParentEntity(_entity));

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
  this->ecm.Each<components::Light,
           components::Pose,
           components::ParentEntity,
           components::Name>(
    [&](const Entity &_entity,
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
        EXPECT_EQ(math::Pose3d(0.0, 0.0, 1.0, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("link_light_point", _name->Data());

        EXPECT_EQ(sphLinkEntity, _parent->Data());
        EXPECT_EQ(sphLinkEntity, this->ecm.ParentEntity(_entity));

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
        EXPECT_EQ(worldEntity, this->ecm.ParentEntity(_entity));

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
        EXPECT_EQ(worldEntity, this->ecm.ParentEntity(_entity));

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
        EXPECT_EQ(worldEntity, this->ecm.ParentEntity(_entity));

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
TEST_F(SdfEntityCreatorTest, CreateJointEntities)
{
  EXPECT_EQ(0u, this->ecm.EntityCount());

  // SdfEntityCreator
  SdfEntityCreator creator(this->ecm, evm);

  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/demo_joint_types.sdf");
  ASSERT_EQ(1u, root.WorldCount());

  // Create entities
  creator.CreateEntities(root.WorldByIndex(0));

  // Check component types
  EXPECT_TRUE(this->ecm.HasComponentType(components::World::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::CanonicalLink::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Link::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Joint::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::JointAxis::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::JointType::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::ChildLinkName::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::ParentLinkName::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::ParentEntity::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Pose::typeId));
  EXPECT_TRUE(this->ecm.HasComponentType(components::Name::typeId));

  const sdf::Model *model = root.WorldByIndex(0)->ModelByIndex(1);

  // Check canonical links
  unsigned int canonicalLinkCount{0};
  this->ecm.Each<components::CanonicalLink>(
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
  this->ecm.Each<components::Joint,
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
          this->ecm.Component<components::JointAxis>(_entity);
      auto axis2 =
          this->ecm.Component<components::JointAxis2>(_entity);

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
TEST_F(SdfEntityCreatorTest, RemoveEntities)
{
  EXPECT_EQ(0u, this->ecm.EntityCount());

  // SdfEntityCreator
  SdfEntityCreator creator(this->ecm, evm);

  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");
  ASSERT_EQ(1u, root.WorldCount());

  // Create entities
  creator.CreateEntities(root.WorldByIndex(0));

  // Check entities
  // 1 x world + 1 wind + 4 x model + 4 x link + 4 x collision + 4 x visual
  // + 1 x light (light + visual)
  EXPECT_EQ(24u, this->ecm.EntityCount());

  auto world = this->ecm.EntityByComponents(components::World());
  EXPECT_NE(kNullEntity, world);

  auto models = this->ecm.ChildrenByComponents(world, components::Model());
  ASSERT_EQ(5u, models.size());

  for (auto model : models)
  {
    auto links = this->ecm.ChildrenByComponents(model, components::Link());
    ASSERT_EQ(1u, links.size());

    auto collisions = this->ecm.ChildrenByComponents(links.front(),
        components::Collision());
    ASSERT_EQ(1u, collisions.size());

    auto visuals = this->ecm.ChildrenByComponents(links.front(),
        components::Visual());
    ASSERT_EQ(1u, visuals.size());
  }

  // Delete a model recursively
  creator.RequestRemoveEntity(models.front());
  this->ecm.ProcessEntityRemovals();

  EXPECT_EQ(20u, this->ecm.EntityCount());

  models = this->ecm.ChildrenByComponents(world, components::Model());
  ASSERT_EQ(4u, models.size());

  for (auto model : models)
  {
    auto links = this->ecm.ChildrenByComponents(model, components::Link());
    ASSERT_EQ(1u, links.size());

    auto collisions = this->ecm.ChildrenByComponents(links.front(),
        components::Collision());
    ASSERT_EQ(1u, collisions.size());

    auto visuals = this->ecm.ChildrenByComponents(links.front(),
        components::Visual());
    ASSERT_EQ(1u, visuals.size());
  }

  // Delete a model but leave its children
  creator.RequestRemoveEntity(models.front(), false);
  this->ecm.ProcessEntityRemovals();

  EXPECT_EQ(19u, this->ecm.EntityCount());

  // There's only 1 model left
  models = this->ecm.ChildrenByComponents(world, components::Model());
  ASSERT_EQ(3u, models.size());
  EXPECT_EQ(world, this->ecm.ParentEntity(models.front()));

  // There are 2 links, but one is parentless
  unsigned int linkCount{0};
  this->ecm.Each<components::Link>(
    [&](const Entity &_entity,
        const components::Link *_link)->bool
    {
      EXPECT_NE(nullptr, _link);
      auto parent = this->ecm.ParentEntity(_entity);
      if (linkCount == 0)
      {
        EXPECT_EQ(kNullEntity, parent);
      }
      else
      {
        EXPECT_NE(kNullEntity, parent);
      }
      linkCount++;
      return true;
    });
  EXPECT_EQ(4u, linkCount);

  // There are 2 collisions, both with parents
  unsigned int collisionCount{0};
  this->ecm.Each<components::Collision>(
    [&](const Entity &_entity,
        const components::Collision *_collision)->bool
    {
      EXPECT_NE(nullptr, _collision);
      auto parent = this->ecm.ParentEntity(_entity);
      EXPECT_NE(kNullEntity, parent);
      collisionCount++;
      return true;
    });
  EXPECT_EQ(4u, collisionCount);

  // There are 2 visuals, both with parents
  unsigned int visualCount{0};
  this->ecm.Each<components::Visual>(
    [&](const Entity &_entity,
        const components::Visual *_visual)->bool
    {
      EXPECT_NE(nullptr, _visual);
      auto parent = this->ecm.ParentEntity(_entity);
      EXPECT_NE(kNullEntity, parent);
      visualCount++;
      return true;
    });
  EXPECT_EQ(5u, visualCount);
}

template <typename... Ts>
size_t removedCount(EntityCompMgrTest &_manager)
{
  size_t count = 0;
  _manager.EachRemoved<Ts...>(
      [&](const Entity &, const Ts *...) -> bool
      {
        ++count;
        return true;
      });
  return count;
}

/////////////////////////////////////////////////
TEST_F(SdfEntityCreatorTest, EachRemovedRecursiveRemoved)
{
  // SdfEntityCreator
  SdfEntityCreator creator(this->ecm, evm);

  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");
  ASSERT_EQ(1u, root.WorldCount());

  // Create entities
  creator.CreateEntities(root.WorldByIndex(0));

  auto world = this->ecm.EntityByComponents(components::World());
  EXPECT_NE(kNullEntity, world);

  auto models = this->ecm.ChildrenByComponents(world, components::Model());
  ASSERT_EQ(5u, models.size());

  // Removed should be 0 before requesting removure
  EXPECT_EQ(0u, removedCount<components::Model>(ecm));
  EXPECT_EQ(0u, removedCount<components::Link>(ecm));
  EXPECT_EQ(0u, removedCount<components::Collision>(ecm));
  EXPECT_EQ(0u, removedCount<components::Visual>(ecm));

  // Delete a model recursively
  creator.RequestRemoveEntity(models.front());

  // Since the model is deleted recursively, the child links, collisions and
  // visuals should be returned by an EachRemoved call
  EXPECT_EQ(1u, removedCount<components::Model>(ecm));
  EXPECT_EQ(1u, removedCount<components::Link>(ecm));
  EXPECT_EQ(1u, removedCount<components::Collision>(ecm));
  EXPECT_EQ(1u, removedCount<components::Visual>(ecm));

  this->ecm.ProcessEntityRemovals();

  // Removed should be 0 after requesting removure
  EXPECT_EQ(0u, removedCount<components::Model>(ecm));
  EXPECT_EQ(0u, removedCount<components::Link>(ecm));
  EXPECT_EQ(0u, removedCount<components::Collision>(ecm));
  EXPECT_EQ(0u, removedCount<components::Visual>(ecm));
}

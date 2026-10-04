/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "MujocoPhysics.hh"

#include <gz/msgs/contacts.pb.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityReset.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointEffortLimitsCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityLimitsCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityReset.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/WrenchMeasured.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <sdf/Surface.hh>

#include "MujocoComponents.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace mujoco_physics;

namespace
{
inline void SetPose(mjtNum *_pos, mjtNum *_quat, const math::Pose3d &_pose)
{
  _pos[0] = _pose.Pos().X();
  _pos[1] = _pose.Pos().Y();
  _pos[2] = _pose.Pos().Z();
  _quat[0] = _pose.Rot().W();
  _quat[1] = _pose.Rot().X();
  _quat[2] = _pose.Rot().Y();
  _quat[3] = _pose.Rot().Z();
}

inline void SetVector(mjtNum *_arr, const math::Vector3d &_vec)
{
  _arr[0] = _vec.X();
  _arr[1] = _vec.Y();
  _arr[2] = _vec.Z();
}

inline math::Vector3d GetVector(const mjtNum *_arr)
{
  return math::Vector3d(_arr[0], _arr[1], _arr[2]);
}

inline math::Quaterniond GetQuaternion(const mjtNum *_arr)
{
  return math::Quaterniond(_arr[0], _arr[1], _arr[2], _arr[3]);
}

inline math::Pose3d GetPose(const mjtNum *_pos, const mjtNum *_quat)
{
  return math::Pose3d(GetVector(_pos), GetQuaternion(_quat));
}

inline math::Pose3d GetPoseFromMat(const mjtNum *_pos, const mjtNum *_mat)
{
  std::array<mjtNum, 4> quat;
  mju_mat2Quat(quat.data(), _mat);
  return GetPose(_pos, quat.data());
}

inline void AddWrench(mjtNum *_xfrc, const msgs::Wrench &_wrench)
{
  _xfrc[0] += _wrench.force().x();
  _xfrc[1] += _wrench.force().y();
  _xfrc[2] += _wrench.force().z();
  _xfrc[3] += _wrench.torque().x();
  _xfrc[4] += _wrench.torque().y();
  _xfrc[5] += _wrench.torque().z();
}

inline void SetVectorProto(msgs::Vector3d *_msg, const math::Vector3d &_vec)
{
  _msg->set_x(_vec.X());
  _msg->set_y(_vec.Y());
  _msg->set_z(_vec.Z());
}

template <typename T>
inline void SetInertial(mjtNum *_fullinertia, mjtNum *_ipos, mjtNum *_iquat,
                        const T &_inertial)
{
  _fullinertia[0] = _inertial.MassMatrix().Ixx();
  _fullinertia[1] = _inertial.MassMatrix().Iyy();
  _fullinertia[2] = _inertial.MassMatrix().Izz();
  _fullinertia[3] = _inertial.MassMatrix().Ixy();
  _fullinertia[4] = _inertial.MassMatrix().Ixz();
  _fullinertia[5] = _inertial.MassMatrix().Iyz();
  SetPose(_ipos, _iquat, _inertial.Pose());
}

inline math::Vector3d GetContactForce(const mjtNum *_frame,
                                      const mjtNum *_contactForce)
{
  return math::Vector3d(
      _frame[0] * _contactForce[0] + _frame[3] * _contactForce[1] +
          _frame[6] * _contactForce[2],
      _frame[1] * _contactForce[0] + _frame[4] * _contactForce[1] +
          _frame[7] * _contactForce[2],
      _frame[2] * _contactForce[0] + _frame[5] * _contactForce[1] +
          _frame[8] * _contactForce[2]);
}
}  // namespace

MujocoPhysics::MujocoPhysics() = default;

MujocoPhysics::~MujocoPhysics()
{
  if (this->data)
    mj_deleteData(this->data);
  if (this->model)
    mj_deleteModel(this->model);
  if (this->spec)
    mj_deleteSpec(this->spec);
}

void MujocoPhysics::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
  // Initialize mjSpec
  this->spec = mj_makeSpec();

  this->spec->option.timestep = 0.001;
  this->spec->option.integrator = mjtIntegrator::mjINT_IMPLICITFAST;
  // this->spec->option.enableflags |= mjtEnableBit::mjENBL_SLEEP;

  // Extract gravity from ECM
  auto gravityComp = _ecm.Component<components::Gravity>(_entity);
  if (gravityComp)
  {
    SetVector(this->spec->option.gravity, gravityComp->Data());
  }

  // Compile the model initially
  this->model = mj_compile(this->spec, nullptr);
  if (this->model)
  {
    this->data = mj_makeData(this->model);
  }
  else
  {
    gzerr << "Failed to compile MuJoCo model in Configure\n";
  }
}

void MujocoPhysics::CreatePhysicsEntities(EntityComponentManager &_ecm)
{
  if (!_ecm.HasNewEntities())
    return;

  bool specChanged = false;
  std::unordered_map<Entity, mjsBody *> entityToMujocoBody;

  this->CreateModels(_ecm, specChanged, entityToMujocoBody);
  this->CreateCollisions(_ecm, specChanged, entityToMujocoBody);
  this->CreateJoints(_ecm, specChanged, entityToMujocoBody);
  this->CreateSensors(_ecm, specChanged, entityToMujocoBody);

  if (specChanged)
  {
    mj_recompile(this->spec, nullptr, this->model, this->data);
    this->PostRecompileSetup(_ecm);
  }
}

void MujocoPhysics::CreateModels(
    EntityComponentManager &_ecm, bool &_specChanged,
    std::unordered_map<Entity, mjsBody *> &_entityToMujocoBody)
{
  mjsBody *worldBody = mjs_findBody(this->spec, "world");
  if (!worldBody)
  {
    gzerr << "worldBody is null! Try looking for worldbody or default\n";
  }

  std::vector<std::pair<Entity, std::string>> newModels;
  _ecm.EachNew<components::Model, components::Name>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_nameComp) -> bool
      {
        newModels.push_back({_entity, _nameComp->Data()});
        return true;
      });

  for (auto const &pair : newModels)
  {
    Entity modelEntity = pair.first;
    std::string modelName = pair.second;
    bool isStatic = false;
    auto staticComp = _ecm.Component<components::Static>(modelEntity);
    if (staticComp && staticComp->Data())
    {
      isStatic = true;
    }

    std::vector<Entity> modelLinks;
    _ecm.Each<components::Link, components::ParentEntity>(
        [&](const Entity &_linkEntity, const components::Link *,
            const components::ParentEntity *_parent) -> bool
        {
          if (_parent->Data() == modelEntity)
            modelLinks.push_back(_linkEntity);
          return true;
        });

    std::vector<Entity> modelJoints;
    _ecm.Each<components::Joint, components::ParentEntity>(
        [&](const Entity &_jointEntity, const components::Joint *,
            const components::ParentEntity *_parent) -> bool
        {
          if (_parent->Data() == modelEntity)
            modelJoints.push_back(_jointEntity);
          return true;
        });

    std::set<Entity> allLinks(modelLinks.begin(), modelLinks.end());
    std::set<Entity> childLinks;
    std::map<Entity, Entity> parentMap;  // child -> parent link

    for (auto jointEntity : modelJoints)
    {
      auto childNameComp =
          _ecm.Component<components::ChildLinkName>(jointEntity);
      auto parentNameComp =
          _ecm.Component<components::ParentLinkName>(jointEntity);
      if (childNameComp && parentNameComp)
      {
        Entity childLinkEntity = _ecm.EntityByComponents(
            components::Link(), components::ParentEntity(modelEntity),
            components::Name(childNameComp->Data()));
        Entity parentLinkEntity = _ecm.EntityByComponents(
            components::Link(), components::ParentEntity(modelEntity),
            components::Name(parentNameComp->Data()));

        if (childLinkEntity != kNullEntity && parentLinkEntity != kNullEntity)
        {
          childLinks.insert(childLinkEntity);
          parentMap[childLinkEntity] = parentLinkEntity;
        }
      }
    }

    std::vector<Entity> rootLinks;
    for (auto linkEntity : modelLinks)
    {
      if (childLinks.find(linkEntity) == childLinks.end())
      {
        rootLinks.push_back(linkEntity);
      }
    }

    for (auto rootLink : rootLinks)
    {
      auto nameComp = _ecm.Component<components::Name>(rootLink);
      std::string linkName =
          nameComp ? nameComp->Data() : std::to_string(rootLink);
      std::string bodyName = modelName;
      bodyName += "::";
      bodyName += linkName;
      mjsBody *newBody = mjs_addBody(worldBody, nullptr);
      if (newBody)
      {
        mjs_setName(newBody->element, bodyName.c_str());
        _entityToMujocoBody[rootLink] = newBody;
        math::Pose3d wPose = gz::sim::worldPose(rootLink, _ecm);
        SetPose(newBody->pos, newBody->quat, wPose);
        _specChanged = true;

        auto inertialComp = _ecm.Component<components::Inertial>(rootLink);
        if (inertialComp)
        {
          const auto &inertial = inertialComp->Data();
          newBody->explicitinertial = 1;
          newBody->mass = inertial.MassMatrix().Mass();
          SetInertial(newBody->fullinertia, newBody->ipos, newBody->iquat,
                      inertial);
        }

        if (!isStatic)
        {
          mjs_addFreeJoint(newBody);
        }
      }
    }

    std::queue<Entity> q;
    for (auto rootLink : rootLinks) q.push(rootLink);

    std::map<Entity, std::vector<Entity>> childrenMap;
    for (auto linkEntity : modelLinks)
    {
      if (parentMap.find(linkEntity) != parentMap.end())
      {
        childrenMap[parentMap[linkEntity]].push_back(linkEntity);
      }
    }

    while (!q.empty())
    {
      Entity parentLink = q.front();
      q.pop();

      auto parentNameComp = _ecm.Component<components::Name>(parentLink);
      std::string parentLinkName =
          parentNameComp ? parentNameComp->Data() : std::to_string(parentLink);
      std::string parentBodyName = modelName;
      parentBodyName += "::";
      parentBodyName += parentLinkName;
      mjsBody *parentBody = mjs_findBody(this->spec, parentBodyName.c_str());

      for (auto childLink : childrenMap[parentLink])
      {
        auto childNameComp = _ecm.Component<components::Name>(childLink);
        std::string childLinkName =
            childNameComp ? childNameComp->Data() : std::to_string(childLink);
        std::string childBodyName = modelName;
        childBodyName += "::";
        childBodyName += childLinkName;
        mjsBody *childBody = mjs_addBody(parentBody, nullptr);
        if (childBody)
        {
          mjs_setName(childBody->element, childBodyName.c_str());
          _entityToMujocoBody[childLink] = childBody;

          math::Pose3d parentWorldPose = gz::sim::worldPose(parentLink, _ecm);
          math::Pose3d childWorldPose = gz::sim::worldPose(childLink, _ecm);
          math::Pose3d relativePose =
              parentWorldPose.Inverse() * childWorldPose;

          SetPose(childBody->pos, childBody->quat, relativePose);
          _specChanged = true;

          auto inertialComp = _ecm.Component<components::Inertial>(childLink);
          if (inertialComp)
          {
            const auto &inertial = inertialComp->Data();
            childBody->explicitinertial = 1;
            childBody->mass = inertial.MassMatrix().Mass();
            SetInertial(childBody->fullinertia, childBody->ipos,
                        childBody->iquat, inertial);
          }

          q.push(childLink);
        }
      }
    }

    auto canonicalLinkComp =
        _ecm.Component<components::ModelCanonicalLink>(modelEntity);
    if (canonicalLinkComp)
    {
      Entity canonicalLink = canonicalLinkComp->Data();
      mjsBody *canonBody = nullptr;
      if (_entityToMujocoBody.find(canonicalLink) != _entityToMujocoBody.end())
      {
        canonBody = _entityToMujocoBody[canonicalLink];
      }
      else
      {
        auto linkNameComp = _ecm.Component<components::Name>(canonicalLink);
        std::string linkName =
            linkNameComp ? linkNameComp->Data() : std::to_string(canonicalLink);
        std::string fullLinkName = modelName;
        fullLinkName += "::";
        fullLinkName += linkName;
        canonBody = mjs_findBody(this->spec, fullLinkName.c_str());
      }

      if (canonBody)
      {
        mjsSite *modelSite = mjs_addSite(canonBody, nullptr);
        std::string siteName = modelName + "::model_frame";
        mjs_setName(modelSite->element, siteName.c_str());

        auto linkPoseComp = _ecm.Component<components::Pose>(canonicalLink);
        math::Pose3d modelFromLink =
            linkPoseComp ? linkPoseComp->Data().Inverse() : math::Pose3d::Zero;

        SetPose(modelSite->pos, modelSite->quat, modelFromLink);
        _specChanged = true;
      }
    }
  }
}

void MujocoPhysics::CreateCollisions(
    EntityComponentManager &_ecm, bool &_specChanged,
    const std::unordered_map<Entity, mjsBody *> &_entityToMujocoBody)
{
  mjsBody *worldBody = mjs_findBody(this->spec, "world");

  _ecm.EachNew<components::Collision, components::Name,
               components::ParentEntity>(
      [&](const Entity &entity, const components::Collision *,
          const components::Name *nameComp,
          const components::ParentEntity *parentComp) -> bool
      {
        mjsBody *parentBody = worldBody;
        auto linkNameComp =
            _ecm.Component<components::Name>(parentComp->Data());
        std::string linkName = linkNameComp
                                   ? linkNameComp->Data()
                                   : std::to_string(parentComp->Data());
        Entity modelEntity = _ecm.ParentEntity(parentComp->Data());
        auto modelNameComp = _ecm.Component<components::Name>(modelEntity);
        std::string modelName =
            modelNameComp ? modelNameComp->Data() : std::to_string(modelEntity);
        std::string parentName = modelName + "::" + linkName;
        mjsBody *foundBody = nullptr;
        if (_entityToMujocoBody.find(parentComp->Data()) !=
            _entityToMujocoBody.end())
          foundBody = _entityToMujocoBody.at(parentComp->Data());
        else
          foundBody = mjs_findBody(this->spec, parentName.c_str());

        if (foundBody)
          parentBody = foundBody;
        else
          gzerr << "Could not find parent body " << parentName
                << " for collision entity " << entity << "\n";

        mjsGeom *newGeom = mjs_addGeom(parentBody, nullptr);
        if (newGeom)
        {
          std::string geomName =
              modelName + "::" + linkName + "::" + nameComp->Data();
          mjs_setName(newGeom->element, geomName.c_str());

          newGeom->condim = 3;
          auto _collisionComp =
              _ecm.Component<components::CollisionElement>(entity);
          if (_collisionComp && _collisionComp->Data().Surface() &&
              _collisionComp->Data().Surface()->Friction() &&
              _collisionComp->Data().Surface()->Friction()->ODE())
          {
            auto friction = _collisionComp->Data().Surface()->Friction()->ODE();
            newGeom->friction[0] = friction->Mu();
            newGeom->friction[1] = friction->Mu2();
          }

          if (_collisionComp)
          {
            const auto &sdfCollision = _collisionComp->Data();
            if (sdfCollision.Surface() && sdfCollision.Surface()->Contact())
            {
              newGeom->contype =
                  sdfCollision.Surface()->Contact()->CategoryBitmask().value_or(
                      1);
              newGeom->conaffinity =
                  sdfCollision.Surface()->Contact()->CollideBitmask();
            }
          }

          auto poseComp = _ecm.Component<components::Pose>(entity);
          if (poseComp)
          {
            SetPose(newGeom->pos, newGeom->quat, poseComp->Data());
          }

          auto geomComp = _ecm.Component<components::Geometry>(entity);
          if (geomComp)
          {
            const sdf::Geometry &shape = geomComp->Data();
            switch (shape.Type())
            {
              case ::sdf::GeometryType::BOX:
              {
                newGeom->type = mjGEOM_BOX;
                SetVector(newGeom->size, shape.BoxShape()->Size() / 2.0);
                break;
              }
              case ::sdf::GeometryType::CYLINDER:
              {
                newGeom->type = mjGEOM_CYLINDER;
                newGeom->size[0] = shape.CylinderShape()->Radius();
                newGeom->size[1] = shape.CylinderShape()->Length() / 2.0;
                break;
              }
              case ::sdf::GeometryType::PLANE:
              {
                newGeom->type = mjGEOM_PLANE;
                for (int j = 0; j < 2; ++j)
                  newGeom->size[j] = shape.PlaneShape()->Size()[j] / 2.0;
                newGeom->size[2] = 1.0;
                break;
              }
              case ::sdf::GeometryType::SPHERE:
              {
                newGeom->type = mjGEOM_SPHERE;
                newGeom->size[0] = shape.SphereShape()->Radius();
                break;
              }
              case ::sdf::GeometryType::CAPSULE:
              {
                newGeom->type = mjGEOM_CAPSULE;
                newGeom->size[0] = shape.CapsuleShape()->Radius();
                newGeom->size[1] = shape.CapsuleShape()->Length() / 2.0;
                break;
              }
              case ::sdf::GeometryType::ELLIPSOID:
              {
                newGeom->type = mjGEOM_ELLIPSOID;
                SetVector(newGeom->size, shape.EllipsoidShape()->Radii());
                break;
              }
              case ::sdf::GeometryType::CONE:
              {
                newGeom->type = mjGEOM_MESH;
                std::string meshName = geomName + "_cone";
                auto *muMesh = mjs_addMesh(this->spec, nullptr);
                mjs_setName(muMesh->element, meshName.c_str());
                mjs_setString(newGeom->meshname, meshName.c_str());
                muMesh->scale[0] = shape.ConeShape()->Radius();
                muMesh->scale[1] = shape.ConeShape()->Radius();
                muMesh->scale[2] = shape.ConeShape()->Length() / 2.0;
                double params[3] = {36, 0};
                mjs_makeMesh(muMesh, mjMESH_BUILTIN_CONE, params, 2);
                break;
              }
              case ::sdf::GeometryType::MESH:
              {
                newGeom->type = mjGEOM_MESH;
                auto *meshSdf = shape.MeshShape();
                std::string meshName = geomName + "_mesh";
                auto &meshManager = *gz::common::MeshManager::Instance();
                auto *mesh = meshManager.Load(meshSdf->Uri());
                if (mesh)
                {
                  auto *muMesh = mjs_addMesh(this->spec, nullptr);
                  mjs_setName(muMesh->element, meshName.c_str());
                  mjs_setString(newGeom->meshname, meshName.c_str());
                  muMesh->scale[0] = meshSdf->Scale().X();
                  muMesh->scale[1] = meshSdf->Scale().Y();
                  muMesh->scale[2] = meshSdf->Scale().Z();

                  double *verts{nullptr};
                  int *indices{nullptr};
                  mesh->FillArrays(&verts, &indices);
                  auto nverts = mesh->VertexCount();

                  if (nverts > 0 && verts)
                  {
                    muMesh->uservert->assign(static_cast<size_t>(nverts) * 3,
                                             0.0f);
                    std::transform(verts,
                                   verts + static_cast<size_t>(nverts) * 3,
                                   muMesh->uservert->begin(), [](double val)
                                   { return static_cast<float>(val); });
                  }
                  if (mesh->IndexCount() > 0 && indices)
                  {
                    muMesh->userface->assign(indices,
                                             indices + mesh->IndexCount());
                  }

                  delete[] verts;
                  delete[] indices;
                }
                else
                {
                  gzwarn << "Failed to load mesh: " << meshSdf->Uri() << "\n";
                }
                break;
              }
              default:
                break;
            }
          }
          _specChanged = true;
        }
        return true;
      });
}

void MujocoPhysics::CreateJoints(
    EntityComponentManager &_ecm, bool &_specChanged,
    const std::unordered_map<Entity, mjsBody *> &_entityToMujocoBody)
{
  _ecm.EachNew<components::Joint, components::Name, components::JointType,
               components::ParentEntity, components::ChildLinkName>(
      [&](const Entity &entity, const components::Joint *,
          const components::Name *nameComp,
          const components::JointType *jointTypeComp,
          const components::ParentEntity *parentModelComp,
          const components::ChildLinkName *childLinkNameComp) -> bool
      {
        Entity childLinkEntity = _ecm.EntityByComponents(
            components::Link(),
            components::ParentEntity(parentModelComp->Data()),
            components::Name(childLinkNameComp->Data()));

        if (childLinkEntity != kNullEntity)
        {
          Entity modelEntity = parentModelComp->Data();
          auto modelNameComp = _ecm.Component<components::Name>(modelEntity);
          std::string modelName = modelNameComp ? modelNameComp->Data()
                                                : std::to_string(modelEntity);
          std::string childName = modelName + "::" + childLinkNameComp->Data();
          mjsBody *childBody = nullptr;
          if (_entityToMujocoBody.find(childLinkEntity) !=
              _entityToMujocoBody.end())
            childBody = _entityToMujocoBody.at(childLinkEntity);
          else
            childBody = mjs_findBody(this->spec, childName.c_str());

          if (childBody)
          {
            mjsJoint *joint{nullptr};
            mjsActuator *actuator{nullptr};
            auto type = jointTypeComp->Data();

            if (type == sdf::JointType::PRISMATIC)
            {
              joint = mjs_addJoint(childBody, nullptr);
              joint->type = mjJNT_SLIDE;
            }
            else if (type == sdf::JointType::REVOLUTE)
            {
              joint = mjs_addJoint(childBody, nullptr);
              joint->type = mjJNT_HINGE;
            }
            else if (type == sdf::JointType::BALL)
            {
              joint = mjs_addJoint(childBody, nullptr);
              joint->type = mjJNT_BALL;
            }
            else if (type == sdf::JointType::FIXED)
            {
              // Handled implicitly
            }
            else if (type == sdf::JointType::UNIVERSAL)
            {
              gzwarn << "Universal joint unsupported.\n";
            }
            else
            {
              gzwarn << "Unsupported joint type for joint [" << nameComp->Data()
                     << "]\n";
            }

            if (joint)
            {
              std::string mjJointName = modelName + "::" + nameComp->Data();
              mjs_setName(joint->element, mjJointName.c_str());

              auto jointPoseComp = _ecm.Component<components::Pose>(entity);
              if (jointPoseComp)
              {
                SetVector(joint->pos, jointPoseComp->Data().Pos());
              }

              auto axisComp = _ecm.Component<components::JointAxis>(entity);
              if (axisComp && type != sdf::JointType::BALL)
              {
                auto axis = axisComp->Data();
                SetVector(joint->axis, axis.Xyz());

                joint->limited =
                    !std::isinf(axis.Lower()) && !std::isinf(axis.Upper());
                if (joint->limited)
                {
                  joint->range[0] = axis.Lower();
                  joint->range[1] = axis.Upper();
                }
                joint->frictionloss = axis.Friction();
                joint->damping = axis.Damping();

                joint->springref = axis.SpringReference();
                joint->stiffness = axis.SpringStiffness();

                double effort = axis.Effort();
                if (effort >= 0 && !std::isinf(effort))
                {
                  joint->actfrclimited = 1;
                  joint->actfrcrange[0] = -effort;
                  joint->actfrcrange[1] = effort;
                }
                else
                {
                  joint->actfrclimited = 0;
                }
              }

              actuator = mjs_addActuator(this->spec, nullptr);
              if (actuator)
              {
                actuator->trntype = mjTRN_JOINT;
                mjs_setString(actuator->target, mjJointName.c_str());
              }

              _specChanged = true;
            }
          }
        }
        return true;
      });
}

void MujocoPhysics::CreateSensors(
    EntityComponentManager &_ecm, bool &_specChanged,
    const std::unordered_map<Entity, mjsBody *> &_entityToMujocoBody)
{
  _ecm.EachNew<components::Imu, components::ParentEntity>(
      [&](const Entity &_entity, const components::Imu *,
          const components::ParentEntity *parentComp) -> bool
      {
        Entity parentLink = parentComp->Data();
        if (parentLink != kNullEntity)
        {
          auto linkNameComp = _ecm.Component<components::Name>(parentLink);
          std::string linkName =
              linkNameComp ? linkNameComp->Data() : std::to_string(parentLink);
          Entity modelEntity = _ecm.ParentEntity(parentLink);
          auto modelNameComp = _ecm.Component<components::Name>(modelEntity);
          std::string modelName = modelNameComp ? modelNameComp->Data()
                                                : std::to_string(modelEntity);
          std::string fullLinkName = modelName + "::" + linkName;
          mjsBody *muBody = nullptr;
          if (_entityToMujocoBody.find(parentLink) != _entityToMujocoBody.end())
            muBody = _entityToMujocoBody.at(parentLink);
          else
            muBody = mjs_findBody(this->spec, fullLinkName.c_str());

          if (muBody)
          {
            auto *muSite = mjs_addSite(muBody, nullptr);

            auto *accelSensor = mjs_addSensor(this->spec);
            accelSensor->type = mjSENS_ACCELEROMETER;
            accelSensor->objtype = mjOBJ_SITE;

            auto *gyroSensor = mjs_addSensor(this->spec);
            gyroSensor->type = mjSENS_GYRO;
            gyroSensor->objtype = mjOBJ_SITE;
            _specChanged = true;
          }
        }
        return true;
      });

  _ecm.EachNew<components::ForceTorque, components::ParentEntity>(
      [&](const Entity &_entity, const components::ForceTorque *,
          const components::ParentEntity *parentComp) -> bool
      {
        Entity parentJoint = parentComp->Data();
        if (parentJoint != kNullEntity)
        {
          auto childLinkComp =
              _ecm.Component<components::ChildLinkName>(parentJoint);
          if (childLinkComp)
          {
            std::string childName = childLinkComp->Data();
            Entity modelEntity = _ecm.ParentEntity(parentJoint);
            Entity childLinkEntity = _ecm.EntityByComponents(
                components::ParentEntity(modelEntity),
                components::Name(childName), components::Link());
            if (childLinkEntity != kNullEntity)
            {
              auto linkNameComp =
                  _ecm.Component<components::Name>(childLinkEntity);
              std::string linkName = linkNameComp
                                         ? linkNameComp->Data()
                                         : std::to_string(childLinkEntity);
              auto modelNameComp =
                  _ecm.Component<components::Name>(modelEntity);
              std::string modelName = modelNameComp
                                          ? modelNameComp->Data()
                                          : std::to_string(modelEntity);
              std::string fullLinkName = modelName + "::" + linkName;
              mjsBody *muBody = nullptr;
              if (_entityToMujocoBody.find(childLinkEntity) !=
                  _entityToMujocoBody.end())
                muBody = _entityToMujocoBody.at(childLinkEntity);
              else
                muBody = mjs_findBody(this->spec, fullLinkName.c_str());

              if (muBody)
              {
                auto *muSite = mjs_addSite(muBody, nullptr);

                auto *forceSensor = mjs_addSensor(this->spec);
                forceSensor->type = mjSENS_FORCE;
                forceSensor->objtype = mjOBJ_SITE;

                auto *torqueSensor = mjs_addSensor(this->spec);
                torqueSensor->type = mjSENS_TORQUE;
                torqueSensor->objtype = mjOBJ_SITE;
                _specChanged = true;
              }
            }
          }
        }
        return true;
      });
}

void MujocoPhysics::PostRecompileSetup(EntityComponentManager &_ecm)
{
  if (this->model)
  {
    this->geomIdToEntity.assign(static_cast<size_t>(this->model->ngeom),
                                kNullEntity);
  }

  _ecm.Each<components::Link, components::Name, components::ParentEntity>(
      [&](const Entity &_entity, const components::Link *,
          const components::Name *nameComp,
          const components::ParentEntity *parentComp) -> bool
      {
        auto modelNameComp =
            _ecm.Component<components::Name>(parentComp->Data());
        std::string modelName = modelNameComp
                                    ? modelNameComp->Data()
                                    : std::to_string(parentComp->Data());
        std::string bodyName = modelName + "::" + nameComp->Data();
        int id = mj_name2id(this->model, mjOBJ_BODY, bodyName.c_str());
        if (id >= 0)
        {
          _ecm.SetComponentData<MujocoBodyId>(_entity, id);
        }
        return true;
      });

  _ecm.Each<components::Collision, components::Name, components::ParentEntity>(
      [&](const Entity &_entity, const components::Collision *,
          const components::Name *nameComp,
          const components::ParentEntity *parentComp) -> bool
      {
        auto linkNameComp =
            _ecm.Component<components::Name>(parentComp->Data());
        std::string linkName = linkNameComp
                                   ? linkNameComp->Data()
                                   : std::to_string(parentComp->Data());
        Entity modelEntity = _ecm.ParentEntity(parentComp->Data());
        auto modelNameComp = _ecm.Component<components::Name>(modelEntity);
        std::string modelName =
            modelNameComp ? modelNameComp->Data() : std::to_string(modelEntity);
        std::string geomName =
            modelName + "::" + linkName + "::" + nameComp->Data();
        int id = mj_name2id(this->model, mjOBJ_GEOM, geomName.c_str());
        if (id >= 0)
        {
          _ecm.SetComponentData<MujocoGeomId>(_entity, id);
          if (id < static_cast<int>(this->geomIdToEntity.size()))
          {
            this->geomIdToEntity[static_cast<size_t>(id)] = _entity;
          }
        }
        return true;
      });

  _ecm.Each<components::Joint, components::Name, components::ParentEntity>(
      [&](const Entity &_entity, const components::Joint *,
          const components::Name *nameComp,
          const components::ParentEntity *parentComp) -> bool
      {
        auto modelNameComp =
            _ecm.Component<components::Name>(parentComp->Data());
        std::string modelName = modelNameComp
                                    ? modelNameComp->Data()
                                    : std::to_string(parentComp->Data());
        std::string jointName = modelName + "::" + nameComp->Data();
        int id = mj_name2id(this->model, mjOBJ_JOINT, jointName.c_str());
        if (id >= 0)
        {
          _ecm.SetComponentData<MujocoJointId>(_entity, id);
        }

        int actId = -1;
        for (int i = 0; i < this->model->nu; ++i)
        {
          if (this->model->actuator_trntype[i] == mjTRN_JOINT &&
              this->model->actuator_trnid[static_cast<ptrdiff_t>(i) * 2] == id)
          {
            actId = i;
            break;
          }
        }
        if (actId >= 0)
        {
          _ecm.SetComponentData<MujocoActuatorId>(_entity, actId);
        }
        return true;
      });

  _ecm.EachNew<components::Model, components::Name,
               components::ModelCanonicalLink>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *nameComp,
          const components::ModelCanonicalLink *) -> bool
      {
        std::string modelName =
            nameComp ? nameComp->Data() : std::to_string(_entity);
        std::string siteName = modelName + "::model_frame";
        int id = mj_name2id(this->model, mjOBJ_SITE, siteName.c_str());
        if (id >= 0)
        {
          _ecm.SetComponentData<MujocoModelSiteId>(_entity, id);
        }
        return true;
      });

  _ecm.Each<components::Model, components::ParentEntity>(
      [&](const Entity &_entity, const components::Model *,
          const components::ParentEntity *parentComp) -> bool
      {
        Entity parent = parentComp->Data();
        auto siteComp = _ecm.Component<MujocoModelSiteId>(parent);
        if (siteComp)
        {
          _ecm.SetComponentData<MujocoParentSiteId>(_entity, siteComp->Data());
        }
        else
        {
          _ecm.SetComponentData<MujocoParentSiteId>(_entity, -1);
        }
        _ecm.SetComponentData<MujocoParentHasPose>(
            _entity, _ecm.Component<components::Pose>(parent) != nullptr);
        return true;
      });

  _ecm.Each<components::Link, components::ParentEntity>(
      [&](const Entity &_entity, const components::Link *,
          const components::ParentEntity *parentComp) -> bool
      {
        Entity parent = parentComp->Data();
        auto siteComp = _ecm.Component<MujocoModelSiteId>(parent);
        if (siteComp)
        {
          _ecm.SetComponentData<MujocoParentSiteId>(_entity, siteComp->Data());
        }
        else
        {
          _ecm.SetComponentData<MujocoParentSiteId>(_entity, -1);
        }
        _ecm.SetComponentData<MujocoParentHasPose>(
            _entity, _ecm.Component<components::Pose>(parent) != nullptr);
        return true;
      });
}

void MujocoPhysics::UpdatePhysics(EntityComponentManager &_ecm)
{
  if (!this->model || !this->data)
    return;

  this->ApplyPoseAndVelocityCmds(_ecm);
  this->ApplyJointAndLinkCmds(_ecm);
}

int MujocoPhysics::FindFreeJoint(Entity _canonicalLinkEntity,
                                 const EntityComponentManager &_ecm) const
{
  auto bodyIdComp = _ecm.Component<MujocoBodyId>(_canonicalLinkEntity);
  if (!bodyIdComp || !this->model)
    return -1;

  int bodyId = bodyIdComp->Data();
  if (bodyId > 0 && bodyId < this->model->nbody)
  {
    int jntAdr = this->model->body_jntadr[bodyId];
    int jntNum = this->model->body_jntnum[bodyId];
    for (int i = 0; i < jntNum; ++i)
    {
      if (this->model->jnt_type[jntAdr + i] == mjJNT_FREE)
      {
        return jntAdr + i;
      }
    }
  }
  return -1;
}

void MujocoPhysics::ApplyPoseAndVelocityCmds(EntityComponentManager &_ecm)
{
  if (_ecm.HasComponentType(components::WorldPoseCmd::typeId))
  {
    // Process WorldPoseCmd
    auto olderWorldPoseCmdsToRemove = std::move(this->worldPoseCmdsToRemove);
    this->worldPoseCmdsToRemove.clear();

    _ecm.Each<components::Model, components::WorldPoseCmd,
              components::ModelCanonicalLink>(
        [&](const Entity &_entity, const components::Model *,
            const components::WorldPoseCmd *_poseCmd,
            const components::ModelCanonicalLink *_canonLink) -> bool
        {
          this->worldPoseCmdsToRemove.insert(_entity);
          math::Pose3d worldPoseCmd = _poseCmd->Data();
          if (!worldPoseCmd.Pos().IsFinite() ||
              !worldPoseCmd.Rot().IsFinite() ||
              worldPoseCmd.Rot() == math::Quaterniond::Zero)
            return true;

          Entity canonicalLinkEntity = _canonLink->Data();
          int freeJntId = this->FindFreeJoint(canonicalLinkEntity, _ecm);
          if (freeJntId >= 0)
          {
            int qposAdr = this->model->jnt_qposadr[freeJntId];
            SetPose(this->data->qpos + qposAdr, this->data->qpos + qposAdr + 3,
                    worldPoseCmd);
          }
          return true;
        });

    for (const Entity &entity : olderWorldPoseCmdsToRemove)
    {
      _ecm.RemoveComponent<components::WorldPoseCmd>(entity);
    }
  }

  if (_ecm.HasComponentType(components::LinearVelocityCmd::typeId))
  {
    // Process LinearVelocityCmd
    _ecm.Each<components::Model, components::LinearVelocityCmd,
              components::ModelCanonicalLink>(
        [&](const Entity &, const components::Model *,
            const components::LinearVelocityCmd *_velCmd,
            const components::ModelCanonicalLink *_canonLink) -> bool
        {
          math::Vector3d velCmd = _velCmd->Data();
          Entity canonicalLinkEntity = _canonLink->Data();
          int freeJntId = this->FindFreeJoint(canonicalLinkEntity, _ecm);
          if (freeJntId >= 0)
          {
            int dofAdr = this->model->jnt_dofadr[freeJntId];
            SetVector(this->data->qvel + dofAdr, velCmd);
          }
          return true;
        });
  }

  // Process AngularVelocityCmd
  if (_ecm.HasComponentType(components::AngularVelocityCmd::typeId))
  {
    _ecm.Each<components::Model, components::AngularVelocityCmd,
              components::ModelCanonicalLink>(
        [&](const Entity &, const components::Model *,
            const components::AngularVelocityCmd *_velCmd,
            const components::ModelCanonicalLink *_canonLink) -> bool
        {
          math::Vector3d velCmd = _velCmd->Data();
          Entity canonicalLinkEntity = _canonLink->Data();
          int freeJntId = this->FindFreeJoint(canonicalLinkEntity, _ecm);
          if (freeJntId >= 0)
          {
            int dofAdr = this->model->jnt_dofadr[freeJntId];
            SetVector(this->data->qvel + dofAdr + 3, velCmd);
          }
          return true;
        });
  }
}

void MujocoPhysics::ApplyJointAndLinkCmds(EntityComponentManager &_ecm)
{
  // Clear actuator controls
  if (this->model->nu > 0)
  {
    std::fill(this->data->ctrl, this->data->ctrl + this->model->nu, 0.0);
  }

  if (_ecm.HasComponentType(components::JointVelocityCmd::typeId))
  {
    // Apply JointForceCmd and JointVelocityCmd
    _ecm.Each<components::Joint, MujocoActuatorId,
              components::JointVelocityCmd>(
        [&](const Entity &entity, const components::Joint *,
            const MujocoActuatorId *_actuatorIdComp,
            const components::JointVelocityCmd *_velCmd) -> bool
        {
          int actId = _actuatorIdComp->Data();
          if (actId >= 0 && actId < this->model->nu)
          {
            if (!_velCmd->Data().empty())
            {
              this->data->ctrl[actId] = _velCmd->Data()[0];
            }
          }

          return true;
        });
  }

  if (_ecm.HasComponentType(components::JointForceCmd::typeId))
  {
    _ecm.Each<components::Joint, MujocoActuatorId, components::JointForceCmd>(
        [&](const Entity &entity, const components::Joint *,
            const MujocoActuatorId *_actuatorIdComp,
            const components::JointForceCmd *_forceCmd) -> bool
        {
          int actId = _actuatorIdComp->Data();
          if (actId >= 0 && actId < this->model->nu)
          {
            if (!_forceCmd->Data().empty())
            {
              this->data->ctrl[actId] = _forceCmd->Data()[0];
            }
          }

          return true;
        });
  }

  // Clear applied forces
  std::fill(this->data->xfrc_applied,
            this->data->xfrc_applied + 6 * this->model->nbody, 0.0);

  if (_ecm.HasComponentType(components::ExternalWorldWrenchCmd::typeId))
  {
    // Apply ExternalWorldWrenchCmd
    _ecm.Each<components::Link, MujocoBodyId,
              components::ExternalWorldWrenchCmd>(
        [&](const Entity &, const components::Link *,
            const MujocoBodyId *_bodyIdComp,
            const components::ExternalWorldWrenchCmd *_wrenchComp) -> bool
        {
          int mjBodyId = _bodyIdComp->Data();
          if (mjBodyId > 0 && mjBodyId < this->model->nbody)
          {
            AddWrench(
                this->data->xfrc_applied + static_cast<ptrdiff_t>(mjBodyId) * 6,
                _wrenchComp->Data());
          }
          return true;
        });
  }
}

void MujocoPhysics::Step(const UpdateInfo &_info)
{
  if (_info.paused || !this->model || !this->data)
    return;
  // Step the simulation
  mj_step(this->model, this->data);
}

namespace
{
template <typename ComponentType>
void RemoveComponentType(EntityComponentManager &_ecm)
{
  if (_ecm.HasComponentType(ComponentType::typeId))
  {
    std::vector<Entity> entities;
    _ecm.Each<ComponentType>(
        [&](const Entity &_entity, ComponentType *) -> bool
        {
          entities.push_back(_entity);
          return true;
        });

    for (const auto entity : entities)
    {
      _ecm.RemoveComponent<ComponentType>(entity);
    }
  }
}

template <typename ComponentType>
void ClearComponentData(EntityComponentManager &_ecm)
{
  if (_ecm.HasComponentType(ComponentType::typeId))
  {
    _ecm.Each<ComponentType>(
        [&](const Entity &, ComponentType *_comp) -> bool
        {
          if constexpr (std::is_same_v<ComponentType,
                                       components::JointForceCmd>)
          {
            std::fill(_comp->Data().begin(), _comp->Data().end(), 0.0);
          }
          else if constexpr (std::is_same_v<ComponentType,
                                            components::ExternalWorldWrenchCmd>)
          {
            _comp->Data().Clear();
          }
          else
          {
            _comp->Data().clear();
          }
          return true;
        });
  }
}
}  // namespace

void MujocoPhysics::UpdateSim(const UpdateInfo &_info,
                              EntityComponentManager &_ecm)
{
  if (_info.paused || !this->model || !this->data)
    return;

  this->UpdatePoses(_ecm);
  this->UpdateVelocities(_ecm);
  this->UpdateSensors(_ecm);
  this->ClearResetsAndCommands(_ecm);
}

void MujocoPhysics::UpdatePoses(EntityComponentManager &_ecm)
{
  // Synchronize the ECM components::Pose of all models
  _ecm.Each<components::Model, components::Pose, MujocoModelSiteId,
            components::ParentEntity, MujocoParentSiteId, MujocoParentHasPose>(
      [&](const Entity &entity, const components::Model *,
          components::Pose *poseComp, const MujocoModelSiteId *_siteIdComp,
          const components::ParentEntity *_parentComp,
          const MujocoParentSiteId *_parentSiteIdComp,
          const MujocoParentHasPose *_parentHasPoseComp) -> bool
      {
        int mjSiteId = _siteIdComp->Data();
        if (mjSiteId >= 0 && mjSiteId < this->model->nsite)
        {
          math::Pose3d worldPose = GetPoseFromMat(
              this->data->site_xpos + static_cast<ptrdiff_t>(mjSiteId) * 3,
              this->data->site_xmat + static_cast<ptrdiff_t>(mjSiteId) * 9);

          math::Pose3d parentWorldPose = math::Pose3d::Zero;
          Entity parent = _parentComp->Data();
          int parentSiteId = _parentSiteIdComp->Data();
          if (parentSiteId >= 0 && parentSiteId < this->model->nsite)
          {
            parentWorldPose =
                GetPoseFromMat(this->data->site_xpos +
                                   static_cast<ptrdiff_t>(parentSiteId) * 3,
                               this->data->site_xmat +
                                   static_cast<ptrdiff_t>(parentSiteId) * 9);
          }
          else if (_parentHasPoseComp->Data())
          {
            parentWorldPose = gz::sim::worldPose(parent, _ecm);
          }

          if (poseComp->SetData(parentWorldPose.Inverse() * worldPose,
                                [](const auto &_a, const auto &_b)
                                { return _a.Equal(_b, 1e-6); }))
          {
            _ecm.SetChanged(entity, components::Pose::typeId,
                            ComponentState::PeriodicChange);
          }
        }
        return true;
      });

  // Synchronize the ECM components::Pose of all links
  _ecm.Each<components::Link, MujocoBodyId, components::Pose,
            components::ParentEntity, MujocoParentSiteId, MujocoParentHasPose>(
      [&](const Entity &entity, const components::Link *,
          const MujocoBodyId *_bodyIdComp, components::Pose *_poseComp,
          const components::ParentEntity *_parentComp,
          const MujocoParentSiteId *_parentSiteIdComp,
          const MujocoParentHasPose *_parentHasPoseComp) -> bool
      {
        int mjBodyId = _bodyIdComp->Data();
        if (mjBodyId > 0 && mjBodyId < this->model->nbody)
        {
          math::Pose3d worldPose =
              GetPose(this->data->xpos + static_cast<ptrdiff_t>(mjBodyId) * 3,
                      this->data->xquat + static_cast<ptrdiff_t>(mjBodyId) * 4);

          math::Pose3d parentWorldPose = math::Pose3d::Zero;
          Entity parent = _parentComp->Data();
          int parentSiteId = _parentSiteIdComp->Data();
          if (parentSiteId >= 0 && parentSiteId < this->model->nsite)
          {
            parentWorldPose =
                GetPoseFromMat(this->data->site_xpos +
                                   static_cast<ptrdiff_t>(parentSiteId) * 3,
                               this->data->site_xmat +
                                   static_cast<ptrdiff_t>(parentSiteId) * 9);
          }
          else if (_parentHasPoseComp->Data())
          {
            parentWorldPose = gz::sim::worldPose(parent, _ecm);
          }

          if (_poseComp->SetData(parentWorldPose.Inverse() * worldPose,
                                 [](const auto &_a, const auto &_b)
                                 { return _a.Equal(_b, 1e-6); }))
          {
            _ecm.SetChanged(entity, components::Pose::typeId,
                            ComponentState::PeriodicChange);
          }
        }
        return true;
      });
}

void MujocoPhysics::UpdateVelocities(EntityComponentManager &_ecm)
{
  if (_ecm.HasComponentType(components::LinearVelocity::typeId))
  {
    // Update LinearVelocity
    _ecm.Each<components::Link, MujocoBodyId, components::LinearVelocity>(
        [&](const Entity &, const components::Link *,
            const MujocoBodyId *_bodyIdComp,
            components::LinearVelocity *_linVelComp) -> bool
        {
          int mjBodyId = _bodyIdComp->Data();
          if (mjBodyId > 0 && mjBodyId < this->model->nbody)
          {
            mjtNum velocity[6];
            mj_objectVelocity(this->model, this->data, mjOBJ_BODY, mjBodyId,
                              velocity, 0);
            math::Vector3d worldLinearVel = GetVector(velocity + 3);
            math::Quaterniond worldRot = GetQuaternion(
                this->data->xquat + static_cast<ptrdiff_t>(mjBodyId) * 4);

            _linVelComp->Data() = worldRot.Inverse() * worldLinearVel;
          }
          return true;
        });
  }

  if (_ecm.HasComponentType(components::AngularVelocity::typeId))
  {
    // Update AngularVelocity
    _ecm.Each<components::Link, MujocoBodyId, components::AngularVelocity>(
        [&](const Entity &, const components::Link *,
            const MujocoBodyId *_bodyIdComp,
            components::AngularVelocity *_angVelComp) -> bool
        {
          int mjBodyId = _bodyIdComp->Data();
          if (mjBodyId > 0 && mjBodyId < this->model->nbody)
          {
            mjtNum velocity[6];
            mj_objectVelocity(this->model, this->data, mjOBJ_BODY, mjBodyId,
                              velocity, 0);
            math::Vector3d worldAngularVel = GetVector(velocity);
            math::Quaterniond worldRot = GetQuaternion(
                this->data->xquat + static_cast<ptrdiff_t>(mjBodyId) * 4);

            _angVelComp->Data() = worldRot.Inverse() * worldAngularVel;
          }
          return true;
        });
  }
}

void MujocoPhysics::UpdateSensors(EntityComponentManager &_ecm)
{
  // Process contacts
  if (_ecm.HasComponentType(components::ContactSensorData::typeId) &&
      this->model)
  {
    std::vector<bool> geomIdHasSensor(static_cast<size_t>(this->model->ngeom),
                                      false);
    _ecm.Each<components::Collision, components::ContactSensorData>(
        [&](const Entity &_entity, const components::Collision *,
            const components::ContactSensorData *) -> bool
        {
          auto idComp = _ecm.Component<MujocoGeomId>(_entity);
          if (idComp)
          {
            int id = idComp->Data();
            if (id >= 0 && id < static_cast<int>(geomIdHasSensor.size()))
            {
              geomIdHasSensor[static_cast<size_t>(id)] = true;
            }
          }
          return true;
        });

    std::unordered_map<Entity, msgs::Contacts> entityContactMap;

    for (int i = 0; i < this->data->ncon; ++i)
    {
      const mjContact *con = this->data->contact + i;

      if (con->geom[0] >= 0 &&
          con->geom[0] < static_cast<int>(this->geomIdToEntity.size()) &&
          con->geom[1] >= 0 &&
          con->geom[1] < static_cast<int>(this->geomIdToEntity.size()))
      {
        bool e1HasSensor = geomIdHasSensor[static_cast<size_t>(con->geom[0])];
        bool e2HasSensor = geomIdHasSensor[static_cast<size_t>(con->geom[1])];

        if (!e1HasSensor && !e2HasSensor)
        {
          continue;
        }

        Entity e1 = this->geomIdToEntity[static_cast<size_t>(con->geom[0])];
        Entity e2 = this->geomIdToEntity[static_cast<size_t>(con->geom[1])];

        if (e1 != kNullEntity && e2 != kNullEntity)
        {
          std::array<mjtNum, 6> contactForce;
          mj_contactForce(this->model, this->data, i, contactForce.data());

          math::Vector3d force =
              GetContactForce(con->frame, contactForce.data());
          math::Vector3d normal = GetVector(con->frame);
          math::Vector3d position = GetVector(con->pos);
          double depth = -con->dist;

          msgs::Contact contactMsg;
          contactMsg.mutable_collision1()->set_id(e1);
          contactMsg.mutable_collision2()->set_id(e2);

          SetVectorProto(contactMsg.add_position(), position);
          SetVectorProto(contactMsg.add_normal(), normal);
          contactMsg.add_depth(depth);

          auto *wrenchMsg = contactMsg.add_wrench();
          wrenchMsg->set_body_1_name(std::to_string(e1));
          wrenchMsg->set_body_2_name(std::to_string(e2));
          SetVectorProto(wrenchMsg->mutable_body_1_wrench()->mutable_force(),
                         force);

          if (e1HasSensor)
          {
            *entityContactMap[e1].add_contact() = contactMsg;
          }
          if (e2HasSensor)
          {
            *entityContactMap[e2].add_contact() = contactMsg;
          }
        }
      }
    }

    _ecm.Each<components::Collision, components::ContactSensorData>(
        [&](const Entity &_collEntity1, components::Collision *,
            components::ContactSensorData *_contacts) -> bool
        {
          auto it = entityContactMap.find(_collEntity1);
          if (it == entityContactMap.end())
          {
            if (_contacts->Data().contact_size() == 0)
            {
              return true;
            }
            msgs::Contacts emptyContacts;
            _contacts->SetData(emptyContacts,
                               [](const msgs::Contacts &,
                                  const msgs::Contacts &) { return false; });
            _ecm.SetChanged(_collEntity1, components::ContactSensorData::typeId,
                            ComponentState::PeriodicChange);
            return true;
          }

          _contacts->SetData(it->second,
                             [](const msgs::Contacts &, const msgs::Contacts &)
                             { return false; });
          _ecm.SetChanged(_collEntity1, components::ContactSensorData::typeId,
                          ComponentState::PeriodicChange);
          return true;
        });
  }

  // Process Sensors
  if (_ecm.HasComponentType(components::Imu::typeId))
  {
    _ecm.Each<components::Imu, components::ParentEntity>(
        [&](const Entity &_entity, const components::Imu *,
            const components::ParentEntity *parentComp) -> bool
        {
          Entity parentLink = parentComp->Data();
          auto bodyIdComp = _ecm.Component<MujocoBodyId>(parentLink);
          if (!bodyIdComp)
            return true;
          int mjBodyId = bodyIdComp->Data();

          int siteId = -1;
          for (int i = 0; i < this->model->nsite; ++i)
          {
            if (this->model->site_bodyid[i] == mjBodyId)
            {
              siteId = i;
              break;
            }
          }
          if (siteId >= 0)
          {
            int accelId = -1;
            int gyroId = -1;
            for (int i = 0; i < this->model->nsensor; ++i)
            {
              if (this->model->sensor_objtype[i] == mjOBJ_SITE &&
                  this->model->sensor_objid[i] == siteId)
              {
                if (this->model->sensor_type[i] == mjSENS_ACCELEROMETER)
                {
                  accelId = i;
                }
                else if (this->model->sensor_type[i] == mjSENS_GYRO)
                {
                  gyroId = i;
                }
              }
            }
            if (accelId >= 0 && gyroId >= 0)
            {
              int accelAdr = this->model->sensor_adr[accelId];
              int gyroAdr = this->model->sensor_adr[gyroId];

              math::Vector3d linAcc =
                  GetVector(this->data->sensordata + accelAdr);
              math::Vector3d angVel =
                  GetVector(this->data->sensordata + gyroAdr);

              auto accComp =
                  _ecm.Component<components::LinearAcceleration>(_entity);
              if (accComp)
                accComp->Data() = linAcc;
              else
                _ecm.CreateComponent(_entity,
                                     components::LinearAcceleration(linAcc));

              auto angComp =
                  _ecm.Component<components::AngularVelocity>(_entity);
              if (angComp)
                angComp->Data() = angVel;
              else
                _ecm.CreateComponent(_entity,
                                     components::AngularVelocity(angVel));

              _ecm.SetChanged(_entity, components::LinearAcceleration::typeId,
                              ComponentState::PeriodicChange);
              _ecm.SetChanged(_entity, components::AngularVelocity::typeId,
                              ComponentState::PeriodicChange);
            }
          }
          return true;
        });
  }

  if (_ecm.HasComponentType(components::ForceTorque::typeId))
  {
    _ecm.Each<components::ForceTorque, components::ParentEntity>(
        [&](const Entity &_entity, const components::ForceTorque *,
            const components::ParentEntity *parentComp) -> bool
        {
          Entity parentJoint = parentComp->Data();
          Entity childLink = kNullEntity;
          auto childLinkComp =
              _ecm.Component<components::ChildLinkName>(parentJoint);
          if (childLinkComp)
          {
            Entity modelEntity = _ecm.ParentEntity(parentJoint);
            childLink = _ecm.EntityByComponents(
                components::ParentEntity(modelEntity),
                components::Name(childLinkComp->Data()), components::Link());
          }
          if (childLink == kNullEntity)
            return true;
          auto bodyIdComp = _ecm.Component<MujocoBodyId>(childLink);
          if (!bodyIdComp)
            return true;
          int mjBodyId = bodyIdComp->Data();

          int siteId = -1;
          for (int i = 0; i < this->model->nsite; ++i)
          {
            if (this->model->site_bodyid[i] == mjBodyId)
            {
              siteId = i;
              break;
            }
          }
          if (siteId >= 0)
          {
            int forceId = -1;
            int torqueId = -1;
            for (int i = 0; i < this->model->nsensor; ++i)
            {
              if (this->model->sensor_objtype[i] == mjOBJ_SITE &&
                  this->model->sensor_objid[i] == siteId)
              {
                if (this->model->sensor_type[i] == mjSENS_FORCE)
                {
                  forceId = i;
                }
                else if (this->model->sensor_type[i] == mjSENS_TORQUE)
                {
                  torqueId = i;
                }
              }
            }
            if (forceId >= 0 && torqueId >= 0)
            {
              int forceAdr = this->model->sensor_adr[forceId];
              int torqueAdr = this->model->sensor_adr[torqueId];

              msgs::Wrench wrenchMsg;
              SetVectorProto(wrenchMsg.mutable_force(),
                             GetVector(this->data->sensordata + forceAdr));
              SetVectorProto(wrenchMsg.mutable_torque(),
                             GetVector(this->data->sensordata + torqueAdr));

              auto wrenchComp =
                  _ecm.Component<components::WrenchMeasured>(_entity);
              if (wrenchComp)
                wrenchComp->Data() = wrenchMsg;
              else
                _ecm.CreateComponent(_entity,
                                     components::WrenchMeasured(wrenchMsg));

              _ecm.SetChanged(_entity, components::WrenchMeasured::typeId,
                              ComponentState::PeriodicChange);
            }
          }
          return true;
        });
  }
}

void MujocoPhysics::ClearResetsAndCommands(EntityComponentManager &_ecm)
{
  // Clear / reset components
  RemoveComponentType<components::JointPositionReset>(_ecm);
  RemoveComponentType<components::JointVelocityReset>(_ecm);
  RemoveComponentType<components::WorldLinearVelocityReset>(_ecm);
  RemoveComponentType<components::WorldAngularVelocityReset>(_ecm);
  RemoveComponentType<components::EnableContactSurfaceCustomization>(_ecm);

  // Clear pending commands
  ClearComponentData<components::JointForceCmd>(_ecm);
  ClearComponentData<components::ExternalWorldWrenchCmd>(_ecm);
  ClearComponentData<components::JointPositionLimitsCmd>(_ecm);
  ClearComponentData<components::JointVelocityLimitsCmd>(_ecm);
  ClearComponentData<components::JointEffortLimitsCmd>(_ecm);

  RemoveComponentType<components::JointVelocityCmd>(_ecm);
  RemoveComponentType<components::AngularVelocityCmd>(_ecm);
  RemoveComponentType<components::LinearVelocityCmd>(_ecm);
}

void MujocoPhysics::Update(const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
{
  this->CreatePhysicsEntities(_ecm);
  this->UpdatePhysics(_ecm);
  this->Step(_info);
  this->UpdateSim(_info, _ecm);
}

GZ_ADD_PLUGIN(MujocoPhysics, System, MujocoPhysics::ISystemConfigure,
              MujocoPhysics::ISystemUpdate)
GZ_ADD_PLUGIN_ALIAS(MujocoPhysics, "gz::sim::systems::MujocoPhysics")

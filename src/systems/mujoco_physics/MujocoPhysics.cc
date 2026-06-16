#include "MujocoPhysics.hh"
#include "MujocoComponents.hh"

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <sdf/Mesh.hh>
#include <sdf/Cone.hh>
#include <sdf/Surface.hh>

#include <cmath>
#include <algorithm>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/WrenchMeasured.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/ChildLinkName.hh>

#include <unordered_map>
#include <vector>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>

#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include <mujoco/mujoco.h>

using namespace gz;
using namespace sim;
using namespace systems;
using namespace mujoco_physics;

class gz::sim::systems::mujoco_physics::MujocoPhysicsPrivate
{
  public: mjSpec* spec = nullptr;
  public: mjModel* model = nullptr;
  public: mjData* data = nullptr;

  public: ~MujocoPhysicsPrivate()
  {
    if (this->data)
      mj_deleteData(this->data);
    if (this->model)
      mj_deleteModel(this->model);
    if (this->spec)
      mj_deleteSpec(this->spec);
  }
};

MujocoPhysics::MujocoPhysics()
    : dataPtr(std::make_unique<MujocoPhysicsPrivate>())
{
}

MujocoPhysics::~MujocoPhysics() = default;

void MujocoPhysics::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
  // Initialize mjSpec
  this->dataPtr->spec = mj_makeSpec();
  
  // Extract gravity from ECM
  auto gravityComp = _ecm.Component<components::Gravity>(_entity);
  if (gravityComp)
  {
    auto gravity = gravityComp->Data();
    this->dataPtr->spec->option.gravity[0] = gravity.X();
    this->dataPtr->spec->option.gravity[1] = gravity.Y();
    this->dataPtr->spec->option.gravity[2] = gravity.Z();
  }

  // Compile the model initially
  this->dataPtr->model = mj_compile(this->dataPtr->spec, nullptr);
  if (this->dataPtr->model)
  {
    this->dataPtr->data = mj_makeData(this->dataPtr->model);
  }
  else
  {
    gzerr << "Failed to compile MuJoCo model in Configure\n";
  }
}

void MujocoPhysics::Update(const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  bool specChanged = false;
  mjsBody* worldBody = mjs_findBody(this->dataPtr->spec, "world");

  std::vector<Entity> newLinks;
  _ecm.EachNew<components::Link>(
    [&](const Entity &_entity, const components::Link *) -> bool
    {
      if (!_ecm.Component<MujocoBodyId>(_entity))
      {
        newLinks.push_back(_entity);
      }
      return true;
    });

  for (auto entity : newLinks)
  {
    std::string bodyName = "link_" + std::to_string(entity);
    mjsBody* newBody = mjs_addBody(worldBody, nullptr);
    if (newBody)
    {
      mjs_setName(newBody->element, bodyName.c_str());
      math::Pose3d wPose = gz::sim::worldPose(entity, _ecm);
      newBody->pos[0] = wPose.Pos().X();
      newBody->pos[1] = wPose.Pos().Y();
      newBody->pos[2] = wPose.Pos().Z();
      newBody->quat[0] = wPose.Rot().W();
      newBody->quat[1] = wPose.Rot().X();
      newBody->quat[2] = wPose.Rot().Y();
      newBody->quat[3] = wPose.Rot().Z();
      specChanged = true;
    }
  }

  std::vector<Entity> newCollisions;
  _ecm.EachNew<components::Collision>(
    [&](const Entity &_entity, const components::Collision *) -> bool
    {
      if (!_ecm.Component<MujocoGeomId>(_entity))
      {
        newCollisions.push_back(_entity);
      }
      return true;
    });

  for (auto entity : newCollisions)
  {
    auto parentComp = _ecm.Component<components::ParentEntity>(entity);
    mjsBody* parentBody = worldBody;
    if (parentComp) {
      std::string parentName = "link_" + std::to_string(parentComp->Data());
      mjsBody* foundBody = mjs_findBody(this->dataPtr->spec, parentName.c_str());
      if (foundBody) parentBody = foundBody;
    }
    mjsGeom* newGeom = mjs_addGeom(parentBody, nullptr);
    if (newGeom)
    {
      std::string geomName = "geom_" + std::to_string(entity);
      mjs_setName(newGeom->element, geomName.c_str());
      
      newGeom->condim = 3;
      auto _collisionComp = _ecm.Component<components::CollisionElement>(entity);
      if (_collisionComp && _collisionComp->Data().Surface() && _collisionComp->Data().Surface()->Friction() && _collisionComp->Data().Surface()->Friction()->ODE())
      {
        auto friction = _collisionComp->Data().Surface()->Friction()->ODE();
        newGeom->friction[0] = friction->Mu();
        newGeom->friction[1] = friction->Mu2();
      }
      
      auto poseComp = _ecm.Component<components::Pose>(entity);
      if (poseComp) {
        auto pose = poseComp->Data();
        newGeom->pos[0] = pose.Pos().X();
        newGeom->pos[1] = pose.Pos().Y();
        newGeom->pos[2] = pose.Pos().Z();
        newGeom->quat[0] = pose.Rot().W();
        newGeom->quat[1] = pose.Rot().X();
        newGeom->quat[2] = pose.Rot().Y();
        newGeom->quat[3] = pose.Rot().Z();
      }
      
      auto geomComp = _ecm.Component<components::Geometry>(entity);
      if (geomComp) {
        const sdf::Geometry& shape = geomComp->Data();
        switch (shape.Type())
        {
          case ::sdf::GeometryType::BOX:
          {
            newGeom->type = mjGEOM_BOX;
            for (int j = 0; j < 3; ++j)
              newGeom->size[j] = shape.BoxShape()->Size()[j] / 2.0;
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
            for (int j = 0; j < 3; ++j)
              newGeom->size[j] = shape.EllipsoidShape()->Radii()[j];
            break;
          }
          case ::sdf::GeometryType::CONE:
          {
            newGeom->type = mjGEOM_MESH;
            std::string meshName = "geom_" + std::to_string(entity) + "_cone";
            auto *muMesh = mjs_addMesh(this->dataPtr->spec, nullptr);
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
            std::string meshName = "geom_" + std::to_string(entity) + "_mesh";
            auto &meshManager = *gz::common::MeshManager::Instance();
            auto *mesh = meshManager.Load(meshSdf->Uri());
            if (mesh) {
                auto *muMesh = mjs_addMesh(this->dataPtr->spec, nullptr);
                mjs_setName(muMesh->element, meshName.c_str());
                mjs_setString(newGeom->meshname, meshName.c_str());
                muMesh->scale[0] = meshSdf->Scale().X();
                muMesh->scale[1] = meshSdf->Scale().Y();
                muMesh->scale[2] = meshSdf->Scale().Z();
                
                double *verts{nullptr};
                int *indices{nullptr};
                mesh->FillArrays(&verts, &indices);
                auto nverts = mesh->VertexCount();
                
                if (nverts > 0 && verts) {
                    muMesh->uservert->assign(3 * nverts, 0.0f);
                    std::transform(verts, verts + 3 * nverts, muMesh->uservert->begin(),
                        [](double val) {return static_cast<float>(val);});
                }
                if (mesh->IndexCount() > 0 && indices) {
                    muMesh->userface->assign(indices, indices + mesh->IndexCount());
                }
                
                delete[] verts;
                delete[] indices;
            } else {
                gzwarn << "Failed to load mesh: " << meshSdf->Uri() << "\n";
            }
            break;
          }
          default:
            break;
        }
      }
      specChanged = true;
    }
  }

  std::vector<Entity> newJoints;
  _ecm.EachNew<components::Joint, components::Name, components::JointType,
               components::ParentEntity, components::ChildLinkName>(
      [&](const Entity &_entity,
          const components::Joint *,
          const components::Name *,
          const components::JointType *,
          const components::ParentEntity *,
          const components::ChildLinkName *) -> bool
      {
        if (!_ecm.Component<MujocoJointId>(_entity))
        {
          newJoints.push_back(_entity);
        }
        return true;
      });

  for (auto entity : newJoints)
  {
    auto nameComp = _ecm.Component<components::Name>(entity);
    auto jointTypeComp = _ecm.Component<components::JointType>(entity);
    auto parentModelComp = _ecm.Component<components::ParentEntity>(entity);
    auto childLinkNameComp = _ecm.Component<components::ChildLinkName>(entity);

    Entity childLinkEntity = _ecm.EntityByComponents(
      components::Link(),
      components::ParentEntity(parentModelComp->Data()),
      components::Name(childLinkNameComp->Data()));

    if (childLinkEntity != kNullEntity)
    {
      std::string childName = "link_" + std::to_string(childLinkEntity);
      mjsBody* childBody = mjs_findBody(this->dataPtr->spec, childName.c_str());
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
          gzwarn << "Unsupported joint type for joint [" << nameComp->Data() << "]\n";
        }

        if (joint)
        {
          std::string mjJointName = "joint_" + std::to_string(entity);
          mjs_setName(joint->element, mjJointName.c_str());

          auto jointPoseComp = _ecm.Component<components::Pose>(entity);
          if (jointPoseComp)
          {
            auto jointPose = jointPoseComp->Data();
            joint->pos[0] = jointPose.Pos().X();
            joint->pos[1] = jointPose.Pos().Y();
            joint->pos[2] = jointPose.Pos().Z();
          }

          auto axisComp = _ecm.Component<components::JointAxis>(entity);
          if (axisComp && type != sdf::JointType::BALL)
          {
            auto axis = axisComp->Data();
            joint->axis[0] = axis.Xyz().X();
            joint->axis[1] = axis.Xyz().Y();
            joint->axis[2] = axis.Xyz().Z();
            
            joint->limited = !std::isinf(axis.Lower()) && !std::isinf(axis.Upper());
            if (joint->limited)
            {
              joint->range[0] = axis.Lower();
              joint->range[1] = axis.Upper();
            }
            joint->frictionloss = axis.Friction();
            joint->damping = axis.Damping();
          }

          actuator = mjs_addActuator(this->dataPtr->spec, nullptr);
          if (actuator)
          {
            actuator->trntype = mjTRN_JOINT;
            mjs_setString(actuator->target, mjJointName.c_str());
            std::string actName = "actuator_" + std::to_string(entity);
            mjs_setName(actuator->element, actName.c_str());
          }

          specChanged = true;
        }
      }
    }
  }

  _ecm.EachNew<components::Imu>(
    [&](const Entity &_entity, const components::Imu *) -> bool
    {
      Entity parentLink = _ecm.ParentEntity(_entity);
      if (parentLink != kNullEntity) {
          std::string linkName = "link_" + std::to_string(parentLink);
          auto *muBody = mjs_findBody(this->dataPtr->spec, linkName.c_str());
          if (muBody) {
              auto *muSite = mjs_addSite(muBody, nullptr);
              std::string siteName = "site_imu_" + std::to_string(_entity);
              mjs_setName(muSite->element, siteName.c_str());
              
              auto *accelSensor = mjs_addSensor(this->dataPtr->spec);
              accelSensor->type = mjSENS_ACCELEROMETER;
              accelSensor->objtype = mjOBJ_SITE;
              mjs_setString(accelSensor->objname, siteName.c_str());
              
              auto *gyroSensor = mjs_addSensor(this->dataPtr->spec);
              gyroSensor->type = mjSENS_GYRO;
              gyroSensor->objtype = mjOBJ_SITE;
              mjs_setString(gyroSensor->objname, siteName.c_str());
              specChanged = true;
          }
      }
      return true;
    });

  _ecm.EachNew<components::ForceTorque>(
    [&](const Entity &_entity, const components::ForceTorque *) -> bool
    {
      Entity parentJoint = _ecm.ParentEntity(_entity);
      if (parentJoint != kNullEntity) {
          auto childLinkComp = _ecm.Component<components::ChildLinkName>(parentJoint);
          if (childLinkComp) {
              std::string childName = childLinkComp->Data();
              Entity modelEntity = _ecm.ParentEntity(parentJoint);
              Entity childLinkEntity = _ecm.EntityByComponents(components::ParentEntity(modelEntity), components::Name(childName), components::Link());
              if (childLinkEntity != kNullEntity) {
                  std::string linkName = "link_" + std::to_string(childLinkEntity);
                  auto *muBody = mjs_findBody(this->dataPtr->spec, linkName.c_str());
                  if (muBody) {
                      auto *muSite = mjs_addSite(muBody, nullptr);
                      std::string siteName = "site_ft_" + std::to_string(_entity);
                      mjs_setName(muSite->element, siteName.c_str());
                      
                      auto *forceSensor = mjs_addSensor(this->dataPtr->spec);
                      forceSensor->type = mjSENS_FORCE;
                      forceSensor->objtype = mjOBJ_SITE;
                      mjs_setString(forceSensor->objname, siteName.c_str());
                      
                      auto *torqueSensor = mjs_addSensor(this->dataPtr->spec);
                      torqueSensor->type = mjSENS_TORQUE;
                      torqueSensor->objtype = mjOBJ_SITE;
                      mjs_setString(torqueSensor->objname, siteName.c_str());
                      specChanged = true;
                  }
              }
          }
      }
      return true;
    });

  if (specChanged)
  {
    if (this->dataPtr->data)
      mj_deleteData(this->dataPtr->data);
    if (this->dataPtr->model)
      mj_deleteModel(this->dataPtr->model);

    this->dataPtr->model = mj_compile(this->dataPtr->spec, nullptr);
    if (this->dataPtr->model)
    {
      this->dataPtr->data = mj_makeData(this->dataPtr->model);

      for (auto entity : newLinks)
      {
        std::string bodyName = "link_" + std::to_string(entity);
        int id = mj_name2id(this->dataPtr->model, mjOBJ_BODY, bodyName.c_str());
        if (id >= 0)
          _ecm.CreateComponent(entity, MujocoBodyId(id));
      }
      for (auto entity : newCollisions)
      {
        std::string geomName = "geom_" + std::to_string(entity);
        int id = mj_name2id(this->dataPtr->model, mjOBJ_GEOM, geomName.c_str());
        if (id >= 0)
          _ecm.CreateComponent(entity, MujocoGeomId(id));
      }
      for (auto entity : newJoints)
      {
        std::string jointName = "joint_" + std::to_string(entity);
        int id = mj_name2id(this->dataPtr->model, mjOBJ_JOINT, jointName.c_str());
        if (id >= 0)
          _ecm.CreateComponent(entity, MujocoJointId(id));

        std::string actName = "actuator_" + std::to_string(entity);
        int actId = mj_name2id(this->dataPtr->model, mjOBJ_ACTUATOR, actName.c_str());
        if (actId >= 0)
          _ecm.CreateComponent(entity, MujocoActuatorId(actId));
      }
    }
    else
    {
      gzerr << "Failed to recompile MuJoCo model after adding entities.\n";
    }
  }

  if (this->dataPtr->model && this->dataPtr->data)
  {
    // Clear actuator controls
    if (this->dataPtr->model->nu > 0)
    {
      std::fill(this->dataPtr->data->ctrl,
                this->dataPtr->data->ctrl + this->dataPtr->model->nu,
                0.0);
    }

    // Apply JointForceCmd and JointVelocityCmd
    _ecm.Each<components::Joint, MujocoActuatorId>(
      [&](const Entity &entity,
          const components::Joint *,
          const MujocoActuatorId *_actuatorIdComp) -> bool
      {
        auto velCmd = _ecm.Component<components::JointVelocityCmd>(entity);
        auto forceCmd = _ecm.Component<components::JointForceCmd>(entity);

        if (forceCmd || velCmd)
        {
          int actId = _actuatorIdComp->Data();
          if (actId >= 0 && actId < this->dataPtr->model->nu)
          {
            if (forceCmd && !forceCmd->Data().empty())
            {
              this->dataPtr->data->ctrl[actId] = forceCmd->Data()[0];
            }
            else if (velCmd && !velCmd->Data().empty())
            {
              this->dataPtr->data->ctrl[actId] = velCmd->Data()[0];
            }
          }
        }
        return true;
      });

    // Clear applied forces
    std::fill(this->dataPtr->data->xfrc_applied,
              this->dataPtr->data->xfrc_applied + 6 * this->dataPtr->model->nbody,
              0.0);

    // Apply ExternalWorldWrenchCmd
    _ecm.Each<components::Link, MujocoBodyId, components::ExternalWorldWrenchCmd>(
      [&](const Entity &,
          const components::Link *,
          const MujocoBodyId *_bodyIdComp,
          const components::ExternalWorldWrenchCmd *_wrenchComp) -> bool
      {
        int mjBodyId = _bodyIdComp->Data();
        if (mjBodyId > 0 && mjBodyId < this->dataPtr->model->nbody)
        {
          auto wrench = _wrenchComp->Data();
          this->dataPtr->data->xfrc_applied[6 * mjBodyId + 0] = wrench.torque().x();
          this->dataPtr->data->xfrc_applied[6 * mjBodyId + 1] = wrench.torque().y();
          this->dataPtr->data->xfrc_applied[6 * mjBodyId + 2] = wrench.torque().z();
          this->dataPtr->data->xfrc_applied[6 * mjBodyId + 3] = wrench.force().x();
          this->dataPtr->data->xfrc_applied[6 * mjBodyId + 4] = wrench.force().y();
          this->dataPtr->data->xfrc_applied[6 * mjBodyId + 5] = wrench.force().z();
        }
        return true;
      });

    // Step the simulation
    mj_step(this->dataPtr->model, this->dataPtr->data);

    // Synchronize the ECM components::Pose of all links
    // NOTE: The implementation plan suggested doing this in PostUpdate, but 
    // PostUpdate provides a const EntityComponentManager, so mutating components
    // there is impossible and thread-unsafe. It must be done in Update.
    _ecm.Each<components::Link, MujocoBodyId, components::Pose, components::ParentEntity>(
      [&](const Entity &entity,
          const components::Link *,
          const MujocoBodyId *_bodyIdComp,
          components::Pose *_poseComp,
          const components::ParentEntity *_parentComp) -> bool
      {
        int mjBodyId = _bodyIdComp->Data();
        if (mjBodyId > 0 && mjBodyId < this->dataPtr->model->nbody)
        {
          // mjData->xpos is [nbody x 3], mjData->xquat is [nbody x 4] (w,x,y,z)
          mjtNum* pos = this->dataPtr->data->xpos + 3 * mjBodyId;
          mjtNum* quat = this->dataPtr->data->xquat + 4 * mjBodyId;

          math::Pose3d worldPose(
            pos[0], pos[1], pos[2],
            quat[0], quat[1], quat[2], quat[3] // math::Quaternion(w, x, y, z)
          );
          
          math::Pose3d parentWorldPose = gz::sim::worldPose(_parentComp->Data(), _ecm);
          _poseComp->Data() = parentWorldPose.Inverse() * worldPose;

          mjtNum velocity[6];
          mj_objectVelocity(this->dataPtr->model, this->dataPtr->data, mjOBJ_BODY, mjBodyId, velocity, 0);
          math::Vector3d worldAngularVel(velocity[0], velocity[1], velocity[2]);
          math::Vector3d worldLinearVel(velocity[3], velocity[4], velocity[5]);

          auto linVelComp = _ecm.Component<components::LinearVelocity>(entity);
          if (linVelComp) {
            linVelComp->Data() = worldPose.Rot().Inverse() * worldLinearVel;
          } else {
            _ecm.CreateComponent(entity, components::LinearVelocity(worldPose.Rot().Inverse() * worldLinearVel));
          }

          auto angVelComp = _ecm.Component<components::AngularVelocity>(entity);
          if (angVelComp) {
            angVelComp->Data() = worldPose.Rot().Inverse() * worldAngularVel;
          } else {
            _ecm.CreateComponent(entity, components::AngularVelocity(worldPose.Rot().Inverse() * worldAngularVel));
          }
        }
        return true;
      });

    // Process contacts
    if (_ecm.HasComponentType(components::ContactSensorData::typeId)) {
      std::unordered_map<int, Entity> geomToEntity;
      _ecm.Each<MujocoGeomId>([&](const Entity &_entity, const MujocoGeomId *_idComp) -> bool {
          geomToEntity[_idComp->Data()] = _entity;
          return true;
      });

      std::unordered_map<Entity, std::unordered_map<Entity, std::vector<msgs::Contact>>> entityContactMap;
      
      for (int i = 0; i < this->dataPtr->data->ncon; ++i) {
          const mjContact *con = this->dataPtr->data->contact + i;
          
          auto it1 = geomToEntity.find(con->geom[0]);
          auto it2 = geomToEntity.find(con->geom[1]);
          
          if (it1 != geomToEntity.end() && it2 != geomToEntity.end()) {
              Entity e1 = it1->second;
              Entity e2 = it2->second;
              
              mjtNum contactForce[6];
              mj_contactForce(this->dataPtr->model, this->dataPtr->data, i, contactForce);
              
              math::Vector3d force(
                  (con->frame[0] * contactForce[0] + con->frame[3] * contactForce[1] + con->frame[6] * contactForce[2]),
                  (con->frame[1] * contactForce[0] + con->frame[4] * contactForce[1] + con->frame[7] * contactForce[2]),
                  (con->frame[2] * contactForce[0] + con->frame[5] * contactForce[1] + con->frame[8] * contactForce[2])
              );
              math::Vector3d normal(con->frame[0], con->frame[1], con->frame[2]);
              math::Vector3d position(con->pos[0], con->pos[1], con->pos[2]);
              double depth = -con->dist;
              
              msgs::Contact contactMsg;
              contactMsg.mutable_collision1()->set_id(e1);
              contactMsg.mutable_collision2()->set_id(e2);
              
              auto *posMsg = contactMsg.add_position();
              posMsg->set_x(position.X()); posMsg->set_y(position.Y()); posMsg->set_z(position.Z());
              
              auto *normalMsg = contactMsg.add_normal();
              normalMsg->set_x(normal.X()); normalMsg->set_y(normal.Y()); normalMsg->set_z(normal.Z());
              
              contactMsg.add_depth(depth);
              
              auto *wrenchMsg = contactMsg.add_wrench();
              wrenchMsg->set_body_1_name(std::to_string(e1));
              wrenchMsg->set_body_2_name(std::to_string(e2));
              auto *forceMsg = wrenchMsg->mutable_body_1_wrench()->mutable_force();
              forceMsg->set_x(force.X()); forceMsg->set_y(force.Y()); forceMsg->set_z(force.Z());
              
              entityContactMap[e1][e2].push_back(contactMsg);
              entityContactMap[e2][e1].push_back(contactMsg);
          }
      }

      _ecm.Each<components::Collision, components::ContactSensorData>(
          [&](const Entity &_collEntity1, components::Collision *,
              components::ContactSensorData *_contacts) -> bool
          {
              msgs::Contacts contactsComp;
              if (entityContactMap.find(_collEntity1) == entityContactMap.end()) {
                  _contacts->SetData(contactsComp, [](const msgs::Contacts &, const msgs::Contacts &) { return false; });
                  _ecm.SetChanged(_collEntity1, components::ContactSensorData::typeId, ComponentState::PeriodicChange);
                  return true;
              }
              
              const auto &contactMap = entityContactMap[_collEntity1];
              for (const auto &[collEntity2, contactList] : contactMap) {
                  for (const auto &contactMsg : contactList) {
                      *contactsComp.add_contact() = contactMsg;
                  }
              }
              
              _contacts->SetData(contactsComp, [](const msgs::Contacts &, const msgs::Contacts &) { return false; });
              _ecm.SetChanged(_collEntity1, components::ContactSensorData::typeId, ComponentState::PeriodicChange);
              return true;
          });
    }

    // Process Sensors
    _ecm.Each<components::Imu>(
      [&](const Entity &_entity, const components::Imu *) -> bool
      {
        std::string siteName = "site_imu_" + std::to_string(_entity);
        int siteId = mj_name2id(this->dataPtr->model, mjOBJ_SITE, siteName.c_str());
        if (siteId >= 0) {
            int accelId = -1;
            int gyroId = -1;
            for (int i = 0; i < this->dataPtr->model->nsensor; ++i) {
                if (this->dataPtr->model->sensor_objtype[i] == mjOBJ_SITE &&
                    this->dataPtr->model->sensor_objid[i] == siteId) {
                    if (this->dataPtr->model->sensor_type[i] == mjSENS_ACCELEROMETER) {
                        accelId = i;
                    } else if (this->dataPtr->model->sensor_type[i] == mjSENS_GYRO) {
                        gyroId = i;
                    }
                }
            }
            if (accelId >= 0 && gyroId >= 0) {
                int accelAdr = this->dataPtr->model->sensor_adr[accelId];
                int gyroAdr = this->dataPtr->model->sensor_adr[gyroId];
                
                math::Vector3d linAcc(
                    this->dataPtr->data->sensordata[accelAdr],
                    this->dataPtr->data->sensordata[accelAdr+1],
                    this->dataPtr->data->sensordata[accelAdr+2]
                );
                
                math::Vector3d angVel(
                    this->dataPtr->data->sensordata[gyroAdr],
                    this->dataPtr->data->sensordata[gyroAdr+1],
                    this->dataPtr->data->sensordata[gyroAdr+2]
                );
                
                auto accComp = _ecm.Component<components::LinearAcceleration>(_entity);
                if (accComp) accComp->Data() = linAcc;
                else _ecm.CreateComponent(_entity, components::LinearAcceleration(linAcc));
                
                auto angComp = _ecm.Component<components::AngularVelocity>(_entity);
                if (angComp) angComp->Data() = angVel;
                // Note: The previous logic might have already created components::AngularVelocity for the entity if it is a Link, but IMU is usually a separate entity.
                else _ecm.CreateComponent(_entity, components::AngularVelocity(angVel));
                
                _ecm.SetChanged(_entity, components::LinearAcceleration::typeId, ComponentState::PeriodicChange);
                _ecm.SetChanged(_entity, components::AngularVelocity::typeId, ComponentState::PeriodicChange);
            }
        }
        return true;
      });

    _ecm.Each<components::ForceTorque>(
      [&](const Entity &_entity, const components::ForceTorque *) -> bool
      {
        std::string siteName = "site_ft_" + std::to_string(_entity);
        int siteId = mj_name2id(this->dataPtr->model, mjOBJ_SITE, siteName.c_str());
        if (siteId >= 0) {
            int forceId = -1;
            int torqueId = -1;
            for (int i = 0; i < this->dataPtr->model->nsensor; ++i) {
                if (this->dataPtr->model->sensor_objtype[i] == mjOBJ_SITE &&
                    this->dataPtr->model->sensor_objid[i] == siteId) {
                    if (this->dataPtr->model->sensor_type[i] == mjSENS_FORCE) {
                        forceId = i;
                    } else if (this->dataPtr->model->sensor_type[i] == mjSENS_TORQUE) {
                        torqueId = i;
                    }
                }
            }
            if (forceId >= 0 && torqueId >= 0) {
                int forceAdr = this->dataPtr->model->sensor_adr[forceId];
                int torqueAdr = this->dataPtr->model->sensor_adr[torqueId];
                
                msgs::Wrench wrenchMsg;
                auto *forceMsg = wrenchMsg.mutable_force();
                forceMsg->set_x(this->dataPtr->data->sensordata[forceAdr]);
                forceMsg->set_y(this->dataPtr->data->sensordata[forceAdr+1]);
                forceMsg->set_z(this->dataPtr->data->sensordata[forceAdr+2]);
                
                auto *torqueMsg = wrenchMsg.mutable_torque();
                torqueMsg->set_x(this->dataPtr->data->sensordata[torqueAdr]);
                torqueMsg->set_y(this->dataPtr->data->sensordata[torqueAdr+1]);
                torqueMsg->set_z(this->dataPtr->data->sensordata[torqueAdr+2]);
                
                auto wrenchComp = _ecm.Component<components::WrenchMeasured>(_entity);
                if (wrenchComp) wrenchComp->Data() = wrenchMsg;
                else _ecm.CreateComponent(_entity, components::WrenchMeasured(wrenchMsg));
                
                _ecm.SetChanged(_entity, components::WrenchMeasured::typeId, ComponentState::PeriodicChange);
            }
        }
        return true;
      });

  }
}

void MujocoPhysics::PostUpdate(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm)
{
  // Left intentionally blank. The ECM synchronization logic has been moved to 
  // Update() to respect the constness of the ECM here.
}

GZ_ADD_PLUGIN(MujocoPhysics,
              System,
              MujocoPhysics::ISystemConfigure,
              MujocoPhysics::ISystemUpdate,
              MujocoPhysics::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(MujocoPhysics, "gz::sim::systems::MujocoPhysics")

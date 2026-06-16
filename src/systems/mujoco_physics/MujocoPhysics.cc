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
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
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

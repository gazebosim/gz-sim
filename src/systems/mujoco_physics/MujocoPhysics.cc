#include "MujocoPhysics.hh"
#include "MujocoComponents.hh"

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <queue>
#include <set>
#include <map>
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
#include <gz/sim/components/Inertial.hh>
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

#include <gz/sim/components/AngularVelocityReset.hh>
#include <gz/sim/components/JointEffortLimitsCmd.hh>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityLimitsCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearVelocityReset.hh>

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
  this->spec->option.enableflags |= mjtEnableBit::mjENBL_SLEEP;
  
  // Extract gravity from ECM
  auto gravityComp = _ecm.Component<components::Gravity>(_entity);
  if (gravityComp)
  {
    auto gravity = gravityComp->Data();
    this->spec->option.gravity[0] = gravity.X();
    this->spec->option.gravity[1] = gravity.Y();
    this->spec->option.gravity[2] = gravity.Z();
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
  bool specChanged = false;
  mjsBody* worldBody = mjs_findBody(this->spec, "world");
  if (!worldBody) {
    gzerr << "worldBody is null! Try looking for worldbody or default\n";
    // We will just pass nullptr as parent if it's null, which might attach to root
  }

  std::unordered_map<Entity, mjsBody*> entityToMujocoBody;
  
  std::vector<std::pair<Entity, std::string>> newModels;
  _ecm.EachNew<components::Model, components::Name>(
    [&](const Entity &_entity, const components::Model *, const components::Name *_nameComp) -> bool
    {
      newModels.push_back({_entity, _nameComp->Data()});
      return true;
    });

  for (auto const& [modelEntity, modelName] : newModels)
  {
    gzdbg << "Processing new model entity: " << modelEntity << "\n";
    bool isStatic = false;
    auto staticComp = _ecm.Component<components::Static>(modelEntity);
    if (staticComp && staticComp->Data())
    {
      isStatic = true;
    }

    std::vector<Entity> modelLinks;
    _ecm.Each<components::Link, components::ParentEntity>(
      [&](const Entity &_linkEntity, const components::Link *, const components::ParentEntity *_parent) -> bool
      {
        if (_parent->Data() == modelEntity)
          modelLinks.push_back(_linkEntity);
        return true;
      });

    gzdbg << "  Model " << modelEntity << " found " << modelLinks.size() << " links.\n";

    std::vector<Entity> modelJoints;
    _ecm.Each<components::Joint, components::ParentEntity>(
      [&](const Entity &_jointEntity, const components::Joint *, const components::ParentEntity *_parent) -> bool
      {
        if (_parent->Data() == modelEntity)
          modelJoints.push_back(_jointEntity);
        return true;
      });

    std::set<Entity> allLinks(modelLinks.begin(), modelLinks.end());
    std::set<Entity> childLinks;
    std::map<Entity, Entity> parentMap; // child -> parent link

    for (auto jointEntity : modelJoints)
    {
      auto childNameComp = _ecm.Component<components::ChildLinkName>(jointEntity);
      auto parentNameComp = _ecm.Component<components::ParentLinkName>(jointEntity);
      if (childNameComp && parentNameComp)
      {
        Entity childLinkEntity = _ecm.EntityByComponents(
          components::Link(),
          components::ParentEntity(modelEntity),
          components::Name(childNameComp->Data()));
        Entity parentLinkEntity = _ecm.EntityByComponents(
          components::Link(),
          components::ParentEntity(modelEntity),
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
      std::string linkName = nameComp ? nameComp->Data() : std::to_string(rootLink);
      std::string bodyName = modelName + "::" + linkName;
      gzdbg << "  mjs_addBody for root link: " << bodyName << "\n";
      mjsBody* newBody = mjs_addBody(worldBody, nullptr);
      if (newBody)
      {
        gzdbg << "  Successfully added newBody for " << bodyName << "\n";
        mjs_setName(newBody->element, bodyName.c_str());
        entityToMujocoBody[rootLink] = newBody;
        math::Pose3d wPose = gz::sim::worldPose(rootLink, _ecm);
        newBody->pos[0] = wPose.Pos().X();
        newBody->pos[1] = wPose.Pos().Y();
        newBody->pos[2] = wPose.Pos().Z();
        newBody->quat[0] = wPose.Rot().W();
        newBody->quat[1] = wPose.Rot().X();
        newBody->quat[2] = wPose.Rot().Y();
        newBody->quat[3] = wPose.Rot().Z();
        specChanged = true;

        auto inertialComp = _ecm.Component<components::Inertial>(rootLink);
        if (inertialComp)
        {
          const auto &inertial = inertialComp->Data();
          newBody->explicitinertial = 1;
          newBody->mass = inertial.MassMatrix().Mass();
          
          newBody->fullinertia[0] = inertial.MassMatrix().Ixx();
          newBody->fullinertia[1] = inertial.MassMatrix().Iyy();
          newBody->fullinertia[2] = inertial.MassMatrix().Izz();
          newBody->fullinertia[3] = inertial.MassMatrix().Ixy();
          newBody->fullinertia[4] = inertial.MassMatrix().Ixz();
          newBody->fullinertia[5] = inertial.MassMatrix().Iyz();
          
          newBody->ipos[0] = inertial.Pose().Pos().X();
          newBody->ipos[1] = inertial.Pose().Pos().Y();
          newBody->ipos[2] = inertial.Pose().Pos().Z();
          newBody->iquat[0] = inertial.Pose().Rot().W();
          newBody->iquat[1] = inertial.Pose().Rot().X();
          newBody->iquat[2] = inertial.Pose().Rot().Y();
          newBody->iquat[3] = inertial.Pose().Rot().Z();
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
      std::string parentLinkName = parentNameComp ? parentNameComp->Data() : std::to_string(parentLink);
      std::string parentBodyName = modelName + "::" + parentLinkName;
      mjsBody* parentBody = mjs_findBody(this->spec, parentBodyName.c_str());

      for (auto childLink : childrenMap[parentLink])
      {
        auto childNameComp = _ecm.Component<components::Name>(childLink);
        std::string childLinkName = childNameComp ? childNameComp->Data() : std::to_string(childLink);
        std::string childBodyName = modelName + "::" + childLinkName;
        gzdbg << "  mjs_addBody for child link: " << childBodyName << "\n";
        mjsBody* childBody = mjs_addBody(parentBody, nullptr);
        if (childBody)
        {
          mjs_setName(childBody->element, childBodyName.c_str());
          entityToMujocoBody[childLink] = childBody;
          
          math::Pose3d parentWorldPose = gz::sim::worldPose(parentLink, _ecm);
          math::Pose3d childWorldPose = gz::sim::worldPose(childLink, _ecm);
          math::Pose3d relativePose = parentWorldPose.Inverse() * childWorldPose;

          childBody->pos[0] = relativePose.Pos().X();
          childBody->pos[1] = relativePose.Pos().Y();
          childBody->pos[2] = relativePose.Pos().Z();
          childBody->quat[0] = relativePose.Rot().W();
          childBody->quat[1] = relativePose.Rot().X();
          childBody->quat[2] = relativePose.Rot().Y();
          childBody->quat[3] = relativePose.Rot().Z();
          specChanged = true;
          
          auto inertialComp = _ecm.Component<components::Inertial>(childLink);
          if (inertialComp)
          {
            const auto &inertial = inertialComp->Data();
            childBody->explicitinertial = 1;
            childBody->mass = inertial.MassMatrix().Mass();
            
            childBody->fullinertia[0] = inertial.MassMatrix().Ixx();
            childBody->fullinertia[1] = inertial.MassMatrix().Iyy();
            childBody->fullinertia[2] = inertial.MassMatrix().Izz();
            childBody->fullinertia[3] = inertial.MassMatrix().Ixy();
            childBody->fullinertia[4] = inertial.MassMatrix().Ixz();
            childBody->fullinertia[5] = inertial.MassMatrix().Iyz();
            
            childBody->ipos[0] = inertial.Pose().Pos().X();
            childBody->ipos[1] = inertial.Pose().Pos().Y();
            childBody->ipos[2] = inertial.Pose().Pos().Z();
            childBody->iquat[0] = inertial.Pose().Rot().W();
            childBody->iquat[1] = inertial.Pose().Rot().X();
            childBody->iquat[2] = inertial.Pose().Rot().Y();
            childBody->iquat[3] = inertial.Pose().Rot().Z();
          }
          
          q.push(childLink);
        }
      }
    }
  }

  _ecm.EachNew<components::Collision, components::Name, components::ParentEntity>(
    [&](const Entity &entity, const components::Collision *, const components::Name *nameComp, const components::ParentEntity *parentComp) -> bool
    {
    mjsBody* parentBody = worldBody;
    auto linkNameComp = _ecm.Component<components::Name>(parentComp->Data());
    std::string linkName = linkNameComp ? linkNameComp->Data() : std::to_string(parentComp->Data());
    Entity modelEntity = _ecm.ParentEntity(parentComp->Data());
    auto modelNameComp = _ecm.Component<components::Name>(modelEntity);
    std::string modelName = modelNameComp ? modelNameComp->Data() : std::to_string(modelEntity);
    std::string parentName = modelName + "::" + linkName;
    mjsBody *foundBody = nullptr;
    if (entityToMujocoBody.find(parentComp->Data()) != entityToMujocoBody.end())
      foundBody = entityToMujocoBody[parentComp->Data()];
    else
      foundBody = mjs_findBody(this->spec, parentName.c_str());

    if (foundBody)
      parentBody = foundBody;
    else
      gzerr << "Could not find parent body " << parentName << " for collision entity " << entity << "\n";

    gzdbg << "Collision entity: " << entity << " searching for parent bodyName: " << parentName << ", found: " << foundBody << "\n";
    mjsGeom* newGeom = mjs_addGeom(parentBody, nullptr);
    if (newGeom)
    {
      std::string geomName = modelName + "::" + linkName + "::" + nameComp->Data();
      mjs_setName(newGeom->element, geomName.c_str());
      
      newGeom->condim = 3;
      auto _collisionComp = _ecm.Component<components::CollisionElement>(entity);
      if (_collisionComp && _collisionComp->Data().Surface() && _collisionComp->Data().Surface()->Friction() && _collisionComp->Data().Surface()->Friction()->ODE())
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
          newGeom->contype = sdfCollision.Surface()->Contact()->CategoryBitmask().value_or(1);
          newGeom->conaffinity = sdfCollision.Surface()->Contact()->CollideBitmask();
        }
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
            if (mesh) {
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
    return true;
  });

  _ecm.EachNew<components::Joint, components::Name, components::JointType,
               components::ParentEntity, components::ChildLinkName>(
      [&](const Entity &entity,
          const components::Joint *,
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
    std::string modelName = modelNameComp ? modelNameComp->Data() : std::to_string(modelEntity);
    std::string childName = modelName + "::" + childLinkNameComp->Data();
      mjsBody* childBody = nullptr;
      if (entityToMujocoBody.find(childLinkEntity) != entityToMujocoBody.end())
        childBody = entityToMujocoBody[childLinkEntity];
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
          gzwarn << "Unsupported joint type for joint [" << nameComp->Data() << "]\n";
        }

        if (joint)
        {
          std::string mjJointName = modelName + "::" + nameComp->Data();
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

          specChanged = true;
        }
      }
    }
    return true;
  });

  _ecm.EachNew<components::Imu, components::ParentEntity>(
    [&](const Entity &_entity, const components::Imu *, const components::ParentEntity *parentComp) -> bool
    {
      Entity parentLink = parentComp->Data();
      if (parentLink != kNullEntity) {
          auto linkNameComp = _ecm.Component<components::Name>(parentLink);
          std::string linkName = linkNameComp ? linkNameComp->Data() : std::to_string(parentLink);
          Entity modelEntity = _ecm.ParentEntity(parentLink);
          auto modelNameComp = _ecm.Component<components::Name>(modelEntity);
          std::string modelName = modelNameComp ? modelNameComp->Data() : std::to_string(modelEntity);
          std::string fullLinkName = modelName + "::" + linkName;
          mjsBody* muBody = nullptr;
          if (entityToMujocoBody.find(parentLink) != entityToMujocoBody.end())
            muBody = entityToMujocoBody[parentLink];
          else
            muBody = mjs_findBody(this->spec, fullLinkName.c_str());

          if (muBody) {
              auto *muSite = mjs_addSite(muBody, nullptr);
              
              auto *accelSensor = mjs_addSensor(this->spec);
              accelSensor->type = mjSENS_ACCELEROMETER;
              accelSensor->objtype = mjOBJ_SITE;
              
              auto *gyroSensor = mjs_addSensor(this->spec);
              gyroSensor->type = mjSENS_GYRO;
              gyroSensor->objtype = mjOBJ_SITE;
              specChanged = true;
          }
      }
      return true;
    });

  _ecm.EachNew<components::ForceTorque, components::ParentEntity>(
    [&](const Entity &_entity, const components::ForceTorque *, const components::ParentEntity *parentComp) -> bool
    {
      Entity parentJoint = parentComp->Data();
      if (parentJoint != kNullEntity) {
          auto childLinkComp = _ecm.Component<components::ChildLinkName>(parentJoint);
          if (childLinkComp) {
              std::string childName = childLinkComp->Data();
              Entity modelEntity = _ecm.ParentEntity(parentJoint);
              Entity childLinkEntity = _ecm.EntityByComponents(components::ParentEntity(modelEntity), components::Name(childName), components::Link());
              if (childLinkEntity != kNullEntity) {
                  auto linkNameComp = _ecm.Component<components::Name>(childLinkEntity);
                  std::string linkName = linkNameComp ? linkNameComp->Data() : std::to_string(childLinkEntity);
                  auto modelNameComp = _ecm.Component<components::Name>(modelEntity);
                  std::string modelName = modelNameComp ? modelNameComp->Data() : std::to_string(modelEntity);
                  std::string fullLinkName = modelName + "::" + linkName;
                  mjsBody* muBody = nullptr;
                  if (entityToMujocoBody.find(childLinkEntity) != entityToMujocoBody.end())
                    muBody = entityToMujocoBody[childLinkEntity];
                  else
                    muBody = mjs_findBody(this->spec, fullLinkName.c_str());

                  if (muBody) {
                      auto *muSite = mjs_addSite(muBody, nullptr);
                      
                      auto *forceSensor = mjs_addSensor(this->spec);
                      forceSensor->type = mjSENS_FORCE;
                      forceSensor->objtype = mjOBJ_SITE;
                      
                      auto *torqueSensor = mjs_addSensor(this->spec);
                      torqueSensor->type = mjSENS_TORQUE;
                      torqueSensor->objtype = mjOBJ_SITE;
                      specChanged = true;
                  }
              }
          }
      }
      return true;
    });

  if (specChanged)
  {
    mj_recompile(this->spec, nullptr, this->model, this->data);
    gzdbg << "Recompiled:\n";
    mj_saveXML(this->spec, "/tmp/mujoco_model.xml", nullptr, 0);

    _ecm.Each<components::Link, components::Name, components::ParentEntity>(
      [&](const Entity &_entity, const components::Link *, const components::Name *nameComp, const components::ParentEntity *parentComp) -> bool
      {
        auto modelNameComp = _ecm.Component<components::Name>(parentComp->Data());
        std::string modelName = modelNameComp ? modelNameComp->Data() : std::to_string(parentComp->Data());
        std::string bodyName = modelName + "::" + nameComp->Data();
        int id = mj_name2id(this->model, mjOBJ_BODY, bodyName.c_str());
        if (id >= 0)
        {
          gzdbg << "New id for:" << _entity << " " << bodyName << " " << id << "\n";
          _ecm.SetComponentData<MujocoBodyId>(_entity, id);
        }
        return true;
      });

    _ecm.Each<components::Collision, components::Name, components::ParentEntity>(
      [&](const Entity &_entity, const components::Collision *, const components::Name *nameComp, const components::ParentEntity *parentComp) -> bool
      {
        auto linkNameComp = _ecm.Component<components::Name>(parentComp->Data());
        std::string linkName = linkNameComp ? linkNameComp->Data() : std::to_string(parentComp->Data());
        Entity modelEntity = _ecm.ParentEntity(parentComp->Data());
        auto modelNameComp = _ecm.Component<components::Name>(modelEntity);
        std::string modelName = modelNameComp ? modelNameComp->Data() : std::to_string(modelEntity);
        std::string geomName = modelName + "::" + linkName + "::" + nameComp->Data();
        int id = mj_name2id(this->model, mjOBJ_GEOM, geomName.c_str());
        if (id >= 0)
        {
          _ecm.SetComponentData<MujocoGeomId>(_entity, id);
        }
        return true;
      });

    _ecm.Each<components::Joint, components::Name, components::ParentEntity>(
      [&](const Entity &_entity, const components::Joint *, const components::Name *nameComp, const components::ParentEntity *parentComp) -> bool
      {
        auto modelNameComp = _ecm.Component<components::Name>(parentComp->Data());
        std::string modelName = modelNameComp ? modelNameComp->Data() : std::to_string(parentComp->Data());
        std::string jointName = modelName + "::" + nameComp->Data();
        int id = mj_name2id(this->model, mjOBJ_JOINT, jointName.c_str());
        if (id >= 0)
        {
          _ecm.SetComponentData<MujocoJointId>(_entity, id);
        }

        int actId = -1;
        for (int i = 0; i < this->model->nu; ++i) {
            if (this->model->actuator_trntype[i] == mjTRN_JOINT &&
                this->model->actuator_trnid[i * 2] == id) {
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
  }
}

void MujocoPhysics::UpdatePhysics(EntityComponentManager &_ecm)
{
  if (!this->model || !this->data)
    return;

  // Clear actuator controls
  if (this->model->nu > 0)
  {
    std::fill(this->data->ctrl,
              this->data->ctrl + this->model->nu,
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
        if (actId >= 0 && actId < this->model->nu)
        {
          if (forceCmd && !forceCmd->Data().empty())
          {
            this->data->ctrl[actId] = forceCmd->Data()[0];
          }
          else if (velCmd && !velCmd->Data().empty())
          {
            this->data->ctrl[actId] = velCmd->Data()[0];
          }
        }
      }
      return true;
    });

  // Clear applied forces
  std::fill(this->data->xfrc_applied,
            this->data->xfrc_applied + 6 * this->model->nbody,
            0.0);

  // Apply ExternalWorldWrenchCmd
  _ecm.Each<components::Link, MujocoBodyId, components::ExternalWorldWrenchCmd>(
    [&](const Entity &,
        const components::Link *,
        const MujocoBodyId *_bodyIdComp,
        const components::ExternalWorldWrenchCmd *_wrenchComp) -> bool
    {
      int mjBodyId = _bodyIdComp->Data();
      if (mjBodyId > 0 && mjBodyId < this->model->nbody)
      {
        auto wrench = _wrenchComp->Data();
        this->data->xfrc_applied[6 * mjBodyId + 0] += wrench.force().x();
        this->data->xfrc_applied[6 * mjBodyId + 1] += wrench.force().y();
        this->data->xfrc_applied[6 * mjBodyId + 2] += wrench.force().z();
        this->data->xfrc_applied[6 * mjBodyId + 3] += wrench.torque().x();
        this->data->xfrc_applied[6 * mjBodyId + 4] += wrench.torque().y();
        this->data->xfrc_applied[6 * mjBodyId + 5] += wrench.torque().z();
      }
      return true;
    });
}

void MujocoPhysics::Step(const UpdateInfo &_info)
{
  if (_info.paused || !this->model || !this->data)
    return;
  // Step the simulation
  mj_step(this->model, this->data);
}

void MujocoPhysics::UpdateSim(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (_info.paused || !this->model || !this->data)
    return;


  // Synchronize the ECM components::Pose of all links
  _ecm.Each<components::Link, MujocoBodyId, components::Pose, components::ParentEntity>(
    [&](const Entity &entity,
        const components::Link *,
        const MujocoBodyId *_bodyIdComp,
        components::Pose *_poseComp,
        const components::ParentEntity *_parentComp) -> bool
    {
      int mjBodyId = _bodyIdComp->Data();
      if (mjBodyId > 0 && mjBodyId < this->model->nbody)
      {
        mjtNum* pos = this->data->xpos + 3 * mjBodyId;
        mjtNum* quat = this->data->xquat + 4 * mjBodyId;

        math::Pose3d worldPose(
          pos[0], pos[1], pos[2],
          quat[0], quat[1], quat[2], quat[3]
        );

        math::Pose3d parentWorldPose = gz::sim::worldPose(_parentComp->Data(), _ecm);
        _poseComp->Data() = parentWorldPose.Inverse() * worldPose;

        _ecm.SetChanged(entity, components::Pose::typeId,
                        ComponentState::PeriodicChange);

        mjtNum velocity[6];
        mj_objectVelocity(this->model, this->data, mjOBJ_BODY, mjBodyId, velocity, 0);
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
    
    for (int i = 0; i < this->data->ncon; ++i) {
        const mjContact *con = this->data->contact + i;
        
        auto it1 = geomToEntity.find(con->geom[0]);
        auto it2 = geomToEntity.find(con->geom[1]);
        
        if (it1 != geomToEntity.end() && it2 != geomToEntity.end()) {
            Entity e1 = it1->second;
            Entity e2 = it2->second;
            
            mjtNum contactForce[6];
            mj_contactForce(this->model, this->data, i, contactForce);
            
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
  _ecm.Each<components::Imu, components::ParentEntity>(
    [&](const Entity &_entity, const components::Imu *, const components::ParentEntity *parentComp) -> bool
    {
      Entity parentLink = parentComp->Data();
      auto bodyIdComp = _ecm.Component<MujocoBodyId>(parentLink);
      if (!bodyIdComp) return true;
      int mjBodyId = bodyIdComp->Data();
      
      int siteId = -1;
      for (int i = 0; i < this->model->nsite; ++i) {
          if (this->model->site_bodyid[i] == mjBodyId) {
              siteId = i;
              break;
          }
      }
      if (siteId >= 0) {
          int accelId = -1;
          int gyroId = -1;
          for (int i = 0; i < this->model->nsensor; ++i) {
              if (this->model->sensor_objtype[i] == mjOBJ_SITE &&
                  this->model->sensor_objid[i] == siteId) {
                  if (this->model->sensor_type[i] == mjSENS_ACCELEROMETER) {
                      accelId = i;
                  } else if (this->model->sensor_type[i] == mjSENS_GYRO) {
                      gyroId = i;
                  }
              }
          }
          if (accelId >= 0 && gyroId >= 0) {
              int accelAdr = this->model->sensor_adr[accelId];
              int gyroAdr = this->model->sensor_adr[gyroId];
              
              math::Vector3d linAcc(
                  this->data->sensordata[accelAdr],
                  this->data->sensordata[accelAdr+1],
                  this->data->sensordata[accelAdr+2]
              );
              
              math::Vector3d angVel(
                  this->data->sensordata[gyroAdr],
                  this->data->sensordata[gyroAdr+1],
                  this->data->sensordata[gyroAdr+2]
              );
              
              auto accComp = _ecm.Component<components::LinearAcceleration>(_entity);
              if (accComp) accComp->Data() = linAcc;
              else _ecm.CreateComponent(_entity, components::LinearAcceleration(linAcc));
              
              auto angComp = _ecm.Component<components::AngularVelocity>(_entity);
              if (angComp) angComp->Data() = angVel;
              else _ecm.CreateComponent(_entity, components::AngularVelocity(angVel));
              
              _ecm.SetChanged(_entity, components::LinearAcceleration::typeId, ComponentState::PeriodicChange);
              _ecm.SetChanged(_entity, components::AngularVelocity::typeId, ComponentState::PeriodicChange);
          }
      }
      return true;
    });

  _ecm.Each<components::ForceTorque, components::ParentEntity>(
    [&](const Entity &_entity, const components::ForceTorque *, const components::ParentEntity *parentComp) -> bool
    {
      Entity parentJoint = parentComp->Data();
      Entity childLink = kNullEntity;
      auto childLinkComp = _ecm.Component<components::ChildLinkName>(parentJoint);
      if (childLinkComp) {
          Entity modelEntity = _ecm.ParentEntity(parentJoint);
          childLink = _ecm.EntityByComponents(components::ParentEntity(modelEntity), components::Name(childLinkComp->Data()), components::Link());
      }
      if (childLink == kNullEntity) return true;
      auto bodyIdComp = _ecm.Component<MujocoBodyId>(childLink);
      if (!bodyIdComp) return true;
      int mjBodyId = bodyIdComp->Data();

      int siteId = -1;
      for (int i = 0; i < this->model->nsite; ++i) {
          if (this->model->site_bodyid[i] == mjBodyId) {
              siteId = i;
              break;
          }
      }
      if (siteId >= 0) {
          int forceId = -1;
          int torqueId = -1;
          for (int i = 0; i < this->model->nsensor; ++i) {
              if (this->model->sensor_objtype[i] == mjOBJ_SITE &&
                  this->model->sensor_objid[i] == siteId) {
                  if (this->model->sensor_type[i] == mjSENS_FORCE) {
                      forceId = i;
                  } else if (this->model->sensor_type[i] == mjSENS_TORQUE) {
                      torqueId = i;
                  }
              }
          }
          if (forceId >= 0 && torqueId >= 0) {
              int forceAdr = this->model->sensor_adr[forceId];
              int torqueAdr = this->model->sensor_adr[torqueId];
              
              msgs::Wrench wrenchMsg;
              auto *forceMsg = wrenchMsg.mutable_force();
              forceMsg->set_x(this->data->sensordata[forceAdr]);
              forceMsg->set_y(this->data->sensordata[forceAdr+1]);
              forceMsg->set_z(this->data->sensordata[forceAdr+2]);
              
              auto *torqueMsg = wrenchMsg.mutable_torque();
              torqueMsg->set_x(this->data->sensordata[torqueAdr]);
              torqueMsg->set_y(this->data->sensordata[torqueAdr+1]);
              torqueMsg->set_z(this->data->sensordata[torqueAdr+2]);
              
              auto wrenchComp = _ecm.Component<components::WrenchMeasured>(_entity);
              if (wrenchComp) wrenchComp->Data() = wrenchMsg;
              else _ecm.CreateComponent(_entity, components::WrenchMeasured(wrenchMsg));
              
              _ecm.SetChanged(_entity, components::WrenchMeasured::typeId, ComponentState::PeriodicChange);
          }
      }
      return true;
    });

  // Clear / reset components
  std::vector<Entity> entitiesPositionReset;
  _ecm.Each<components::JointPositionReset>(
      [&](const Entity &_entity, components::JointPositionReset *) -> bool
      {
        entitiesPositionReset.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesPositionReset)
  {
    _ecm.RemoveComponent<components::JointPositionReset>(entity);
  }

  std::vector<Entity> entitiesVelocityReset;
  _ecm.Each<components::JointVelocityReset>(
      [&](const Entity &_entity, components::JointVelocityReset *) -> bool
      {
        entitiesVelocityReset.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesVelocityReset)
  {
    _ecm.RemoveComponent<components::JointVelocityReset>(entity);
  }

  std::vector<Entity> entitiesLinearVelocityReset;
  _ecm.Each<components::WorldLinearVelocityReset>(
      [&](const Entity &_entity,
      components::WorldLinearVelocityReset *) -> bool
      {
        entitiesLinearVelocityReset.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesLinearVelocityReset)
  {
    _ecm.RemoveComponent<components::WorldLinearVelocityReset>(entity);
  }

  std::vector<Entity> entitiesAngularVelocityReset;
  _ecm.Each<components::WorldAngularVelocityReset>(
      [&](const Entity &_entity,
      components::WorldAngularVelocityReset *) -> bool
      {
        entitiesAngularVelocityReset.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesAngularVelocityReset)
  {
    _ecm.RemoveComponent<components::WorldAngularVelocityReset>(entity);
  }

  std::vector<Entity> entitiesCustomContactSurface;
  _ecm.Each<components::EnableContactSurfaceCustomization>(
      [&](const Entity &_entity,
      components::EnableContactSurfaceCustomization *) -> bool
      {
        entitiesCustomContactSurface.push_back(_entity);
        return true;
      });

  for (const auto entity : entitiesCustomContactSurface)
  {
    _ecm.RemoveComponent<components::EnableContactSurfaceCustomization>(entity);
  }

  // Clear pending commands
  _ecm.Each<components::JointForceCmd>(
      [&](const Entity &, components::JointForceCmd *_force) -> bool
      {
        std::fill(_force->Data().begin(), _force->Data().end(), 0.0);
        return true;
      });

  _ecm.Each<components::ExternalWorldWrenchCmd >(
      [&](const Entity &, components::ExternalWorldWrenchCmd *_wrench) -> bool
      {
        _wrench->Data().Clear();
        return true;
      });

  _ecm.Each<components::JointPositionLimitsCmd>(
      [&](const Entity &, components::JointPositionLimitsCmd *_limits) -> bool
      {
        _limits->Data().clear();
        return true;
      });

  _ecm.Each<components::JointVelocityLimitsCmd>(
      [&](const Entity &, components::JointVelocityLimitsCmd *_limits) -> bool
      {
        _limits->Data().clear();
        return true;
      });

  _ecm.Each<components::JointEffortLimitsCmd>(
      [&](const Entity &, components::JointEffortLimitsCmd *_limits) -> bool
      {
        _limits->Data().clear();
        return true;
      });

  {
    std::vector<Entity> entitiesJointVelocityCmd;
    _ecm.Each<components::JointVelocityCmd>(
        [&](const Entity &_entity, components::JointVelocityCmd *) -> bool
        {
          entitiesJointVelocityCmd.push_back(_entity);
          return true;
        });

    for (const auto entity : entitiesJointVelocityCmd)
    {
      _ecm.RemoveComponent<components::JointVelocityCmd>(entity);
    }
  }
}

void MujocoPhysics::Update(const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
{
  this->CreatePhysicsEntities(_ecm);
  this->UpdatePhysics(_ecm);
  this->Step(_info);
  this->UpdateSim(_info, _ecm);
}

GZ_ADD_PLUGIN(MujocoPhysics,
              System,
              MujocoPhysics::ISystemConfigure,
              MujocoPhysics::ISystemUpdate)
GZ_ADD_PLUGIN_ALIAS(MujocoPhysics, "gz::sim::systems::MujocoPhysics")

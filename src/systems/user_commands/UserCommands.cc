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

#include "UserCommands.hh"

#include <google/protobuf/message.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/light.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/physics.pb.h>

#include <string>
#include <utility>
#include <vector>

#include <ignition/msgs/Utility.hh>

#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/Error.hh>
#include <sdf/Light.hh>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/common/Profiler.hh"

#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LightCmd.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/PoseCmd.hh"
#include "ignition/gazebo/components/PhysicsCmd.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/ContactSensor.hh"
#include "ignition/gazebo/components/Sensor.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
/// \brief This class is passed to every command and contains interfaces that
/// can be shared among all commands. For example, all create and remove
/// commands can use the `creator` object.
class UserCommandsInterface
{
  /// \brief Pointer to entity component manager. We don't assume ownership.
  public: EntityComponentManager *ecm{nullptr};

  /// \brief Creator interface, shared among all commands that need it.
  public: std::unique_ptr<SdfEntityCreator> creator{nullptr};

  /// \brief World entity.
  public: Entity worldEntity{kNullEntity};

  /// \brief Check if there's a contact sensor connected to a collision
  /// component
  /// \param[in] _collision Collision entity to be checked
  /// \return True if a contact sensor is connected to the collision entity,
  /// false otherwise
  public: bool HasContactSensor(const Entity _collision);
};

/// \brief All user commands should inherit from this class so they can be
/// undone / redone.
class UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message containing user command.
  /// \param[in] _iface Pointer to interfaces shared by all commands.
  public: UserCommandBase(google::protobuf::Message *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  /// \brief Destructor.
  public: virtual ~UserCommandBase();

  /// \brief Execute the command. All subclasses must implement this
  /// function and update entities and components so the command takes effect.
  /// \return True if command was properly executed.
  public: virtual bool Execute() = 0;

  /// \brief Message containing command.
  protected: google::protobuf::Message *msg{nullptr};

  /// \brief Keep pointer to interfaces shared among commands.
  protected: const std::shared_ptr<UserCommandsInterface> iface{nullptr};
};

/// \brief Command to spawn an entity into simulation.
class CreateCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Factory message.
  /// \param[in] _iface Pointer to user commands interface.
  public: CreateCommand(msgs::EntityFactory *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to remove an entity from simulation.
class RemoveCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message identifying the entity to be removed.
  /// \param[in] _iface Pointer to user commands interface.
  public: RemoveCommand(msgs::Entity *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to modify a light entity from simulation.
class LightCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message identifying the entity to be edited.
  /// \param[in] _iface Pointer to user commands interface.
  public: LightCommand(msgs::Light *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;

  /// \brief Light equality comparison function.
  public: std::function<bool(const msgs::Light &, const msgs::Light &)>
          lightEql { [](const msgs::Light &_a, const msgs::Light &_b)
            {
             return
                _a.type() == _b.type() &&
                _a.name() == _b.name() &&
                math::equal(
                   _a.diffuse().a(), _b.diffuse().a(), 1e-6f) &&
                math::equal(
                  _a.diffuse().r(), _b.diffuse().r(), 1e-6f) &&
                math::equal(
                  _a.diffuse().g(), _b.diffuse().g(), 1e-6f) &&
                math::equal(
                  _a.diffuse().b(), _b.diffuse().b(), 1e-6f) &&
                math::equal(
                  _a.specular().a(), _b.specular().a(), 1e-6f) &&
                math::equal(
                  _a.specular().r(), _b.specular().r(), 1e-6f) &&
                math::equal(
                  _a.specular().g(), _b.specular().g(), 1e-6f) &&
                math::equal(
                  _a.specular().b(), _b.specular().b(), 1e-6f) &&
                math::equal(
                  _a.range(), _b.range(), 1e-6f) &&
               math::equal(
                 _a.attenuation_linear(),
                 _b.attenuation_linear(),
                 1e-6f) &&
               math::equal(
                 _a.attenuation_constant(),
                 _b.attenuation_constant(),
                 1e-6f) &&
               math::equal(
                 _a.attenuation_quadratic(),
                 _b.attenuation_quadratic(),
                 1e-6f) &&
               _a.cast_shadows() == _b.cast_shadows() &&
               math::equal(
                 _a.direction().x(), _b.direction().x(), 1e-6) &&
               math::equal(
                 _a.direction().y(), _b.direction().y(), 1e-6) &&
               math::equal(
                 _a.direction().z(), _b.direction().z(), 1e-6) &&
               math::equal(
                 _a.spot_inner_angle(), _b.spot_inner_angle(), 1e-6f) &&
               math::equal(
                 _a.spot_outer_angle(), _b.spot_outer_angle(), 1e-6f) &&
               math::equal(_a.spot_falloff(), _b.spot_falloff(), 1e-6f);
            }};
};

/// \brief Command to update an entity's pose transform.
class PoseCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg pose message.
  /// \param[in] _iface Pointer to user commands interface.
  public: PoseCommand(msgs::Pose *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;

  /// \brief Pose3d equality comparison function.
  public: std::function<bool(const math::Pose3d &, const math::Pose3d &)>
          pose3Eql { [](const math::Pose3d &_a, const math::Pose3d &_b)
                     {
                       return _a.Pos().Equal(_b.Pos(), 1e-6) &&
                         math::equal(_a.Rot().X(), _b.Rot().X(), 1e-6) &&
                         math::equal(_a.Rot().Y(), _b.Rot().Y(), 1e-6) &&
                         math::equal(_a.Rot().Z(), _b.Rot().Z(), 1e-6) &&
                         math::equal(_a.Rot().W(), _b.Rot().W(), 1e-6);
                     }};
};

/// \brief Command to modify the physics parameters of a simulation.
class PhysicsCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message containing the new physics parameters.
  /// \param[in] _iface Pointer to user commands interface.
  public: PhysicsCommand(msgs::Physics *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to enable a collision component.
class EnableCollisionCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message identifying the collision to be enabled.
  /// \param[in] _iface Pointer to user commands interface.
  public: EnableCollisionCommand(msgs::Entity *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to disable a collision component.
class DisableCollisionCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message identifying the collision to be disabled.
  /// \param[in] _iface Pointer to user commands interface.
  public: DisableCollisionCommand(msgs::Entity *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};
}
}
}
}

/// \brief Private UserCommands data class.
class ignition::gazebo::systems::UserCommandsPrivate
{
  /// \brief Callback for create service
  /// \param[in] _req Request containing entity description.
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the entity will be successfully spawned.
  /// \return True if successful.
  public: bool CreateService(const msgs::EntityFactory &_req,
      msgs::Boolean &_res);

  /// \brief Callback for multiple create service
  /// \param[in] _req Request containing one or more entity descriptions.
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the entities will be successfully spawned.
  /// \return True if successful.
  public: bool CreateServiceMultiple(
              const msgs::EntityFactory_V &_req, msgs::Boolean &_res);

  /// \brief Callback for remove service
  /// \param[in] _req Request containing identification of entity to be removed.
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the entity will be successfully removed.
  /// \return True if successful.
  public: bool RemoveService(const msgs::Entity &_req,
      msgs::Boolean &_res);

  /// \brief Callback for light service
  /// \param[in] _req Request containing light update of an entity.
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the light will be successfully updated.
  /// \return True if successful.
  public: bool LightService(const msgs::Light &_req, msgs::Boolean &_res);

  /// \brief Callback for pose service
  /// \param[in] _req Request containing pose update of an entity.
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the entity will be successfully moved.
  /// \return True if successful.
  public: bool PoseService(const msgs::Pose &_req, msgs::Boolean &_res);

  /// \brief Callback for physics service
  /// \param[in] _req Request containing updates to the physics parameters.
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the physics parameters will be successfully updated.
  /// \return True if successful.
  public: bool PhysicsService(const msgs::Physics &_req, msgs::Boolean &_res);

  /// \brief Callback for enable collision service
  /// \param[in] _req Request containing collision entity.
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the collision will be successfully enabled.
  /// \return True if successful.
  public: bool EnableCollisionService(
      const msgs::Entity &_req, msgs::Boolean &_res);

  /// \brief Callback for disable collision service
  /// \param[in] _req Request containing collision entity.
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the collision will be successfully disabled.
  /// \return True if successful.
  public: bool DisableCollisionService(
      const msgs::Entity &_req, msgs::Boolean &_res);

  /// \brief Queue of commands pending execution.
  public: std::vector<std::unique_ptr<UserCommandBase>> pendingCmds;

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Object holding several interfaces that can be used by any command.
  public: std::shared_ptr<UserCommandsInterface> iface{nullptr};

  /// \brief Mutex to protect pending queue.
  public: std::mutex pendingMutex;
};

//////////////////////////////////////////////////
UserCommands::UserCommands() : System(),
    dataPtr(std::make_unique<UserCommandsPrivate>())
{
}

//////////////////////////////////////////////////
UserCommands::~UserCommands() = default;

//////////////////////////////////////////////////
bool UserCommandsInterface::HasContactSensor(const Entity _collision)
{
  auto *linkEntity = ecm->Component<components::ParentEntity>(_collision);
  auto allLinkSensors =
    ecm->EntitiesByComponents(components::Sensor(),
      components::ParentEntity(*linkEntity));

  for (auto const &sensor : allLinkSensors)
  {
    // Check if it is a contact sensor
    auto isContactSensor =
      ecm->EntityHasComponentType(sensor, components::ContactSensor::typeId);
    if (!isContactSensor)
      continue;

    // Check if sensor is connected to _collision
    auto componentName = ecm->Component<components::Name>(_collision);
    std::string collisionName = componentName->Data();
    auto sensorSDF = ecm->Component<components::ContactSensor>(sensor)->Data();
    auto sensorCollisionName =
      sensorSDF->GetElement("contact")->Get<std::string>("collision");

    if (collisionName == sensorCollisionName)
    {
      return true;
    }
  }

  return false;
}

//////////////////////////////////////////////////
void UserCommands::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &_ecm,
    EventManager &_eventManager)
{
  // Create interfaces shared among commands
  this->dataPtr->iface = std::make_shared<UserCommandsInterface>();
  this->dataPtr->iface->worldEntity = _entity;
  this->dataPtr->iface->ecm = &_ecm;
  this->dataPtr->iface->creator =
      std::make_unique<SdfEntityCreator>(_ecm, _eventManager);

  const components::Name *constCmp = _ecm.Component<components::Name>(_entity);
  const std::string &worldName = constCmp->Data();

  auto validWorldName = transport::TopicUtils::AsValidTopic(worldName);
  if (validWorldName.empty())
  {
    ignerr << "World name [" << worldName
           << "] doesn't work well with transport, services not advertised."
           << std::endl;
    return;
  }

  // Create service
  std::string createService{"/world/" + validWorldName + "/create"};
  this->dataPtr->node.Advertise(createService,
      &UserCommandsPrivate::CreateService, this->dataPtr.get());

  // Create service for EntityFactory_V
  std::string createServiceMultiple{"/world/" + validWorldName +
      "/create_multiple"};
  this->dataPtr->node.Advertise(createServiceMultiple,
      &UserCommandsPrivate::CreateServiceMultiple, this->dataPtr.get());

  ignmsg << "Create service on [" << createService << "]" << std::endl;

  // Remove service
  std::string removeService{"/world/" + validWorldName + "/remove"};
  this->dataPtr->node.Advertise(removeService,
      &UserCommandsPrivate::RemoveService, this->dataPtr.get());

  ignmsg << "Remove service on [" << removeService << "]" << std::endl;

  // Pose service
  std::string poseService{"/world/" + validWorldName + "/set_pose"};
  this->dataPtr->node.Advertise(poseService,
      &UserCommandsPrivate::PoseService, this->dataPtr.get());

  ignmsg << "Pose service on [" << poseService << "]" << std::endl;

  // Light service
  std::string lightService{"/world/" + validWorldName + "/light_config"};
  this->dataPtr->node.Advertise(lightService,
      &UserCommandsPrivate::LightService, this->dataPtr.get());

  ignmsg << "Light configuration service on [" << lightService << "]"
    << std::endl;

  // Physics service
  std::string physicsService{"/world/" + validWorldName + "/set_physics"};
  this->dataPtr->node.Advertise(physicsService,
      &UserCommandsPrivate::PhysicsService, this->dataPtr.get());

  ignmsg << "Physics service on [" << physicsService << "]" << std::endl;

  // Enable collision service
  std::string enableCollisionService{
    "/world/" + validWorldName + "/enable_collision"};
  this->dataPtr->node.Advertise(enableCollisionService,
      &UserCommandsPrivate::EnableCollisionService, this->dataPtr.get());

  ignmsg << "Enable collision service on [" << enableCollisionService << "]"
    << std::endl;

  // Disable collision service
  std::string disableCollisionService{
    "/world/" + validWorldName + "/disable_collision"};
  this->dataPtr->node.Advertise(disableCollisionService,
      &UserCommandsPrivate::DisableCollisionService, this->dataPtr.get());

  ignmsg << "Disable collision service on [" << disableCollisionService << "]"
    << std::endl;
}

//////////////////////////////////////////////////
void UserCommands::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &)
{
  IGN_PROFILE("UserCommands::PreUpdate");
  // make a copy the cmds so execution does not block receiving other
  // incoming cmds
  std::vector<std::unique_ptr<UserCommandBase>> cmds;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->pendingMutex);
    if (this->dataPtr->pendingCmds.empty())
      return;
    cmds = std::move(this->dataPtr->pendingCmds);
    this->dataPtr->pendingCmds.clear();
  }

  // TODO(louise) Record current world state for undo

  // Execute pending commands
  for (auto &cmd : cmds)
  {
    // Execute
    if (!cmd->Execute())
      continue;

    // TODO(louise) Update command with current world state

    // TODO(louise) Move to undo list
  }

  // TODO(louise) Clear redo list
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::CreateServiceMultiple(
    const msgs::EntityFactory_V &_req, msgs::Boolean &_res)
{
  std::lock_guard<std::mutex> lock(this->pendingMutex);
  for (int i = 0; i < _req.data_size(); ++i)
  {
    const msgs::EntityFactory &msg = _req.data(i);
    // Create command and push it to queue
    auto msgCopy = msg.New();
    msgCopy->CopyFrom(msg);
    auto cmd = std::make_unique<CreateCommand>(msgCopy, this->iface);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::CreateService(const msgs::EntityFactory &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<CreateCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::RemoveService(const msgs::Entity &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<RemoveCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::LightService(const msgs::Light &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<LightCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::PoseService(const msgs::Pose &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<PoseCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::EnableCollisionService(const msgs::Entity &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<EnableCollisionCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::DisableCollisionService(const msgs::Entity &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<DisableCollisionCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::PhysicsService(const msgs::Physics &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<PhysicsCommand>(msg, this->iface);
  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
UserCommandBase::UserCommandBase(google::protobuf::Message *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : msg(_msg), iface(_iface)
{
}

//////////////////////////////////////////////////
UserCommandBase::~UserCommandBase()
{
  if (this->msg != nullptr)
    delete this->msg;
  this->msg = nullptr;
}

//////////////////////////////////////////////////
CreateCommand::CreateCommand(msgs::EntityFactory *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool CreateCommand::Execute()
{
  auto createMsg = dynamic_cast<const msgs::EntityFactory *>(this->msg);
  if (nullptr == createMsg)
  {
    ignerr << "Internal error, null create message" << std::endl;
    return false;
  }

  // Load SDF
  sdf::Root root;
  sdf::Light lightSdf;
  sdf::Errors errors;
  switch (createMsg->from_case())
  {
    case msgs::EntityFactory::kSdf:
    {
      errors = root.LoadSdfString(createMsg->sdf());
      break;
    }
    case msgs::EntityFactory::kSdfFilename:
    {
      errors = root.Load(createMsg->sdf_filename());
      break;
    }
    case msgs::EntityFactory::kModel:
    {
      // TODO(louise) Support model msg
      ignerr << "model field not yet supported." << std::endl;
      return false;
    }
    case msgs::EntityFactory::kLight:
    {
      lightSdf = convert<sdf::Light>(createMsg->light());
      break;
    }
    case msgs::EntityFactory::kCloneName:
    {
      // TODO(louise) Implement clone
      ignerr << "Cloning an entity is not yet supported." << std::endl;
      return false;
    }
    default:
    {
      ignerr << "Missing [from] field in create message." << std::endl;
      return false;
    }
  }

  if (!errors.empty())
  {
    for (auto &err : errors)
      ignerr << err << std::endl;
    return false;
  }

  bool isModel{false};
  bool isLight{false};
  bool isActor{false};
  bool isRoot{false};
  if (root.ModelCount() > 0)
  {
    isRoot = true;
    isModel = true;
  }
  else if (root.LightCount() > 0)
  {
    isRoot = true;
    isLight = true;
  }
  else if (root.ActorCount() > 0)
  {
    isRoot = true;
    isActor = true;
  }
  else if (!lightSdf.Name().empty())
  {
    isLight = true;
  }
  else
  {
    ignerr << "Expected exactly one top-level <model>, <light> or <actor> on"
           << " SDF." << std::endl;
    return false;
  }

  if ((root.ModelCount() + root.LightCount() + root.ActorCount()) > 1)
  {
    ignwarn << "Expected exactly one top-level <model>, <light> or <actor>, "
            << "but found more. Only the 1st will be spawned." << std::endl;
  }

  // Check the name of the entity being spawned
  std::string desiredName;
  if (!createMsg->name().empty())
  {
    desiredName = createMsg->name();
  }
  else if (isModel)
  {
    desiredName = root.ModelByIndex(0)->Name();
  }
  else if (isLight && isRoot)
  {
    desiredName = root.LightByIndex(0)->Name();
  }
  else if (isLight)
  {
    desiredName = lightSdf.Name();
  }
  else if (isActor)
  {
    desiredName = root.ActorByIndex(0)->Name();
  }

  // Check if there's already a top-level entity with the given name
  if (kNullEntity != this->iface->ecm->EntityByComponents(
      components::Name(desiredName),
      components::ParentEntity(this->iface->worldEntity)))
  {
    if (!createMsg->allow_renaming())
    {
      ignwarn << "Entity named [" << desiredName << "] already exists and "
              << "[allow_renaming] is false. Entity not spawned."
              << std::endl;
      return false;
    }

    // Generate unique name
    std::string newName = desiredName;
    int i = 0;
    while (kNullEntity != this->iface->ecm->EntityByComponents(
      components::Name(newName),
      components::ParentEntity(this->iface->worldEntity)))
    {
      newName = desiredName + "_" + std::to_string(i++);
    }
    desiredName = newName;
  }

  // Create entities
  Entity entity{kNullEntity};
  if (isModel)
  {
    auto model = *root.ModelByIndex(0);
    model.SetName(desiredName);
    entity = this->iface->creator->CreateEntities(&model);
  }
  else if (isLight && isRoot)
  {
    auto light = root.LightByIndex(0);
    light->SetName(desiredName);
    entity = this->iface->creator->CreateEntities(light);
  }
  else if (isLight)
  {
    lightSdf.SetName(desiredName);
    entity = this->iface->creator->CreateEntities(&lightSdf);
  }
  else if (isActor)
  {
    auto actor = *root.ActorByIndex(0);
    actor.SetName(desiredName);
    entity = this->iface->creator->CreateEntities(&actor);
  }

  this->iface->creator->SetParent(entity, this->iface->worldEntity);

  // Pose
  if (createMsg->has_pose())
  {
    auto poseComp = this->iface->ecm->Component<components::Pose>(entity);
    *poseComp = components::Pose(msgs::Convert(createMsg->pose()));
  }

  igndbg << "Created entity [" << entity << "] named [" << desiredName << "]"
         << std::endl;

  return true;
}

//////////////////////////////////////////////////
RemoveCommand::RemoveCommand(msgs::Entity *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool RemoveCommand::Execute()
{
  auto removeMsg = dynamic_cast<const msgs::Entity *>(this->msg);
  if (nullptr == removeMsg)
  {
    ignerr << "Internal error, null remove message" << std::endl;
    return false;
  }

  Entity entity{kNullEntity};
  if (removeMsg->id() != kNullEntity)
  {
    entity = removeMsg->id();
  }
  else if (!removeMsg->name().empty() &&
      removeMsg->type() != msgs::Entity::NONE)
  {
    if (removeMsg->type() == msgs::Entity::MODEL)
    {
      entity = this->iface->ecm->EntityByComponents(components::Model(),
        components::Name(removeMsg->name()));
    }
    else if (removeMsg->type() == msgs::Entity::LIGHT)
    {
      entity = this->iface->ecm->EntityByComponents(
        components::Name(removeMsg->name()));

      auto lightComp = this->iface->ecm->Component<components::Light>(entity);
      if (nullptr == lightComp)
        entity = kNullEntity;
    }
    else
    {
      ignerr << "Deleting entities of type [" << removeMsg->type()
             << "] is not supported." << std::endl;
      return false;
    }
  }
  else
  {
    ignerr << "Remove command missing either entity's ID or name + type"
           << std::endl;
    return false;
  }

  if (entity == kNullEntity)
  {
    ignerr << "Entity named [" << removeMsg->name() << "] of type ["
           << removeMsg->type() << "] not found, so not removed." << std::endl;
    return false;
  }

  // Check that we support removing this entity
  auto parent = this->iface->ecm->ParentEntity(entity);
  if (nullptr == this->iface->ecm->Component<components::World>(parent))
  {
    ignerr << "Entity [" << entity
           << "] is not a direct child of the world, so it can't be removed."
           << std::endl;
    return false;
  }

  if (nullptr == this->iface->ecm->Component<components::Model>(entity) &&
      nullptr == this->iface->ecm->Component<components::Light>(entity))
  {
    ignerr << "Entity [" << entity
           << "] is not a model or a light, so it can't be removed."
           << std::endl;
    return false;
  }

  igndbg << "Requesting removal of entity [" << entity << "]" << std::endl;
  this->iface->creator->RequestRemoveEntity(entity);
  return true;
}

//////////////////////////////////////////////////
LightCommand::LightCommand(msgs::Light *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool LightCommand::Execute()
{
  auto lightMsg = dynamic_cast<const msgs::Light *>(this->msg);
  if (nullptr == lightMsg)
  {
    ignerr << "Internal error, null light message" << std::endl;
    return false;
  }

  Entity lightEntity{kNullEntity};

  if (lightMsg->id() != kNullEntity)
  {
    lightEntity = lightMsg->id();
  }
  else if (!lightMsg->name().empty())
  {
    if (lightMsg->parent_id() != kNullEntity)
    {
      lightEntity = this->iface->ecm->EntityByComponents(
        components::Name(lightMsg->name()),
        components::ParentEntity(lightMsg->parent_id()));
    }
    else
    {
      lightEntity = this->iface->ecm->EntityByComponents(
        components::Name(lightMsg->name()));
    }
  }
  if (kNullEntity == lightEntity)
  {
    ignerr << "Failed to find light with name [" << lightMsg->name()
           << "], ID [" << lightMsg->id() << "] and parent ID ["
           << lightMsg->parent_id() << "]." << std::endl;
    return false;
  }

  if (!lightEntity)
  {
    ignmsg << "Failed to find light entity named [" << lightMsg->name()
      << "]." << std::endl;
    return false;
  }

  auto lightPose = this->iface->ecm->Component<components::Pose>(lightEntity);
  if (nullptr == lightPose)
    lightEntity = kNullEntity;

  if (!lightEntity)
  {
    ignmsg << "Pose component not available" << std::endl;
    return false;
  }

  if (lightMsg->has_pose())
  {
    lightPose->Data().Pos() = msgs::Convert(lightMsg->pose()).Pos();
  }

  auto lightCmdComp =
    this->iface->ecm->Component<components::LightCmd>(lightEntity);
  if (!lightCmdComp)
  {
    this->iface->ecm->CreateComponent(
        lightEntity, components::LightCmd(*lightMsg));
  }
  else
  {
    auto state = lightCmdComp->SetData(*lightMsg, this->lightEql) ?
        ComponentState::OneTimeChange :
        ComponentState::NoChange;
    this->iface->ecm->SetChanged(lightEntity, components::LightCmd::typeId,
      state);
  }

  return true;
}

//////////////////////////////////////////////////
PoseCommand::PoseCommand(msgs::Pose *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool PoseCommand::Execute()
{
  auto poseMsg = dynamic_cast<const msgs::Pose *>(this->msg);
  if (nullptr == poseMsg)
  {
    ignerr << "Internal error, null create message" << std::endl;
    return false;
  }

  // Check the name of the entity being spawned
  std::string entityName = poseMsg->name();
  Entity entity = kNullEntity;
  // TODO(anyone) Update pose message to use Entity, with default ID null
  if (poseMsg->id() != kNullEntity && poseMsg->id() != 0)
  {
    entity = poseMsg->id();
  }
  else if (!entityName.empty())
  {
    entity = this->iface->ecm->EntityByComponents(components::Name(entityName),
      components::ParentEntity(this->iface->worldEntity));
  }

  if (!this->iface->ecm->HasEntity(entity))
  {
    ignerr << "Unable to update the pose for entity id:[" << poseMsg->id()
           << "], name[" << entityName << "]" << std::endl;
    return false;
  }

  auto poseCmdComp =
    this->iface->ecm->Component<components::WorldPoseCmd>(entity);
  if (!poseCmdComp)
  {
    this->iface->ecm->CreateComponent(
        entity, components::WorldPoseCmd(msgs::Convert(*poseMsg)));
  }
  else
  {
    /// \todo(anyone) Moving an object is not captured in a log file.
    auto state = poseCmdComp->SetData(msgs::Convert(*poseMsg), this->pose3Eql) ?
        ComponentState::OneTimeChange :
        ComponentState::NoChange;
    this->iface->ecm->SetChanged(entity, components::WorldPoseCmd::typeId,
        state);
  }

  return true;
}

//////////////////////////////////////////////////
PhysicsCommand::PhysicsCommand(msgs::Physics *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool PhysicsCommand::Execute()
{
  auto physicsMsg = dynamic_cast<const msgs::Physics *>(this->msg);
  if (nullptr == physicsMsg)
  {
    ignerr << "Internal error, null physics message" << std::endl;
    return false;
  }

  auto worldEntity = this->iface->ecm->EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
  {
    ignmsg << "Failed to find world entity" << std::endl;
    return false;
  }

  if (!this->iface->ecm->EntityHasComponentType(worldEntity,
    components::PhysicsCmd().TypeId()))
  {
    this->iface->ecm->CreateComponent(worldEntity,
        components::PhysicsCmd(*physicsMsg));
  }

  return true;
}

//////////////////////////////////////////////////
EnableCollisionCommand::EnableCollisionCommand(msgs::Entity *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool EnableCollisionCommand::Execute()
{
  auto entityMsg = dynamic_cast<const msgs::Entity *>(this->msg);
  if (nullptr == entityMsg)
  {
    ignerr << "Internal error, null create message" << std::endl;
    return false;
  }

  // Check Entity type
  if (entityMsg->type() != msgs::Entity::COLLISION)
  {
    ignwarn << "Expected msgs::Entity::Type::COLLISION, exiting service..."
      << std::endl;
    return false;
  }

  // Check if collision is connected to a contact sensor
  if (this->iface->HasContactSensor(entityMsg->id()))
  {
    ignwarn << "Requested collision is connected to a contact sensor, "
      << "exiting service..." << std::endl;
    return false;
  }

  // Create ContactSensorData component
  auto contactDataComp =
    this->iface->ecm->Component<
      components::ContactSensorData>(entityMsg->id());
  if (contactDataComp)
  {
    ignwarn << "Can't create component that already exists" << std::endl;
    return false;
  }

  this->iface->ecm->
    CreateComponent(entityMsg->id(), components::ContactSensorData());
  igndbg << "Enabled collision [" << entityMsg->id() << "]" << std::endl;

  return true;
}

//////////////////////////////////////////////////
DisableCollisionCommand::DisableCollisionCommand(msgs::Entity *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool DisableCollisionCommand::Execute()
{
  auto entityMsg = dynamic_cast<const msgs::Entity *>(this->msg);
  if (nullptr == entityMsg)
  {
    ignerr << "Internal error, null create message" << std::endl;
    return false;
  }

  // Check Entity type
  if (entityMsg->type() != msgs::Entity::COLLISION)
  {
    ignwarn << "Expected msgs::Entity::Type::COLLISION, exiting service..."
      << std::endl;
    return false;
  }

  // Check if collision is connected to a contact sensor
  if (this->iface->HasContactSensor(entityMsg->id()))
  {
    ignwarn << "Requested collision is connected to a contact sensor, "
      << "exiting service..." << std::endl;
    return false;
  }

  // Remove ContactSensorData component
  auto *contactDataComp =
    this->iface->ecm->Component<
      components::ContactSensorData>(entityMsg->id());
  if (!contactDataComp)
  {
    ignwarn << "No ContactSensorData detected inside entity " << entityMsg->id()
      << std::endl;
    return false;
  }

  this->iface->ecm->
    RemoveComponent(entityMsg->id(), components::ContactSensorData::typeId);

  igndbg << "Disabled collision [" << entityMsg->id() << "]" << std::endl;

  return true;
}

IGNITION_ADD_PLUGIN(UserCommands, System,
  UserCommands::ISystemConfigure,
  UserCommands::ISystemPreUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(UserCommands,
                          "ignition::gazebo::systems::UserCommands")

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

#include <string>
#include <utility>
#include <vector>

#include <ignition/msgs/Utility.hh>

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
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"

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

  // Create service
  std::string createService{"/world/" + worldName + "/create"};
  this->dataPtr->node.Advertise(createService,
      &UserCommandsPrivate::CreateService, this->dataPtr.get());

  // Create service for EntityFactory_V
  std::string createServiceMultiple{"/world/" + worldName + "/create_multiple"};
  this->dataPtr->node.Advertise(createServiceMultiple,
      &UserCommandsPrivate::CreateServiceMultiple, this->dataPtr.get());

  ignmsg << "Create service on [" << createService << "]" << std::endl;

  // Remove service
  std::string removeService{"/world/" + worldName + "/remove"};
  this->dataPtr->node.Advertise(removeService,
      &UserCommandsPrivate::RemoveService, this->dataPtr.get());

  ignmsg << "Remove service on [" << removeService << "]" << std::endl;

  // Pose service
  std::string poseService{"/world/" + worldName + "/set_pose"};
  this->dataPtr->node.Advertise(poseService,
      &UserCommandsPrivate::PoseService, this->dataPtr.get());

  ignmsg << "Pose service on [" << poseService << "]" << std::endl;

  // Light service
  std::string lightService{"/world/" + worldName + "/light_config"};
  this->dataPtr->node.Advertise(lightService,
      &UserCommandsPrivate::LightService, this->dataPtr.get());

  ignmsg << "Light configuration service on [" << lightService << "]"
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
      // TODO(louise) Support light msg
      ignerr << "light field not yet supported." << std::endl;
      return false;
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
  if (root.ModelCount() > 0)
  {
    isModel = true;
  }
  else if (root.LightCount() > 0)
  {
    isLight = true;
  }
  else if (root.ActorCount() > 0)
  {
    isActor = true;
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
  else if (isLight)
  {
    desiredName = root.LightByIndex(0)->Name();
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
  else if (isLight)
  {
    auto light = root.LightByIndex(0);
    light->SetName(desiredName);
    entity = this->iface->creator->CreateEntities(light);
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

  if (lightMsg->id() != 0)
  {
    lightEntity = lightMsg->id();
  }
  else if (!lightMsg->name().empty())
  {
    if (lightMsg->parent_id() != 0)
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

  auto lightComp = this->iface->ecm->Component<components::Light>(lightEntity);
  if (nullptr == lightComp)
  {
    lightEntity = kNullEntity;
    // try to find the light inside a link
    auto tempLightEnty = this->iface->ecm->EntityByComponents(
      components::Name(lightMsg->name()));
    if (tempLightEnty != kNullEntity)
    {
      // check if light parent is a link
      auto parentComp = this->iface->ecm->Component<components::ParentEntity>(
        tempLightEnty);
      if (parentComp && this->iface->ecm->Component<components::Link>(
        parentComp->Data()))
      {
        lightComp =
          this->iface->ecm->Component<components::Light>(tempLightEnty);
        lightEntity = tempLightEnty;
      }
    }
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


IGNITION_ADD_PLUGIN(UserCommands, System,
  UserCommands::ISystemConfigure,
  UserCommands::ISystemPreUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(UserCommands,
                          "ignition::gazebo::systems::UserCommands")

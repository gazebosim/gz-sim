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

#include <google/protobuf/message.h>
#include <sdf/Root.hh>
#include <sdf/Error.hh>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Factory.hh"

#include "UserCommands.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

namespace ignition
{
namespace gazebo
{
namespace systems
{
/// \brief This class is passed to every command and contains interfaces that
/// can be shared among all commands. For example, all factory and delete
/// commands can use the `factory` object.
class UserCommandsInterface
{
  /// \brief Pointer to entity component manager. We don't assume ownership.
  public: EntityComponentManager *ecm{nullptr};

  /// \brief Factory interface, shared among all commands that need it.
  public: std::unique_ptr<Factory> factory{nullptr};

  /// \brief World entity.
  public: Entity worldEntity{kNullEntity};
};

/// \brief All user commands should inherit from this class so they can be
/// undone / redone.
class UserCommand
{
  /// \brief Constructor
  /// \param[in] _msg Message containing user command.
  /// \param[in] _iface Pointer to interfaces shared by all commands.
  public: UserCommand(google::protobuf::Message *_msg,
      const std::shared_ptr<UserCommandsInterface> &_iface);

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
class FactoryCommand : public UserCommand
{
  /// \brief Constructor
  /// \param[in] _msg Factory message.
  /// \param[in] _iface Pointer to user commands interface.
  public: FactoryCommand(msgs::EntityFactory *_msg,
      const std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: virtual bool Execute() final;
};
}
}
}

/// \brief Private UserCommands data class.
class ignition::gazebo::systems::UserCommandsPrivate
{
  /// \brief Callback for factory service
  /// \param[in] _req Request
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the entity will be successfully spawned.
  /// \return True if successful.
  public: bool FactoryService(const msgs::EntityFactory &_req,
      msgs::Boolean &_res);

  /// \brief Queue of commands pending execution.
  public: std::vector<std::unique_ptr<UserCommand>> pendingCmds;

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
  this->dataPtr->iface->factory = std::make_unique<Factory>(_ecm, _eventManager);

  // Spawn service
  auto worldName = _ecm.Component<components::Name>(_entity)->Data();

  std::string factoryService{"/world/" + worldName + "/factory"};
  this->dataPtr->node.Advertise(factoryService, &UserCommandsPrivate::FactoryService,
      this->dataPtr.get());

  ignmsg << "Factory service on [" << factoryService << "]" << std::endl;
}

//////////////////////////////////////////////////
void UserCommands::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->pendingMutex);
  if (this->dataPtr->pendingCmds.empty())
    return;

  // TODO(louise) Record current world state for undo

  // Execute pending commands
  for (auto &cmd : this->dataPtr->pendingCmds)
  {
    // Execute
    if (!cmd->Execute())
      continue;

    // TODO(louise) Update command with current world state

    // TODO(louise) Move to undo list
  }

  // TODO(louise) Clear redo list

  this->dataPtr->pendingCmds.clear();
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::FactoryService(const msgs::EntityFactory &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<FactoryCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
UserCommand::UserCommand(google::protobuf::Message *_msg,
    const std::shared_ptr<UserCommandsInterface> &_iface)
    : msg(_msg), iface(_iface)
{
}

//////////////////////////////////////////////////
FactoryCommand::FactoryCommand(msgs::EntityFactory *_msg,
    const std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommand(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool FactoryCommand::Execute()
{
  auto factoryMsg = dynamic_cast<const msgs::EntityFactory *>(this->msg);
  if (nullptr == factoryMsg)
  {
    ignerr << "Internal error, null factory message" << std::endl;
    return false;
  }

  // Load SDF
  sdf::Root root;
  sdf::Errors errors;
  switch (factoryMsg->from_case())
  {
    case msgs::EntityFactory::kSdf:
    {
      errors = root.LoadSdfString(factoryMsg->sdf());
      break;
    }
    case msgs::EntityFactory::kSdfFilename:
    {
      errors = root.Load(factoryMsg->sdf_filename());
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
      ignerr << "Missing [from] field in factory message." << std::endl;
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
  if (root.ModelCount() > 0)
  {
    isModel = true;
  }
  else if (root.LightCount() > 0)
  {
    isLight = true;
  }
  else
  {
    ignerr << "Expected exactly one top-level <model> or <light> on SDF."
           << std::endl;
    return false;
  }

  if ((root.ModelCount() + root.LightCount()) > 1)
  {
    ignwarn << "Expected exactly one top-level <model> or <light>, "
            << "but found more. Only the 1st will be spawned." << std::endl;
  }

  // Check the name of the entity being spawned
  std::string desiredName;
  if (!factoryMsg->name().empty())
  {
    desiredName = factoryMsg->name();
  }
  else if (isModel)
  {
    desiredName = root.ModelByIndex(0)->Name();
  }
  else if (isLight)
  {
    desiredName = root.LightByIndex(0)->Name();
  }

  // Check if there's already a top-level entity with the given name
  if (kNullEntity != this->iface->ecm->EntityByComponents(
      components::Name(desiredName),
      components::ParentEntity(this->iface->worldEntity)))
  {
    if (!factoryMsg->allow_renaming())
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
    entity = this->iface->factory->CreateEntities(root.ModelByIndex(0));
  }
  else if (isLight)
  {
    entity = this->iface->factory->CreateEntities(root.LightByIndex(0));
  }

  this->iface->factory->SetParent(entity, this->iface->worldEntity);

  // Pose
  if (factoryMsg->has_pose())
  {
    auto poseComp = this->iface->ecm->Component<components::Pose>(entity);
    *poseComp = components::Pose(msgs::Convert(factoryMsg->pose()));
  }

  // Update name
  auto nameComp = this->iface->ecm->Component<components::Name>(entity);
  *nameComp = components::Name(desiredName);

  return true;
}

IGNITION_ADD_PLUGIN(UserCommands, System,
  UserCommands::ISystemConfigure,
  UserCommands::ISystemPreUpdate
)

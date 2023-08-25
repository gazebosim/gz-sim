/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <gz/msgs/entity_wrench.pb.h>

#include <mutex>
#include <string>
#include <queue>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Link.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/World.hh"
#include "gz/sim/Util.hh"

#include "ApplyLinkWrench.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ApplyLinkWrenchPrivate
{
  /// \brief Callback for wrench subscription
  /// \param[in] _msg Wrench message
  public: void OnWrench(const msgs::EntityWrench &_msg);

  /// \brief Callback for persistent wrench subscription
  /// \param[in] _msg Wrench message
  public: void OnWrenchPersistent(const msgs::EntityWrench &_msg);

  /// \brief Callback for clearing persistent wrenches
  /// \param[in] _msg Entity message
  public: void OnWrenchClear(const msgs::Entity &_msg);

  /// \brief True if a console message should be printed whenever an
  /// instantaneous wrench is applied, a persistent wrench is cleared, etc.
  public: bool verbose{true};

  /// \brief Queue of incoming instantaneous wrenches
  public: std::queue<msgs::EntityWrench> newWrenches;

  /// \brief All persistent wrenches
  public: std::vector<msgs::EntityWrench> persistentWrenches;

  /// \brief Entities whose wrenches should be cleared
  public: std::queue<msgs::Entity> clearWrenches;

  /// \brief Communication node.
  public: transport::Node node;

  /// \brief A mutex to protect wrenches
  public: std::mutex mutex;
};

/// \brief Extract wrench information from a message.
/// \param[in] _ecm Entity component manager
/// \param[in] _msg Entity message. If it's a link, that link is returned. If
/// it's a model, its canonical link is returned.
/// \param[out] Force to apply.
/// \param[out] Offset of the force application point expressed in the link
/// frame.
/// \param[out] Torque to apply.
/// \param[out] Offset of the force application point expressed in the link
/// frame.
/// \return Target link entity.
Link decomposeMessage(const EntityComponentManager &_ecm,
    const msgs::EntityWrench &_msg, math::Vector3d &_force,
    math::Vector3d &_torque, math::Vector3d &_offset)
{
  if (_msg.wrench().has_force_offset())
  {
    _offset = msgs::Convert(_msg.wrench().force_offset());
  }

  if (_msg.wrench().has_force())
  {
    _force = msgs::Convert(_msg.wrench().force());
  }

  if (_msg.wrench().has_torque())
  {
    _torque = msgs::Convert(_msg.wrench().torque());
  }

  auto entity = entityFromMsg(_ecm, _msg.entity());
  if (entity == kNullEntity)
  {
    return Link();
  }

  Link link(entity);
  if (link.Valid(_ecm))
  {
    return link;
  }

  Model model(entity);
  if (model.Valid(_ecm))
  {
    return Link(model.CanonicalLink(_ecm));
  }

  gzerr << "Wrench can only be applied to a link or a model. Entity ["
        << entity << "] isn't either of them." << std::endl;
  return Link();
}

//////////////////////////////////////////////////
ApplyLinkWrench::ApplyLinkWrench()
  : dataPtr(std::make_unique<ApplyLinkWrenchPrivate>())
{
}

//////////////////////////////////////////////////
void ApplyLinkWrench::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto world = World(_entity);
  if (!world.Valid(_ecm))
  {
    gzerr << "ApplyLinkWrench system should be attached to a world."
          << std::endl;
    return;
  }

  this->dataPtr->verbose = _sdf->Get<bool>("verbose", true).first;

  // Initial wrenches
  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  for (auto elem = ptr->GetElement("persistent");
       elem != nullptr;
       elem = elem->GetNextElement("persistent"))
  {
    msgs::EntityWrench msg;
    if (!elem->HasElement("entity_name") || !elem->HasElement("entity_type"))
    {
      gzerr << "Skipping <persistent> element missing entity name or type."
            << std::endl;
      continue;
    }

    msg.mutable_entity()->set_name(elem->Get<std::string>("entity_name"));

    auto typeStr = elem->GetElement("entity_type")->Get<std::string>();
    if (typeStr == "link")
    {
      msg.mutable_entity()->set_type(msgs::Entity::LINK);
    }
    else if (typeStr == "model")
    {
      msg.mutable_entity()->set_type(msgs::Entity::MODEL);
    }
    else
    {
      gzerr << "Skipping <persistent> element, entity type [" << typeStr
            << "] not supported." << std::endl;
      continue;
    }

    if (elem->HasElement("force"))
    {
      msgs::Set(msg.mutable_wrench()->mutable_force(),
          elem->GetElement("force")->Get<math::Vector3d>());
    }
    if (elem->HasElement("torque"))
    {
      msgs::Set(msg.mutable_wrench()->mutable_torque(),
          elem->GetElement("torque")->Get<math::Vector3d>());
    }
    this->dataPtr->OnWrenchPersistent(msg);
  }

  // Topic to apply wrench for one time step
  // TODO(chapulina) Use AsValidTopic when merging forward
  std::string topic{"/world/" + world.Name(_ecm).value() + "/wrench"};
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  this->dataPtr->node.Subscribe(topic, &ApplyLinkWrenchPrivate::OnWrench,
      this->dataPtr.get());

  gzmsg << "Listening to instantaneous wrench commands in [" << topic << "]"
        << std::endl;

  // Topic to apply wrench continuously
  topic = "/world/" + world.Name(_ecm).value() + "/wrench/persistent";
  if (_sdf->HasElement("topic_persistent"))
    topic = _sdf->Get<std::string>("topic_persistent");

  this->dataPtr->node.Subscribe(topic,
      &ApplyLinkWrenchPrivate::OnWrenchPersistent, this->dataPtr.get());

  gzmsg << "Listening to persistent wrench commands in [" << topic << "]"
        << std::endl;

  // Topic to clear persistent wrenches
  topic = "/world/" + world.Name(_ecm).value() + "/wrench/clear";
  if (_sdf->HasElement("topic_clear"))
    topic = _sdf->Get<std::string>("topic_clear");

  this->dataPtr->node.Subscribe(topic,
      &ApplyLinkWrenchPrivate::OnWrenchClear, this->dataPtr.get());

  gzmsg << "Listening to wrench clear commands in [" << topic << "]"
        << std::endl;
}

//////////////////////////////////////////////////
void ApplyLinkWrench::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ApplyLinkWrench::PreUpdate");

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Clear persistent wrenches
  while (!this->dataPtr->clearWrenches.empty())
  {
    auto clearMsg = this->dataPtr->clearWrenches.front();
    auto clearEntity = entityFromMsg(_ecm, clearMsg);

    for (auto msgIt = this->dataPtr->persistentWrenches.begin();
         msgIt != this->dataPtr->persistentWrenches.end(); msgIt++)
    {
      auto persistentEntity = entityFromMsg(_ecm, msgIt->entity());
      if (persistentEntity == clearEntity)
      {
        this->dataPtr->persistentWrenches.erase(msgIt--);

        if (this->dataPtr->verbose)
        {
          gzdbg << "Clearing persistent wrench for entity [" << clearEntity
                << "]" << std::endl;
        }
      }
    }

    this->dataPtr->clearWrenches.pop();
  }

  // Only apply wrenches when not paused
  if (_info.paused)
    return;

  // Apply instantaneous wrenches
  while (!this->dataPtr->newWrenches.empty())
  {
    auto msg = this->dataPtr->newWrenches.front();

    math::Vector3d force;
    math::Vector3d offset;
    math::Vector3d torque;
    auto link = decomposeMessage(_ecm, msg, force, torque, offset);
    if (!link.Valid(_ecm))
    {
      gzerr << "Entity not found." << std::endl
            << msg.DebugString() << std::endl;
      this->dataPtr->newWrenches.pop();
      continue;
    }

    link.AddWorldWrench(_ecm, force, torque, offset);

    if (this->dataPtr->verbose)
    {
      gzdbg << "Applying wrench [" << force << " " << torque
            << "] with force offset [" << offset << "] to entity ["
            << link.Entity() << "] for 1 time step." << std::endl;
    }

    this->dataPtr->newWrenches.pop();
  }

  // Apply persistent wrenches at every time step
  for (auto msg : this->dataPtr->persistentWrenches)
  {
    math::Vector3d force;
    math::Vector3d offset;
    math::Vector3d torque;
    auto link = decomposeMessage(_ecm, msg, force, torque, offset);
    if (!link.Valid(_ecm))
    {
      // Not an error, persistent wrenches can be applied preemptively before
      // an entity is inserted
      continue;
    }
    link.AddWorldWrench(_ecm, force, torque, offset);
  }
}

//////////////////////////////////////////////////
void ApplyLinkWrenchPrivate::OnWrench(const msgs::EntityWrench &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!_msg.has_entity() || !_msg.has_wrench())
  {
    gzerr << "Missing entity or wrench in message: " << std::endl
          << _msg.DebugString() << std::endl;
    return;
  }

  this->newWrenches.push(_msg);
}

//////////////////////////////////////////////////
void ApplyLinkWrenchPrivate::OnWrenchPersistent(const msgs::EntityWrench &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!_msg.has_entity() || !_msg.has_wrench())
  {
    gzerr << "Missing entity or wrench in message: " << std::endl
          << _msg.DebugString() << std::endl;
    return;
  }

  if (this->verbose)
  {
    gzdbg << "Queueing persistent wrench:" << std::endl
          << _msg.DebugString() << std::endl;
  }

  this->persistentWrenches.push_back(_msg);
}

//////////////////////////////////////////////////
void ApplyLinkWrenchPrivate::OnWrenchClear(const msgs::Entity &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  this->clearWrenches.push(_msg);
}

GZ_ADD_PLUGIN(ApplyLinkWrench,
                    System,
                    ApplyLinkWrench::ISystemConfigure,
                    ApplyLinkWrench::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ApplyLinkWrench,
                    "gz::sim::systems::ApplyLinkWrench")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(ApplyLinkWrench,
                    "ignition::gazebo::systems::ApplyLinkWrench")

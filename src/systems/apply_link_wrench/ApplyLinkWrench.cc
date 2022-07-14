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
#include <ignition/msgs/entity_wrench.pb.h>

#include <mutex>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/World.hh"
#include "ignition/gazebo/Util.hh"

#include "ApplyLinkWrench.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::ApplyLinkWrenchPrivate
{
  /// \brief Callback for wrench subscription
  /// \param[in] _msg Wrench message
  public: void OnWrench(const msgs::EntityWrench &_msg);

  /// \brief World entity
  public: World world{kNullEntity};

  /// \brief Queue of incoming wrenches
  public: std::queue<msgs::EntityWrench> newWrenches;

  /// \brief Communication node.
  public: transport::Node node;

  /// \brief A mutex to protect the new wrenches
  public: std::mutex mutex;
};

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
  // Store the world.
  this->dataPtr->world = World(_entity);

  // TODO initial wrench

  // TODO(chapulina) Use AsValidTopic when merging forward
  std::string topic{"/world/" + this->dataPtr->world.Name(_ecm).value() +
      "/wrench"};
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  this->dataPtr->node.Subscribe(topic, &ApplyLinkWrenchPrivate::OnWrench,
      this->dataPtr.get());
}

//////////////////////////////////////////////////
void ApplyLinkWrench::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("ApplyLinkWrench::PreUpdate");

  // Only update if not paused.
  if (_info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  while (!this->dataPtr->newWrenches.empty())
  {
    auto wrenchMsg = this->dataPtr->newWrenches.front();

    auto entity = entityFromMsg(_ecm, wrenchMsg.entity());

    // Keep trying if entity doesn't exist yet?
    if (entity == kNullEntity)
    {
      ignerr << "Entity not found. Failed to apply wrench." << std::endl
             << wrenchMsg.entity().DebugString() << std::endl;
      this->dataPtr->newWrenches.pop();
      continue;
    }

    Link link(entity);
    if (!link.Valid(_ecm))
    {
      Model model(entity);
      if (!model.Valid(_ecm))
      {
        ignerr << "Wrench can only be applied to a link or a model. Entity ["
               << entity << "] isn't either of them." << std::endl;
        this->dataPtr->newWrenches.pop();
        continue;
      }
      link = Link(model.CanonicalLink(_ecm));
    }

    if (wrenchMsg.wrench().has_force_offset())
    {
      ignwarn << "Force offset currently not supported, it will be ignored."
              << std::endl;
    }

    math::Vector3d force;
    if (wrenchMsg.wrench().has_force())
    {
      force = msgs::Convert(wrenchMsg.wrench().force());
    }

    math::Vector3d torque;
    if (wrenchMsg.wrench().has_torque())
    {
      torque = msgs::Convert(wrenchMsg.wrench().torque());
    }

    link.AddWorldWrench(_ecm, force, torque);

    this->dataPtr->newWrenches.pop();
  }
}

//////////////////////////////////////////////////
void ApplyLinkWrenchPrivate::OnWrench(const msgs::EntityWrench &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!_msg.has_entity() || !_msg.has_wrench())
  {
    ignerr << "Missing entity or wrench in message: " << std::endl
           << _msg.DebugString() << std::endl;
    return;
  }

  this->newWrenches.push(_msg);
}

IGNITION_ADD_PLUGIN(ApplyLinkWrench,
                    System,
                    ApplyLinkWrench::ISystemConfigure,
                    ApplyLinkWrench::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ApplyLinkWrench,
                          "ignition::gazebo::systems::ApplyLinkWrench")

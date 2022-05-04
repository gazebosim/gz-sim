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

#include <sdf/sdf.hh>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include "ignition/gazebo/comms/Broker.hh"
#include "ignition/gazebo/comms/MsgManager.hh"
#include "ignition/gazebo/Util.hh"
#include "PerfectComms.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::PerfectComms::Implementation
{
};

//////////////////////////////////////////////////
PerfectComms::PerfectComms()
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void PerfectComms::Load(const Entity &/*_entity*/,
    std::shared_ptr<const sdf::Element> /*_sdf*/,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
}

//////////////////////////////////////////////////
void PerfectComms::Step(
      const UpdateInfo &/*_info*/,
      const comms::Registry &_currentRegistry,
      comms::Registry &_newRegistry,
      EntityComponentManager &_ecm)
{
  // Initialize entity if needed.
  for (auto & [address, content] : _currentRegistry)
  {
    if (content.entity == kNullEntity)
    {
      auto entities = gazebo::entitiesFromScopedName(content.modelName, _ecm);
      if (entities.empty())
        continue;

      auto entityId = *(entities.begin());
      if (entityId == kNullEntity)
        continue;

      _newRegistry[address].entity = entityId;
    }
  }

  for (auto & [address, content] : _currentRegistry)
  {
    // Reference to the outbound queue for this address.
    auto &outbound = content.outboundMsgs;

    // Is the source address bound?
    auto itSrc = _currentRegistry.find(address);
    bool srcAddressBound = itSrc != _currentRegistry.end();

    // Is the source address attached to a model?
    bool srcAddressAttachedToModel =
      srcAddressBound && itSrc->second.entity != kNullEntity;

    if (srcAddressAttachedToModel)
    {
      // All these messages need to be processed.
      for (auto &msg : outbound)
      {
        // Is the destination address bound?
        auto itDst = _currentRegistry.find(msg->dst_address());
        bool dstAddressBound = itDst != _currentRegistry.end();

        // Is the destination address attached to a model?
        bool dstAddressAttachedToModel =
          dstAddressBound && itDst->second.entity != kNullEntity;

        if (dstAddressAttachedToModel)
          _newRegistry[msg->dst_address()].inboundMsgs.push_back(msg);
      }
    }

    // Clear the outbound queue.
    _newRegistry[address].outboundMsgs.clear();
  }
}

IGNITION_ADD_PLUGIN(PerfectComms,
                    ignition::gazebo::System,
                    comms::ICommsModel::ISystemConfigure,
                    comms::ICommsModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(PerfectComms,
                          "ignition::gazebo::systems::PerfectComms")

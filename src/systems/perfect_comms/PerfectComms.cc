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

#include <ignition/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/comms/Broker.hh"
#include "ignition/common/Profiler.hh"
#include "ignition/gazebo/Util.hh"

#include "PerfectComms.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::PerfectComms::Implementation
{
  /// \brief Broker instance.
  public: comms::Broker broker;
};

//////////////////////////////////////////////////
PerfectComms::PerfectComms()
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
  ignerr << "Broker plugin running" << std::endl;
}

//////////////////////////////////////////////////
PerfectComms::~PerfectComms()
{
  // cannot use default destructor because of dataPtr
}

//////////////////////////////////////////////////
void PerfectComms::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->broker.Start();
}

//////////////////////////////////////////////////
void PerfectComms::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("PerfectComms::PreUpdate");

  // Step the comms model.
  this->Step(_info, _ecm, this->dataPtr->broker.Data());

  // Deliver the inbound messages.
  this->dataPtr->broker.DeliverMsgs();
}

void PerfectComms::Step(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm,
      comms::MsgManager &_messageMgr)
{
  this->dataPtr->broker.Lock();

  // Check if there are new messages to process.
  auto &allData = this->dataPtr->broker.Data().Data();

  // Create a copy to modify while we iterate.
  auto allDataCopy = this->dataPtr->broker.Data().Copy();

  for (auto & [address, content] : allData)
  {
    // Reference to the outbound queue for this address.
    auto &outbound = content.outboundMsgs;

    // All these messages need to be processed.
    for (auto &msg : outbound)
      allDataCopy[msg->dst_address()].inboundMsgs.push_back(msg);

    allDataCopy[address].outboundMsgs.clear();
  }

  this->dataPtr->broker.Data().Set(allDataCopy);

  this->dataPtr->broker.Unlock();
}

IGNITION_ADD_PLUGIN(PerfectComms,
                    ignition::gazebo::System,
                    PerfectComms::ISystemConfigure,
                    PerfectComms::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(PerfectComms,
                          "ignition::gazebo::systems::PerfectComms")

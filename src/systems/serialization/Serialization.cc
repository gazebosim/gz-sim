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

#include "msgs/serialized.pb.h"

#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "Serialization.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private Serialization data class.
class ignition::gazebo::systems::SerializationPrivate
{
  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Publishes the current ECM state.
  public: transport::Node::Publisher statePub;
};

//////////////////////////////////////////////////
Serialization::Serialization()
  : System(), dataPtr(std::make_unique<SerializationPrivate>())
{
}

//////////////////////////////////////////////////
Serialization::~Serialization() = default;

//////////////////////////////////////////////////
void Serialization::Configure(const Entity &_id,
    const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &_ecm,
    EventManager &)
{
  auto worldName = _ecm.Component<components::Name>(_id)->Data();

  // Initialize transport
  std::string topic{"/world/" + worldName + "/serialized"};
  this->dataPtr->statePub =
      this->dataPtr->node.Advertise<msgs::SerializedState>(topic);
}

//////////////////////////////////////////////////
void Serialization::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &)
{
  // Handle incoming components
}

//////////////////////////////////////////////////
void Serialization::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  // Handle outgoing components

  msgs::SerializedState stateMsg;


  // Iterate over all registered components
//  auto factory = components::Factory::Instance();
//  for (auto compStr : factory->Components())
//  {
//    auto baseComp = factory->New(compStr);
//    if (nullptr == baseComp)
//    {
//      ignerr << "Failed to create component of type [" << compStr << "]"
//             << std::endl;
//    }
//
//    auto comp = static_cast<decltype()>(baseCom);
//
//    auto compTypeMsg = stateMsg.add_component_types();
//    compTypeMsg->set_type(compStr);
//
//    components::BaseComponent c = *comp.get();
//
//    _ecm.Each<decltype(comp)>(
//        [&](const Entity &_entity, const decltype(comp) _component) -> bool
//        {
//
//          return true;
//        });
//  }

  //this->dataPtr->statePub.Publish(stateMsg);
}

IGNITION_ADD_PLUGIN(Serialization, System,
  Serialization::ISystemConfigure,
  Serialization::ISystemPreUpdate,
  Serialization::ISystemPostUpdate
)

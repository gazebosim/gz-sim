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

#include <string>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "CommsEndpoint.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::CommsEndpoint::Implementation
{
  /// \brief The address.
  public: std::string address;

  /// \brief The topic where the messages will be delivered.
  public: std::string topic;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief True when the address has been bound in the broker.
  public: bool bound{false};

  /// \brief Service where the broker is listening bind requests.
  public: std::string bindSrv = "/broker/bind";

  /// \brief Service where the broker is listening unbind requests.
  public: std::string unbindSrv = "/broker/unbind";

  /// \brief The ignition transport node.
  public: ignition::transport::Node node;
};


//////////////////////////////////////////////////
CommsEndpoint::CommsEndpoint()
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
CommsEndpoint::~CommsEndpoint()
{
  if (!this->dataPtr->bound)
    return;

  // Prepare the unbind parameters.
  ignition::msgs::StringMsg_V unbindReq;
  unbindReq.add_data(this->dataPtr->address);
  unbindReq.add_data(this->dataPtr->topic);

  // Unbind.
  if (!this->dataPtr->node.Request("/broker/unbind", unbindReq))
    ignerr << "Bind call failed" << std::endl;

}

//////////////////////////////////////////////////
void CommsEndpoint::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  // Parse <address>.
  if (!_sdf->HasElement("address"))
  {
    ignerr << "No <address> specified." << std::endl;
    return;
  }
  this->dataPtr->address = _sdf->Get<std::string>("address");

  // Parse <topic>.
  if (!_sdf->HasElement("topic"))
  {
    ignerr << "No <topic> specified." << std::endl;
    return;
  }
  this->dataPtr->topic = _sdf->Get<std::string>("topic");

  if (_sdf->HasElement("broker"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("broker");
    this->dataPtr->bindSrv =
      elem->Get<std::string>("bind_service", this->dataPtr->bindSrv).first;
    this->dataPtr->unbindSrv =
      elem->Get<std::string>("unbind_service", this->dataPtr->unbindSrv).first;
  }

  // Set model.
  this->dataPtr->model = Model(_entity);
}

//////////////////////////////////////////////////
void CommsEndpoint::PreUpdate(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("CommsEndpoint::PreUpdate");

  if (this->dataPtr->bound)
    return;

  // Prepare the bind parameters.
  ignition::msgs::StringMsg_V bindReq;
  bindReq.add_data(this->dataPtr->address);
  bindReq.add_data(this->dataPtr->model.Name(_ecm));
  bindReq.add_data(this->dataPtr->topic);

  // Bind.
  if (this->dataPtr->node.Request("/broker/bind", bindReq))
    this->dataPtr->bound = true;
  else
    ignerr << "Bind call failed" << std::endl;
}

IGNITION_ADD_PLUGIN(CommsEndpoint,
                    ignition::gazebo::System,
                    CommsEndpoint::ISystemConfigure,
                    CommsEndpoint::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(CommsEndpoint,
                          "ignition::gazebo::systems::CommsEndpoint")

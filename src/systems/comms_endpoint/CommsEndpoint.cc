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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <atomic>
#include <chrono>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "CommsEndpoint.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::CommsEndpoint::Implementation
{
  /// \brief Send the bind request.
  public: void Bind();

  /// \brief Service response callback.
  /// \brief \param[in] _rep Unused.
  /// \brief \param[in] _result Bind result.
  public: void BindCallback(const gz::msgs::Boolean &_rep,
                            const bool _result);

  /// \brief The address.
  public: std::string address;

  /// \brief The topic where the messages will be delivered.
  public: std::string topic;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief True when the address has been bound in the broker.
  public: std::atomic_bool bound{false};

  /// \brief Service where the broker is listening bind requests.
  public: std::string bindSrv = "/broker/bind";

  /// \brief Service where the broker is listening unbind requests.
  public: std::string unbindSrv = "/broker/unbind";

  /// \brief Message to send the bind request.
  public: gz::msgs::StringMsg_V bindReq;

  /// \brief Message to send the unbind request.
  public: gz::msgs::StringMsg_V unbindReq;

  /// \brief Time between bind retries (secs).
  public: std::chrono::steady_clock::duration bindRequestPeriod{1};

  /// \brief Last simulation time we tried to bind.
  public: std::chrono::steady_clock::duration lastBindRequestTime{-2};

  /// \brief The Gazebo Transport node.
  public: std::unique_ptr<gz::transport::Node> node;
};

//////////////////////////////////////////////////
void CommsEndpoint::Implementation::BindCallback(
  const gz::msgs::Boolean &/*_rep*/, const bool _result)
{
  if (_result)
    this->bound = true;

  gzdbg << "Succesfuly bound to [" << this->address << "] on topic ["
         << this->topic << "]" << std::endl;
}

//////////////////////////////////////////////////
void CommsEndpoint::Implementation::Bind()
{
  this->node->Request(this->bindSrv, this->bindReq,
    &CommsEndpoint::Implementation::BindCallback, this);
}

//////////////////////////////////////////////////
CommsEndpoint::CommsEndpoint()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->node = std::make_unique<gz::transport::Node>();
}

//////////////////////////////////////////////////
CommsEndpoint::~CommsEndpoint()
{
  if (!this->dataPtr->bound)
    return;

  // Unbind.
  // We use a oneway request because we're not going
  // to be alive to check the result or retry.
  this->dataPtr->node->Request(
    this->dataPtr->unbindSrv, this->dataPtr->unbindReq);
}

//////////////////////////////////////////////////
void CommsEndpoint::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Parse <address>.
  if (!_sdf->HasElement("address"))
  {
    gzerr << "No <address> specified." << std::endl;
    return;
  }
  this->dataPtr->address = _sdf->Get<std::string>("address");

  // Parse <topic>.
  if (!_sdf->HasElement("topic"))
  {
    gzerr << "No <topic> specified." << std::endl;
    return;
  }
  this->dataPtr->topic = _sdf->Get<std::string>("topic");

  // Parse <broker>.
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

  // Prepare the bind parameters.
  this->dataPtr->bindReq.add_data(this->dataPtr->address);
  this->dataPtr->bindReq.add_data(this->dataPtr->model.Name(_ecm));
  this->dataPtr->bindReq.add_data(this->dataPtr->topic);

  // Prepare the unbind parameters.
  this->dataPtr->unbindReq.add_data(this->dataPtr->address);
  this->dataPtr->unbindReq.add_data(this->dataPtr->topic);
}

//////////////////////////////////////////////////
void CommsEndpoint::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
  GZ_PROFILE("CommsEndpoint::PreUpdate");

  if (this->dataPtr->bound)
    return;

  auto elapsed = _info.simTime - this->dataPtr->lastBindRequestTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->dataPtr->bindRequestPeriod)
  {
    return;
  }
  this->dataPtr->lastBindRequestTime = _info.simTime;

  // Let's try to bind.
  this->dataPtr->Bind();
}

GZ_ADD_PLUGIN(CommsEndpoint,
                    gz::sim::System,
                    CommsEndpoint::ISystemConfigure,
                    CommsEndpoint::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(CommsEndpoint,
                          "gz::sim::systems::CommsEndpoint")

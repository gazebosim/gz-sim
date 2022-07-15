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

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>

#include <gz/common/Profiler.hh>
#include <sdf/Element.hh>
#include "gz/sim/comms/Broker.hh"
#include "gz/sim/comms/ICommsModel.hh"
#include "gz/sim/comms/MsgManager.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"

using namespace gz;
using namespace sim;
using namespace comms;

/// \brief Private ICommsModel data class.
class gz::sim::comms::ICommsModel::Implementation
{
  /// \brief Broker instance.
  public: Broker broker;

  /// \brief The step size for each step iteration.
  public: std::optional<std::chrono::steady_clock::duration>
    timeStep = std::nullopt;

  /// \brief Current time.
  public: std::chrono::steady_clock::time_point currentTime;
};

//////////////////////////////////////////////////
ICommsModel::ICommsModel()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void ICommsModel::Configure(const Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       EntityComponentManager &_ecm,
                       EventManager &_eventMgr)
{
  // Parse the optional <step_size>.
  if (_sdf->HasElement("step_size"))
  {
    this->dataPtr->timeStep = std::chrono::duration<int64_t, std::nano>(
      static_cast<int64_t>(_sdf->Get<double>("step_size") * 1e9));
  }
  this->Load(_entity, _sdf, _ecm, _eventMgr);
  this->dataPtr->broker.Load(_sdf);
  this->dataPtr->broker.Start();
}

//////////////////////////////////////////////////
void ICommsModel::PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm)
{
    GZ_PROFILE("ICommsModel::PreUpdate");

    if (_info.paused)
      return;

    this->dataPtr->currentTime =
      std::chrono::steady_clock::time_point(_info.simTime);

    if (!this->dataPtr->timeStep.has_value())
    {
      // If no step_size is defined simply execute one step of the comms model
      this->StepImpl(_info, _ecm);
    }
    else
    {
      // Otherwise step at the specified time step until we converge on the
      // final timestep. If the timestep is larger than the dt, then dt will
      // be used.
      auto endTime = this->dataPtr->currentTime + _info.dt;

      while (this->dataPtr->currentTime < endTime)
      {
        gz::sim::UpdateInfo info(_info);
        info.dt = std::min(this->dataPtr->timeStep.value(), _info.dt);
        info.simTime = this->dataPtr->currentTime.time_since_epoch();
        this->StepImpl(_info, _ecm);
        this->dataPtr->currentTime += info.dt;
      }
    }
}

//////////////////////////////////////////////////
void ICommsModel::StepImpl(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm)
{
  // We lock while we manipulate data.
  this->dataPtr->broker.Lock();

  // Update the time in the broker.
  this->dataPtr->broker.SetTime(_info.simTime);

  // Step the comms model.
  const Registry &currentRegistry =
    this->dataPtr->broker.DataManager().DataConst();
  Registry newRegistry = this->dataPtr->broker.DataManager().Copy();
  this->Step(_info, currentRegistry, newRegistry, _ecm);
  this->dataPtr->broker.DataManager().Set(newRegistry);

  this->dataPtr->broker.Unlock();

  // Deliver the inbound messages.
  this->dataPtr->broker.DeliverMsgs();
}

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

#ifndef IGNITION_GAZEBO_COMMSMODEL_HH_
#define IGNITION_GAZEBO_COMMSMODEL_HH_

#include <sdf/sdf.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/comms/MsgManager.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace comms
{
  /// \brief Abstract interface to define how the environment should handle
  /// communication simulation. This class should be responsible for
  /// handling dropouts, decay and packet collisions.
  class ICommsModel
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override
            {
              this->Load(_entity, _sdf, _ecm, _eventMgr);
              this->broker.Start();
            }

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override
            {
              // Step the comms model.
              this->Step(_info, _ecm, this->broker.Data());

              // Deliver the inbound messages.
              this->broker.DeliverMsgs();
            }

    /// \brief This method is called when the system is being configured
    /// override this to load any additional params for the comms model
    /// \param[in] _entity The entity this plugin is attached to.
    /// \param[in] _sdf The SDF Element associated with this system plugin.
    /// \param[in] _ecm The EntityComponentManager of the given simulation
    /// instance.
    /// \param[in] _eventMgr The EventManager of the given simulation
    /// instance.
    public: virtual void Load(const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) = 0;

    /// \brief This method is called when there is a timestep in the simulator
    /// override this to update your data structures as needed.
    /// \param[in] _info - Simulator information about the current timestep.
    /// \param[in] _ecm - Ignition's ECM.
    /// \param[in] _messageMgr - Use this to mark the message as arrived.
    public: virtual void Step(const UpdateInfo &_info,
      EntityComponentManager &_ecm, MsgManager &_messageMgr) = 0;

    /// \brief Destructor
    public: virtual ~ICommsModel() = default;

    /// \brief Broker instance
    public: Broker broker;
  };
}
}
}
}

#endif

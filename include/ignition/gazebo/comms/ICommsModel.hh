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

#ifndef IGNITION_GAZEBO_ICOMMSMODEL_HH_
#define IGNITION_GAZEBO_ICOMMSMODEL_HH_

#include <algorithm>
#include <memory>

#include <sdf/sdf.hh>
#include <ignition/common/Profiler.hh>
#include "ignition/gazebo/comms/Broker.hh"
#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace comms
{
  // Forward declarations.
  class MsgManager;

  /// \brief Abstract interface to define how the environment should handle
  /// communication simulation. As an example, this class could be responsible
  /// for handling dropouts, decay and packet collisions.
  ///
  /// The derived comms models can be configured with the following SDF
  /// parameters:
  ///
  /// * Optional parameters:
  /// <step_size> If defined this will allow the comms model to run at a
  /// higher frequency than the physics engine. This is useful when dealing
  /// with ranging. If the <step_size> is set larger than the physics engine dt
  /// then the comms model step size will default to dt.
  /// Note: for consistency it is adviced that the dt is a multiple of timestep.
  /// Units are in seconds.
  ///    <messages_topic>: Topic name where the broker receives all the incoming
  ///                      messages. The default value is "/broker/msgs"
  ///    <bind_service>: Service name used to bind an address.
  ///                    The default value is "/broker/bind"
  ///    <unbind_service>: Service name used to unbind from an address.
  ///                      The default value is "/broker/unbind"
  ///
  /// Here's an example:
  /// <physics name="1ms" type="ignored">
  ///   <max_step_size>2</max_step_size>
  ///   <real_time_factor>1.0</real_time_factor>
  /// </physics>
  /// <plugin
  ///   filename="ignition-gazebo-perfect-comms-system"
  ///   name="ignition::gazebo::systems::PerfectComms">
  ///   <step_size>1</step_size>
  /// </plugin>
  class ICommsModel
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Destructor.
    public: virtual ~ICommsModel() = default;

    // Documentation inherited.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override
    {
      sdf::ElementPtr sdfClone = _sdf->Clone();

      // Parse the optional <step_size>.
      if (sdfClone->HasElement("step_size"))
      {
        this->timeStep = std::chrono::duration<int64_t, std::nano>(
          static_cast<int64_t>(sdfClone->Get<double>("step_size") * 1e9));
      }

      this->Load(_entity, sdfClone, _ecm, _eventMgr);
      this->broker.Load(sdfClone);
      this->broker.Start();
    }

    // Documentation inherited.
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override
    {
      IGN_PROFILE("ICommsModel::PreUpdate");

      if (_info.paused)
        return;

      this->currentTime =
        std::chrono::steady_clock::time_point(_info.simTime);

      if (!this->timeStep.has_value())
      {
        // If no step_size is defined simply execute one step of the comms model
        this->StepImpl(_info, _ecm);
      }
      else
      {
        // Otherwise step at the specified time step until we converge on the
        // final timestep. If the timestep is larger than the dt, then dt will
        // be used.
        auto endTime = this->currentTime + _info.dt;

        while (this->currentTime < endTime)
        {
          ignition::gazebo::UpdateInfo info(_info);
          info.dt = std::min(this->timeStep.value(), _info.dt);
          info.simTime = this->currentTime.time_since_epoch();
          this->StepImpl(_info, _ecm);
          this->currentTime += info.dt;
        }
      }
    }

    /// \brief This method is called when there is a timestep in the simulator
    /// override this to update your data structures as needed.
    /// \param[in] _info Simulator information about the current timestep.
    /// \param[in] _currentRegistry The current registry.
    /// \param[in] _newRegistry The new registry. When Step() is finished this
    ///                         will become the new registry.
    /// \param[in] _ecm - Ignition's ECM.
    public: virtual void StepImpl(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm)
    {
      // We lock while we manipulate data.
      this->broker.Lock();

      // Update the time in the broker.
      this->broker.SetTime(_info.simTime);

      // Step the comms model.
      const Registry &currentRegistry = this->broker.DataManager().DataConst();
      Registry newRegistry = this->broker.DataManager().Copy();
      this->Step(_info, currentRegistry, newRegistry, _ecm);
      this->broker.DataManager().Set(newRegistry);

      this->broker.Unlock();

      // Deliver the inbound messages.
      this->broker.DeliverMsgs();
    }

    /// \brief This method is called when the system is being configured
    /// override this to load any additional params for the comms model
    /// \param[in] _entity The entity this plugin is attached to.
    /// \param[in] _sdf The SDF Element associated with this system plugin.
    /// \param[in] _ecm The EntityComponentManager of the given simulation
    /// instance.
    /// \param[in] _eventMgr The EventManager of the given simulation instance.
    public: virtual void Load(const Entity &_entity,
                              std::shared_ptr<const sdf::Element> _sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr) = 0;

    /// \brief This method is called when there is a timestep in the simulator
    /// override this to update your data structures as needed.
    /// \param[in] _info Simulator information about the current timestep.
    /// \param[in] _currentRegistry The current registry.
    /// \param[in] _newRegistry The new registry. When Step() is finished this
    ///                         will become the new registry.
    /// \param[in] _ecm - Ignition's ECM.
    public: virtual void Step(const UpdateInfo &_info,
                              const Registry &_currentRegistry,
                              Registry &_newRegistry,
                              EntityComponentManager &_ecm) = 0;

    /// \brief Broker instance.
    protected: Broker broker;

    /// \brief The step size for each step iteration.
    protected: std::optional<std::chrono::steady_clock::duration>
      timeStep = std::nullopt;

    /// \brief Current time.
    protected: std::chrono::steady_clock::time_point currentTime;
  };
}
}
}
}

#endif

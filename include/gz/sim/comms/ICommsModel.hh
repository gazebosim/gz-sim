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

#ifndef GZ_SIM_ICOMMSMODEL_HH_
#define GZ_SIM_ICOMMSMODEL_HH_

#include <memory>

#include <gz/utils/ImplPtr.hh>
#include <sdf/Element.hh>
#include "gz/sim/comms/MsgManager.hh"
#include "gz/sim/config.hh"
#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {

  // Forward declarations
  class EntityComponentManager;
  class EventManager;

namespace comms
{
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
  ///
  /// Here's an example:
  /// <physics name="1ms" type="ignored">
  ///   <max_step_size>2</max_step_size>
  ///   <real_time_factor>1.0</real_time_factor>
  /// </physics>
  /// <plugin
  ///   filename="gz-sim-perfect-comms-system"
  ///   name="gz::sim::systems::PerfectComms">
  ///   <step_size>1</step_size>
  /// </plugin>
  class GZ_SIM_VISIBLE ICommsModel:
#ifdef _MSC_VER
  #pragma warning(push)
  #pragma warning(disable:4275)
#endif
        public System,
#ifdef _MSC_VER
  #pragma warning(pop)
#endif
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor.
    public: explicit ICommsModel();

    // Documentation inherited.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief This method is called when there is a timestep in the simulator.
    /// \param[in] _info Simulator information about the current timestep.
    ///                         will become the new registry.
    /// \param[in] _ecm - Gazebo Sim's ECM.
    public: virtual void StepImpl(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm);

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
    ///
    /// Note: this is an experimental interface and might change in the future.
    ///
    /// \param[in] _info Simulator information about the current timestep.
    /// \param[in] _currentRegistry The current registry.
    /// \param[out] _newRegistry The new registry. When Step() is finished this
    ///                         will become the new registry.
    /// \param[in] _ecm - Gazebo Sim's ECM.
    public: virtual void Step(const UpdateInfo &_info,
                              const Registry &_currentRegistry,
                              Registry &_newRegistry,
                              EntityComponentManager &_ecm) = 0;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
}
}
}

#endif

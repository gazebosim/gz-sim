/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEM_HH_
#define GZ_SIM_SYSTEM_HH_

#include <cstdint>
#include <memory>

#include <gz/sim/config.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/Types.hh>

#include <gz/transport/parameters/Registry.hh>

#include <sdf/Element.hh>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    /// \brief Namespace for all System plugins. Refer to the System class for
    /// more information about systems.
    namespace systems {}

    /// \class System System.hh gz/sim/System.hh
    /// \brief Base class for a System.
    ///
    /// A System operates on Entities that have certain Components. A System
    /// will only operate on an Entity if it has all of the required
    /// Components.
    ///
    /// Systems are executed in three phases, with each phase for a given step
    /// corresponding to the entities at time UpdateInfo::simTime:
    ///  * PreUpdate
    ///    * Has read-write access to world entities and components.
    ///    * This is where systems say what they'd like to happen at time
    ///      UpdateInfo::simTime.
    ///    * Can be used to modify state before physics runs, for example for
    ///      applying control signals or performing network syncronization.
    ///  * Update
    ///    * Has read-write access to world entities and components.
    ///    * Used for physics simulation step (i.e., simulates what happens at
    ///      time UpdateInfo::simTime).
    ///  * PostUpdate
    ///    * Has read-only access to world entities and components.
    ///    * Captures everything that happened at time UpdateInfo::simTime.
    ///    * Used to read out results at the end of a simulation step to be used
    ///      for sensor or controller updates.
    ///
    /// The PreUpdate and Update phases are executed sequentially in the same
    /// thread, while the PostUpdate phase is executed in parallel in multiple
    /// threads. The order of execution of PreUpdate and Update phases can be
    /// controlled by specifying a signed integer Priority value for the System
    /// in its XML configuration. The default Priority value is zero, and
    /// smaller values are executed earlier. Systems with the same Priority
    /// value are executed in the order in which they are loaded.
    ///
    /// It's important to note that UpdateInfo::simTime does not refer to the
    /// current time, but the time reached after the PreUpdate and Update calls
    /// have finished. So, if any of the *Update functions are called with
    /// simulation paused, time does not advance, which means the time reached
    /// after PreUpdate and Update is the same as the starting time. This
    /// explains why UpdateInfo::simTime is initially 0 if simulation is started
    /// paused, while UpdateInfo::simTime is initially UpdateInfo::dt if
    /// simulation is started un-paused.
    class System
    {
      /// \brief Signed integer type used for specifying priority of the
      /// execution order of PreUpdate and Update phases.
      public: using PriorityType = int32_t;

      /// \brief Default priority value for execution order of the PreUpdate
      /// and Update phases.
      public: constexpr static PriorityType kDefaultPriority = {0};

      /// \brief Name of the XML element from which the priority value will be
      /// parsed.
      public: constexpr static std::string_view kPriorityElementName =
          {"gz:system_priority"};

      /// \brief Constructor
      public: System() = default;

      /// \brief Destructor
      public: virtual ~System() = default;
    };

    /// \class ISystemConfigure ISystem.hh gz/sim/System.hh
    /// \brief Interface for a system that implements optional configuration
    ///
    /// Configure is called after the system is instantiated and all entities
    /// and components are loaded from the corresponding SDF world, and before
    /// simulation begins exectution.
    class ISystemConfigure {
      /// \brief Configure the system
      /// \param[in] _entity The entity this plugin is attached to.
      /// \param[in] _sdf The SDF Element associated with this system plugin.
      /// \param[in] _ecm The EntityComponentManager of the given simulation
      /// instance.
      /// \param[in] _eventMgr The EventManager of the given simulation
      /// instance.
      public: virtual void Configure(
                  const Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  EntityComponentManager &_ecm,
                  EventManager &_eventMgr) = 0;
    };

    /// \class ISystemConfigure ISystem.hh gz/sim/System.hh
    /// \brief Interface for a system that implements optional configuration
    /// of the default priority value.
    ///
    /// ConfigurePriority is called before the system is instantiated to
    /// override System::kDefaultPriority. It can still be overridden by the
    /// XML priority element.
    class ISystemConfigurePriority {
      /// \brief Configure the default priority of the system, which can still
      /// be overridden by the XML priority element.
      /// \return The default priority for the system.
      public: virtual System::PriorityType ConfigurePriority() = 0;
    };

    /// \class ISystemConfigureParameters ISystem.hh gz/sim/System.hh
    /// \brief Interface for a system that declares parameters.
    ///
    /// ISystemConfigureParameters::ConfigureParameters is called after
    /// ISystemConfigure::Configure.
    class ISystemConfigureParameters {
      /// \brief Configure the parameters of the system.
      /// \param[in] _registry The parameter registry.
      public: virtual void ConfigureParameters(
                  gz::transport::parameters::ParametersRegistry &_registry,
                  EntityComponentManager &_ecm) = 0;
    };


    class ISystemReset {
      public: virtual void Reset(const UpdateInfo &_info,
                                 EntityComponentManager &_ecm) = 0;
    };

    /// \class ISystemPreUpdate ISystem.hh gz/sim/System.hh
    /// \brief Interface for a system that uses the PreUpdate phase
    class ISystemPreUpdate {
      public: virtual void PreUpdate(const UpdateInfo &_info,
                                     EntityComponentManager &_ecm) = 0;
    };

    /// \class ISystemUpdate ISystem.hh gz/sim/System.hh
    /// \brief Interface for a system that uses the Update phase
    class ISystemUpdate {
      public: virtual void Update(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm) = 0;
    };

    /// \class ISystemPostUpdate ISystem.hh gz/sim/System.hh
    /// \brief Interface for a system that uses the PostUpdate phase
    class ISystemPostUpdate{
      public: virtual void PostUpdate(const UpdateInfo &_info,
                                      const EntityComponentManager &_ecm) = 0;
    };
  }
  }
}
#endif

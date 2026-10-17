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
#include <string>
#include <string_view>
#include <vector>

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
    ///      applying control signals or performing network synchronization.
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

      /// \brief Name of the XML element from which the physics requirements
      /// of a system will be parsed. See ISystemPhysicsRequirements.
      public: constexpr static std::string_view
          // cppcheck-suppress unusedStructMember
          kPhysicsRequirementsElementName = {"gz:physics_requirements"};

      /// \brief Constructor
      public: System() = default;

      /// \brief Destructor
      public: virtual ~System() = default;
    };

    /// \brief Define constant priority values for specific systems.
    namespace systems
    {
      /// \brief Default priority value for the UserCommands system, with a very
      /// negative value to indicate that it should run before most systems in
      /// order to modify the ECM in response to user commands received over
      /// gz-transport. This is especially important for user commands that add
      /// objects to the world.
      constexpr System::PriorityType kUserCommandsPriority = -16384;

      /// \brief A suggested priority value for a system that should execute
      /// before the Physics system.
      constexpr System::PriorityType kPrePhysicsPriority = -128;

      /// \brief Default priority value for the Physics system, with a negative
      /// value ensuring that it will run before systems with priority
      /// System::kDefaultPriority.
      constexpr System::PriorityType kPhysicsPriority = -64;

      /// \brief A suggested priority value for sensor systems that should
      /// execute after the Physics system but before the systems with priority
      /// System::kDefaultPriority.
      constexpr System::PriorityType kPostPhysicsSensorPriority = -32;
    }

    /// \class ISystemConfigure System.hh gz/sim/System.hh
    /// \brief Interface for a system that implements optional configuration
    ///
    /// Configure is called after the system is instantiated and all entities
    /// and components are loaded from the corresponding SDF world, and before
    /// simulation begins execution.
    class ISystemConfigure {
      public: virtual ~ISystemConfigure() = default;

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

    /// \class ISystemConfigure System.hh gz/sim/System.hh
    /// \brief Interface for a system that implements optional configuration
    /// of the default priority value.
    ///
    /// ConfigurePriority is called before the system is instantiated to
    /// override System::kDefaultPriority. It can still be overridden by the
    /// XML priority element.
    class ISystemConfigurePriority {
      public: virtual ~ISystemConfigurePriority() = default;

      /// \brief Configure the default priority of the system, which can still
      /// be overridden by the XML priority element.
      /// \return The default priority for the system.
      public: virtual System::PriorityType ConfigurePriority() = 0;
    };

    /// \class ISystemPhysicsRequirements System.hh gz/sim/System.hh
    /// \brief Interface for a system that needs features from the physics
    /// provider that simulates its entity.
    ///
    /// A physics provider is a system that simulates the dynamics of models,
    /// such as the Physics system. Providers can differ in what they simulate.
    /// A model can list the features that its provider supports in the
    /// SupportedPhysicsFeatures component. A system that depends on a feature,
    /// for example because it writes a command that only some providers
    /// consume, lists the feature here.
    ///
    /// A system with a requirement that its entity's provider does not meet
    /// is not run. A warning names the system, the model (or the world), and
    /// the missing feature. The entity keeps its physics provider, and other
    /// systems on the entity are not affected. If no SupportedPhysicsFeatures
    /// component applies to the entity, every requirement is met.
    ///
    /// A system attached to the world is checked against the world's
    /// SupportedPhysicsFeatures component, not against each model's. A world
    /// system that serves many models should not declare requirements. It
    /// should call physicsSupports() for each model instead.
    ///
    /// The requirements are queried each time the system is activated, after
    /// Configure. That includes the activation that follows a reset. Configure
    /// has already run by then. Its effects on the ECM stay, the parameters it
    /// declared stay registered, and SystemPluginInfo still lists the system,
    /// even if the system is not activated. A system can therefore return
    /// requirements that depend on its parameters. PhysicsRequirements() runs
    /// while the system manager holds a lock, so it must not add systems.
    ///
    /// The requirements can also be set, and replaced, in the SDF of the
    /// plugin with a `<gz:physics_requirements>` element that holds the feature
    /// names separated by spaces. An empty element removes all requirements.
    /// This allows annotating a system that does not implement this interface.
    /// The element is read only for plugins loaded from SDF (world, model or
    /// spawn), not for systems passed to Server::AddSystem.
    ///
    /// Feature names are plain strings. Their meaning is shared between the
    /// providers that list them and the systems that require them.
    ///
    /// A plugin has to list this interface in its GZ_ADD_PLUGIN call, as it
    /// does for every other optional system interface. Otherwise the system
    /// manager can't find it, and the requirements are silently ignored.
    class ISystemPhysicsRequirements {
      public: virtual ~ISystemPhysicsRequirements() = default;

      /// \brief Get the physics features that the system needs.
      /// \return Names of the required features.
      public: virtual std::vector<std::string> PhysicsRequirements() = 0;
    };

    /// \class ISystemConfigureParameters System.hh gz/sim/System.hh
    /// \brief Interface for a system that declares parameters.
    ///
    /// ISystemConfigureParameters::ConfigureParameters is called after
    /// ISystemConfigure::Configure.
    class ISystemConfigureParameters {
      public: virtual ~ISystemConfigureParameters() = default;

      /// \brief Configure the parameters of the system.
      /// \param[in] _registry The parameter registry.
      /// \param[in] _ecm The entity component manager.
      public: virtual void ConfigureParameters(
                  gz::transport::parameters::ParametersRegistry &_registry,
                  EntityComponentManager &_ecm) = 0;
    };


    class ISystemReset {
      public: virtual ~ISystemReset() = default;
      public: virtual void Reset(const UpdateInfo &_info,
                                 EntityComponentManager &_ecm) = 0;
    };

    /// \class ISystemPreUpdate System.hh gz/sim/System.hh
    /// \brief Interface for a system that uses the PreUpdate phase
    class ISystemPreUpdate {
      public: virtual ~ISystemPreUpdate() = default;
      public: virtual void PreUpdate(const UpdateInfo &_info,
                                     EntityComponentManager &_ecm) = 0;
    };

    /// \class ISystemUpdate System.hh gz/sim/System.hh
    /// \brief Interface for a system that uses the Update phase
    class ISystemUpdate {
      public: virtual ~ISystemUpdate() = default;
      public: virtual void Update(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm) = 0;
    };

    /// \class ISystemPostUpdate System.hh gz/sim/System.hh
    /// \brief Interface for a system that uses the PostUpdate phase
    class ISystemPostUpdate{
      public: virtual ~ISystemPostUpdate() = default;
      public: virtual void PostUpdate(const UpdateInfo &_info,
                                      const EntityComponentManager &_ecm) = 0;
    };
  }
  }
}
#endif

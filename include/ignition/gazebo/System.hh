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
#ifndef IGNITION_GAZEBO_SYSTEM_HH_
#define IGNITION_GAZEBO_SYSTEM_HH_

#include <memory>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

#include <sdf/Element.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \brief Namespace for all System plugins. Refer to the System class for
    /// more information about systems.
    namespace systems {}

    /// \class System System.hh ignition/gazebo/System.hh
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
    /// It's important to note that UpdateInfo::simTime does not refer to the
    /// current time, but the time reached after the PreUpdate and Update calls
    /// have finished. So, if any of the *Update functions are called with
    /// simulation paused, time does not advance, which means the time reached
    /// after PreUpdate and Update is the same as the starting time. This
    /// explains why UpdateInfo::simTime is initially 0 if simulation is started
    /// paused, while UpdateInfo::simTime is initially UpdateInfo::dt if
    /// simulation is started un-paused.
    class IGNITION_GAZEBO_VISIBLE System
    {
      /// \brief Constructor
      public: System();

      /// \brief Destructor
      public: virtual ~System();
    };

    /// \class ISystemConfigure ISystem.hh ignition/gazebo/System.hh
    /// \brief Interface for a system that implements optional configuration
    ///
    /// Configure is called after the system is instatiated and all entities
    /// and components are loaded from the corresponding SDF world, and before
    /// simulation begins exectution.
    class IGNITION_GAZEBO_VISIBLE ISystemConfigure {
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

    /// \class ISystemPreUpdate ISystem.hh ignition/gazebo/System.hh
    /// \brief Interface for a system that uses the PreUpdate phase
    class IGNITION_GAZEBO_VISIBLE ISystemPreUpdate {
      public: virtual void PreUpdate(const UpdateInfo &_info,
                                     EntityComponentManager &_ecm) = 0;
    };

    /// \class ISystemUpdate ISystem.hh ignition/gazebo/System.hh
    /// \brief Interface for a system that uses the Update phase
    class IGNITION_GAZEBO_VISIBLE ISystemUpdate {
      public: virtual void Update(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm) = 0;
    };

    /// \class ISystemPostUpdate ISystem.hh ignition/gazebo/System.hh
    /// \brief Interface for a system that uses the PostUpdate phase
    class IGNITION_GAZEBO_VISIBLE ISystemPostUpdate{
      public: virtual void PostUpdate(const UpdateInfo &_info,
                                      const EntityComponentManager &_ecm) = 0;
    };
  }
  }
}
#endif

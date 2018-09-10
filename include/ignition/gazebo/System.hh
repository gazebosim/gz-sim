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

#include <ignition/common/Time.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \class System System.hh ignition/gazebo/System.hh
    /// \brief Base class for a System.
    ///
    /// A System operates on Entities that have certain Components. A System
    /// will only operate on an Entity if it has all of the required
    /// Components.
    class IGNITION_GAZEBO_VISIBLE System
    {
      /// \brief Constructor
      public: System();

      /// \brief Destructor
      public: virtual ~System();

      /// \brief Initialize the system.
      public: virtual void Init() = 0;

      /// \brief Called when an entity is added to the simulation.
      // //TODO(mjcarroll): Should this be filtered by matching components?
      public: virtual void EntityAdded(const Entity &_entity,
                                       const EntityComponentManager &_ecm);

      /// \brief Called when an entity is removed from the simulation.
      // //TODO(mjcarroll): Should this be filtered by matching components?
      public: virtual void EntityRemoved(const Entity &_entity,
                                         const EntityComponentManager &_ecm);

      public: virtual void PreUpdate(const UpdateInfo &_info,
                                     EntityComponentManager &_ecm);

      public: virtual void Update(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm);

      public: virtual void PostUpdate(const UpdateInfo &_info,
                                      const EntityComponentManager &_ecm);
    };
    }
  }
}
#endif

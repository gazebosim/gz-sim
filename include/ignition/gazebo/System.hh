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
#include <string>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityQueryRegistrar.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
  namespace gazebo
  {
    // Forward declarations.
    class SystemPrivate;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    /// \class System System.hh ignition/gazebo/System.hh
    /// \brief Base class for a System.
    ///
    /// A System operates on Entities that have certain Components. A System
    /// will only operate on an Entity if it has all of the required
    /// Components.
    class IGNITION_GAZEBO_VISIBLE System
    {
      /// \brief Constructor
      public: explicit System(const std::string &_name);

      /// \brief Destructor
      public: virtual ~System();

      public: virtual void Init(EntityQueryRegistrar &_registrar,
                  EntityComponentManager &_ecMgr);

      /// \brief Get the name of the system.
      public: const std::string &Name() const;

      /// \brief Set the name of the System
      public: void SetName(const std::string &_name) const;

      private: std::unique_ptr<SystemPrivate> dataPtr;
    };
    }
  }
}
#endif

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
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class SystemPrivate;

    /// \class System System.hh ignition/gazebo/System.hh
    /// \brief Base class for a System.
    ///
    /// A System operates on Entities that have certain Components. A System
    /// will only operate on an Entity if it has all of the required
    /// Components.
    class IGNITION_GAZEBO_VISIBLE System
    {
      /// \brief Constructor
      /// \param[in] A name for the system.
      public: explicit System(const std::string &_name);

      /// \brief Destructor
      public: virtual ~System();

      /// \brief Initialize the system.
      /// \param[out] _registrar A registrar which should be filled with
      /// queries and callbacks.
      public: virtual void Init(EntityQueryRegistrar &_registrar);

      /// \brief Get the name of the system.
      /// \return The name.
      public: const std::string &Name() const;

      /// \brief Set the name of the System
      /// \param[in] _name The name.
      public: void SetName(const std::string &_name) const;

      /// \brief Pointer to private data.
      private: std::unique_ptr<SystemPrivate> dataPtr;
    };
    }
  }
}
#endif

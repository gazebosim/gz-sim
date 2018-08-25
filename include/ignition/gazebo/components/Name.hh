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
#ifndef IGNITION_GAZEBO_COMPONENTS_NAME_HH_
#define IGNITION_GAZEBO_COMPONENTS_NAME_HH_

#include <memory>
#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
namespace components
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declarations.
  class NamePrivate;

  /// \brief This component holds an entity's name. The component has no concept
  /// of scoped names nor does it care about uniqueness.
  class IGNITION_GAZEBO_VISIBLE Name
  {
    /// \brief Constructor
    /// \param[in] _name The entity's name
    public: explicit Name(const std::string &_name);

    /// \brief Destructor
    public: virtual ~Name();

    /// \brief Copy Constructor
    /// \param[in] _name Name component to copy.
    public: Name(const Name &_name);

    /// \brief Move Constructor
    /// \param[in] _name Name component to move.
    public: Name(Name &&_name) noexcept;

    /// \brief Move assignment operator.
    /// \param[in] _name Name component to move.
    /// \return Reference to this.
    public: Name &operator=(Name &&_name);

    /// \brief Copy assignment operator.
    /// \param[in] _name Name component to copy.
    /// \return Reference to this.
    public: Name &operator=(const Name &_name);

    /// \brief Get the name data.
    /// \return The actual name string.
    public: const std::string &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<NamePrivate> dataPtr;
  };
  }
}
}
}
#endif

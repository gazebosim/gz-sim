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
#ifndef IGNITION_GAZEBO_COMPONENTS_INERTIAL_HH_
#define IGNITION_GAZEBO_COMPONENTS_INERTIAL_HH_

#include <memory>

#include <ignition/math/Inertial.hh>

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
  class InertialPrivate;

  /// \brief A component type that contains inertial, ignition::math::Inertiald,
  /// information.
  class IGNITION_GAZEBO_VISIBLE Inertial
  {
    /// \brief Constructor
    /// \param[in] _inertial Ignition math inertial to copy
    public: explicit Inertial(const ignition::math::Inertiald &_inertial);

    /// \brief Copy Constructor
    /// \param[in] _inertial Inertial component to copy.
    public: Inertial(const Inertial &_inertial);

    /// \brief Move Constructor
    /// \param[in] _inertial Inertial component to move.
    public: Inertial(Inertial &&_inertial) noexcept;

    /// \brief Destructor.
    public: virtual ~Inertial();

    /// \brief Move assignment operator.
    /// \param[in] _inertial Inertial component to move.
    /// \return Reference to this.
    public: Inertial &operator=(Inertial &&_inertial);

    /// \brief Copy assignment operator.
    /// \param[in] _inertial Inertial component to copy.
    /// \return Reference to this.
    public: Inertial &operator=(const Inertial &_inertial);

    /// \brief Get the inertial data.
    /// \return The actual inertial information.
    public: const ignition::math::Inertiald &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<InertialPrivate> dataPtr;
  };
  }
}
}
}
#endif

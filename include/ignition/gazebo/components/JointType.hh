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
#ifndef IGNITION_GAZEBO_COMPONENTS_JOINTTYPE_HH_
#define IGNITION_GAZEBO_COMPONENTS_JOINTTYPE_HH_

#include <memory>

#include <sdf/Joint.hh>

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
  class JointTypePrivate;

  /// \brief A component type that contains jointType, sdf::JointType,
  /// information.
  class IGNITION_GAZEBO_VISIBLE JointType
  {
    /// \brief Constructor
    /// \param[in] _jointType sdf::JointType to copy
    public: explicit JointType(const sdf::JointType &_jointType);

    /// \brief Copy Constructor
    /// \param[in] _jointType JointType component to copy.
    public: JointType(const JointType &_jointType);

    /// \brief Move Constructor
    /// \param[in] _jointType JointType component to move.
    public: JointType(JointType &&_jointType) noexcept;

    /// \brief Destructor.
    public: virtual ~JointType();

    /// \brief Move assignment operator.
    /// \param[in] _jointType JointType component to move.
    /// \return Reference to this.
    public: JointType &operator=(JointType &&_jointType);

    /// \brief Copy assignment operator.
    /// \param[in] _jointType JointType component to copy.
    /// \return Reference to this.
    public: JointType &operator=(const JointType &_jointType);

    /// \brief Get the jointtype data.
    /// \return The actual jointtype information.
    public: const sdf::JointType &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<JointTypePrivate> dataPtr;
  };
  }
}
}
}
#endif

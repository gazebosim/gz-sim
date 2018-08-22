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
#ifndef IGNITION_GAZEBO_POSE_COMPONENT_TYPE_HH_
#define IGNITION_GAZEBO_POSE_COMPONENT_TYPE_HH_

#include <memory>

#include <ignition/math/Pose3.hh>

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
  class PosePrivate;

  /// \brief A component type that contains pose, ignition::math::Pose3d,
  /// information.
  class IGNITION_GAZEBO_VISIBLE Pose
  {
    /// \brief Constructor
    /// \param[in] _pose Ignition math pose to copy
    public: explicit Pose(const ignition::math::Pose3d &_pose);

    /// \brief Copy Constructor
    /// \param[in] _pose Pose component to copy.
    public: Pose(const Pose &_pose);

    /// \brief Move Constructor
    /// \param[in] _pose Pose component to move.
    public: Pose(Pose &&_pose) noexcept;

    /// \brief Destructor.
    public: virtual ~Pose();

    /// \brief Move assignment operator.
    /// \param[in] _pose Pose component to move.
    /// \return Reference to this.
    public: Pose &operator=(Pose &&_pose);

    /// \brief Copy assignment operator.
    /// \param[in] _pose Pose component to copy.
    /// \return Reference to this.
    public: Pose &operator=(const Pose &_pose);

    /// \brief Get the pose data.
    /// \return The actual pose information.
    public: const ignition::math::Pose3d &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<PosePrivate> dataPtr;
  };
  }
}
}
}
#endif

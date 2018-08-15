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
#include <string>

#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class PoseComponentPrivate;

    /// \brief A component type that contains pose, ignition::math::Pose3d,
    /// information.
    class IGNITION_GAZEBO_VISIBLE PoseComponent
    {
      /// \brief Constructor
      /// \param[in] _compMgr The entity component manager, which is used to
      /// register the component type.
      public: explicit PoseComponent(const ignition::math::Pose3d &_pose);

      /// \brief Copy Constructor
      /// \param[in] _pose Pose component to copy.
      public: PoseComponent(const PoseComponent &_pose);

      /// \brief Move Constructor
      /// \param[in] _pose Pose component to move.
      public: PoseComponent(PoseComponent &&_pose) noexcept;

      /// \brief Destructor.
      public: virtual ~PoseComponent();

      /// \brief Get the component's name.
      /// \return The name.
      public: const std::string &Name() const;

      /// \brief Move assignment operator.
      /// \param[in] _pose Pose component to move.
      /// \return Reference to this.
      public: PoseComponent &operator=(PoseComponent &&_pose);

      /// \brief Copy assignment operator.
      /// \param[in] _pose Pose component to copy.
      /// \return Reference to this.
      public: PoseComponent &operator=(const PoseComponent &_pose);

      /// \brief Get the pose data.
      /// \return The actual pose information.
      public: const ignition::math::Pose3d &Pose() const;

      /// \brief Private data pointer.
      private: std::unique_ptr<PoseComponentPrivate> dataPtr;
    };
    }
  }
}
#endif

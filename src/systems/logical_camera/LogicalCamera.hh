/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_LOGICALCAMERA_HH_
#define IGNITION_GAZEBO_SYSTEMS_LOGICALCAMERA_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class LogicalCameraPrivate;

  /** \class LogicalCamera LogicalCamera.hh \
   *  ignition/gazebo/systems/LogicalCamera.hh
  **/
  /// \brief A logical camera sensor that reports objects detected within its
  /// frustum readings over ign transport
  class LogicalCamera:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit LogicalCamera();

    /// \brief Destructor
    public: ~LogicalCamera() override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;


    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<LogicalCameraPrivate> dataPtr;
  };
  }
}
}
}
#endif

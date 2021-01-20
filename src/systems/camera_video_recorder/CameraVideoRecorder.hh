/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_CAMERAVIDEORECORDER_SYSTEM_HH_
#define IGNITION_GAZEBO_CAMERAVIDEORECORDER_SYSTEM_HH_

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
  class CameraVideoRecorderPrivate;

  /** \class CameraVideoRecorder CameraVideoRecorder.hh \
   * ignition/gazebo/systems/CameraVideoRecorder.hh
  **/
  /// \brief Record video from a camera sensor
  /// The system takes in the following parameter:
  ///   <topic>    Name of topic for the video recorder service. If this is
  ///              not specified, the topic defaults to:
  ///              /world/<world_name/model/<model_name>/link/<link_name>/
  ///                  sensor/<sensor_name>/record_video
  class CameraVideoRecorder:
    public System,
    public ISystemConfigure,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: CameraVideoRecorder();

    /// \brief Destructor
    public: ~CameraVideoRecorder() final = default;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer
    private: std::unique_ptr<CameraVideoRecorderPrivate> dataPtr;
  };
}
}
}
}
#endif


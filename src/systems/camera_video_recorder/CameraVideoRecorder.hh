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
#ifndef GZ_SIM_CAMERAVIDEORECORDER_SYSTEM_HH_
#define GZ_SIM_CAMERAVIDEORECORDER_SYSTEM_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class CameraVideoRecorderPrivate;

  /** \class CameraVideoRecorder CameraVideoRecorder.hh \
   * gz/sim/systems/CameraVideoRecorder.hh
  **/
  ///
  /// \brief Record video from a camera sensor
  ///
  /// ## System Parameters
  ///
  /// - `<service>`:  Name of topic for the video recorder service. If this is
  ///   not specified, the topic defaults to:
  ///   /world/<world_name/model/<model_name>/link/<link_name>/
  ///   sensor/<sensor_name>/record_video
  ///
  /// - `<use_sim_time>`: True/false value that specifies if the video should
  ///   be recorded using simulation time or real time. The default is false,
  ///   which indicates the use of real time.
  ///
  /// - `<fps>`: Video recorder frames per second. The default value is 25, and
  ///   the support type is unsigned int.
  ///
  /// - `<bitrate>`: Video recorder bitrate (bps). The default value is
  ///   2070000 bps, and the supported type is unsigned int.
  class CameraVideoRecorder final:
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

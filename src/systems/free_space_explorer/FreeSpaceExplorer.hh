/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_FREESPACEEXPLORER_HH_
#define GZ_SIM_SYSTEMS_FREESPACEEXPLORER_HH_

#include <gz/utils/ImplPtr.hh>
#include <memory>
#include <string>

#include "gz/sim/System.hh"
#include "gz/sim/config.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief This plugin is to be used with a model that has a 2D
  /// LiDAR attached to it. It uses this 2d lidar to export an occupancy
  /// map of the world.
  ///
  /// ## System Parameters
  ///
  /// This system takes in the following parameters:
  ///    * `<lidar_topic>` - Topic to listen to LiDAR on. Defaults to `scan`,
  ///      or `<namespace>`/scan when the model has a namespace.
  ///    * `<width>` - Number of columns in the occupancy map
  ///    * `<height>` - Number of rows in the occupancy map
  ///    * `<resolution>` - Resolution of an individual cell
  ///    * `<sensor_link>` - Link on which the sensor is attached within the
  ///                      model.
  ///    * `<image_topic>` - Topic to publish occupancy map on. Defaults to
  ///      `scan_image`, or `<namespace>`/scan_image when the model has a
  ///      namespace.
  ///    * `<start_topic>` - Topic to listen on before starting exploration.
  ///      Defaults to `start`, or `<namespace>`/start when the model has a
  ///      namespace.
  ///
  /// Relative topic names are prefixed with the model namespace. Absolute
  /// topic names (starting with `/`) are left unchanged.
  ///
  /// Note: It is assumed the sensor does not implement any pose
  /// offset from the link itself. This may of may not be true.
  /// and should be fixed in a future release.
  class FreeSpaceExplorer:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    /// \brief Resolved topic names
    public: struct TopicNames
    {
      /// \brief LiDAR topic
      public: std::string lidarTopic;

      /// \brief Occupancy map image topic
      public: std::string imageTopic;

      /// \brief Start topic
      public: std::string startTopic;
    };

    /// \brief Constructor
    public: FreeSpaceExplorer();

    /// \brief Destructor
    public: ~FreeSpaceExplorer() override;

    /// Documentation inherited
    public: void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &/*_eventMgr*/) override;

    /// Documentation inherited
    public: void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Get the resolved topic names
    /// \return Resolved topic names
    public: TopicNames ResolvedTopicNames() const;

    /// \brief Private data pointer
    private: GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
}
}
}
#endif

/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_ENVIRONMENTPRELOAD_VIZTOOL_HH_
#define GZ_SIM_SYSTEMS_ENVIRONMENTPRELOAD_VIZTOOL_HH_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>

#include <gz/transport/Node.hh>

#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>

#include "gz/sim/components/Environment.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{

/// \brief This class helps handle point cloud visuallizations
/// of environment data.
class EnvironmentVisualizationTool
{
  /// \brief Environment constructor
  public: EnvironmentVisualizationTool();

  /// \brief To synchronize member access.
  private: std::mutex mutex;

  /// \brief First load we need to scan for existing data sensor
  private: bool first{true};

  /// \brief Enable resampling
  public: std::atomic<bool> resample{true};

  /// \brief Time has come to an end.
  private: bool finishedTime{false};

  /// \brief Create publisher structures whenever a new environment is made
  /// available.
  /// \param[in] _data Data to be visualized
  /// \param[in] _info simulation info for current time step
  private: void CreatePointCloudTopics(
    const std::shared_ptr<components::EnvironmentalData> &_data,
    const UpdateInfo &_info);

  /// \brief Invoke when new file is made available.
  public: void FileReloaded();

  /// \brief Step the visualizations
  /// \param[in] _info The simulation info including timestep
  /// \param[in] _ecm The Entity-Component-Manager
  /// \param[in] _data The data to be visualized
  /// \param[in] _xSample Samples along x
  /// \param[in] _ySample Samples along y
  /// \param[in] _zSample Samples along z
  public: void Step(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm,
    const std::shared_ptr<components::EnvironmentalData> &_data,
    unsigned int _xSamples, unsigned int _ySamples, unsigned int _zSamples);

  /// \brief Publishes a sample of the data
  /// \param[in] _data The data to be visualized
  /// \param[in] _xSample Samples along x
  /// \param[in] _ySample Samples along y
  /// \param[in] _zSample Samples along z
  private: void Visualize(
    const std::shared_ptr<components::EnvironmentalData> &_data,
    unsigned int _xSamples, unsigned int _ySamples, unsigned int _zSamples);

  /// \brief Get the point cloud data.
  private: void Publish();

  /// \brief Resize the point cloud structure (used to reallocate
  /// memory when resolution changes)
  /// \param[in] _ecm The Entity-Component-Manager
  /// \param[in] _data The data to be visualized
  /// \param[in] _xSample Samples along x
  /// \param[in] _ySample Samples along y
  /// \param[in] _zSample Samples along z
  private: void ResizeCloud(
    const std::shared_ptr<components::EnvironmentalData> &_data,
    const EntityComponentManager &_ecm,
    unsigned int _xSamples, unsigned int _ySamples, unsigned int _zSamples);

  /// \brief Publisher for point clouds
  private: transport::Node::Publisher pcPub;

  /// \brief Publishers for data
  private: std::unordered_map<std::string, transport::Node::Publisher> pubs;

  /// \brief Floating point message buffers
  private: std::unordered_map<std::string, gz::msgs::Float_V> floatFields;

  /// \brief GZ buffers
  private: transport::Node node;

  /// \brief Point cloud buffer
  private: gz::msgs::PointCloudPacked pcMsg;

  /// \brief Session cursors
  private: std::unordered_map<std::string,
    gz::math::InMemorySession<double, double>> sessions;

  /// \brief Duration from last update
  private:  std::chrono::time_point<std::chrono::steady_clock> lastTick;
};
}
}
}
#endif

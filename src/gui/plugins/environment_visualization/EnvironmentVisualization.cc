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

#include "EnvironmentVisualization.hh"

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/sim/components/Environment.hh>
#include <gz/sim/Util.hh>

#include <gz/plugin/Register.hh>

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>

#include <gz/transport/Node.hh>

#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/Utility.hh>

using namespace gz;
using namespace sim;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
/// \brief Private data class for EnvironmentVisualization
class EnvironmentVisualizationTool
{

  /////////////////////////////////////////////////
  public: void Initiallize(
    const EntityComponentManager &_ecm)
  {
    auto world = worldEntity(_ecm);
    auto topic =
      common::joinPaths(
        scopedName(world, _ecm), "environment", "visualize", "res");
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pcPub = node.Advertise<gz::msgs::Vector3d>(topic);
  }

  /////////////////////////////////////////////////
  public: void Visualize(
    double xSamples, double ySamples, double zSamples)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->vec = msgs::Convert(math::Vector3d(xSamples, ySamples, zSamples));
    this->pcPub.Publish(vec);
  }

  /// \brief The sample resolution
  public: gz::msgs::Vector3d vec;

  /// \brief Publisher to publish sample resolution
  public: transport::Node::Publisher pcPub;

  /// \brief Gz transport node
  public: transport::Node node;

  /// \brief To synchronize member access.
  public: std::mutex mutex;

  /// \brief first load we need to scan for existing data sensor
  public: bool first{true};
};
}
}
}

/////////////////////////////////////////////////
EnvironmentVisualization::EnvironmentVisualization()
  : GuiSystem(), dataPtr(new EnvironmentVisualizationTool)
{
  gui::App()->Engine()->rootContext()->setContextProperty(
      "EnvironmentVisualization", this);
  this->qtimer = new QTimer(this);
  connect(qtimer, &QTimer::timeout,
    this, &EnvironmentVisualization::ResamplePointcloud);
  this->qtimer->start(1000);
}

/////////////////////////////////////////////////
EnvironmentVisualization::~EnvironmentVisualization()
{
}

/////////////////////////////////////////////////
void EnvironmentVisualization::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Environment Visualization Resolution";

  gui::App()->findChild<gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void EnvironmentVisualization::Update(const UpdateInfo &,
                               EntityComponentManager &_ecm)
{
  if (this->dataPtr->first)
  {
    this->dataPtr->Initiallize(_ecm);
    this->dataPtr->first = false;
    this->ResamplePointcloud();
  }
}

/////////////////////////////////////////////////
void EnvironmentVisualization::ResamplePointcloud()
{
  this->dataPtr->Visualize(
    this->xSamples, this->ySamples, this->zSamples
  );
}


// Register this plugin
GZ_ADD_PLUGIN(gz::sim::EnvironmentVisualization, gz::gui::Plugin)

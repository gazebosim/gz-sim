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
#include <utility>
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>

using namespace gz;
using namespace sim;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
/// \brief Private data class for EnvironmentVisualization
class EnvironmentVisualizationPrivate
{
  /// \brief To synchronize member access.
  public: std::mutex mutex;

  /// \brief Whether to attempt an environmental data load.
  public: std::atomic<bool> needsLoad{false};

  /// \brief Setup publishers
  public: 

  /// \brief first load we need to scan for existing data sensor
  public: bool first;
};
}
}
}

/////////////////////////////////////////////////
EnvironmentVisualization::EnvironmentVisualization()
  : GuiSystem(), dataPtr(new EnvironmentVisualizationPrivate)
{
  gui::App()->Engine()->rootContext()->setContextProperty(
      "EnvironmentVisualization", this);
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
  _ecm.EachNew<components::Environment>(
    [](
      const Entity &_entity,
      const components::Environment* environment
    ) {
      
    }
  );

  auto environData =
    _ecm.Component<components::Environment>(
      worldEntity(_ecm));
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::EnvironmentVisualization, gz::gui::Plugin)

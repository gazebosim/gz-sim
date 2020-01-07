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

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/rendering.hh>

#include "GridGUI.hh"

// Default cell count
static const int DefaultHonCellCount{20};

// Default vertical cell count
static const int DefaultVerCellCount{1};

// Default cell length
static const double DefaultCellLength{1.0};

// Default pose
static const ignition::math::Pose3d DefaultPose{0, 0, 0.015};

// Default color
static const ignition::math::Color DefaultColor{0.7, 0.7, 0.7, 1.0};

namespace ignition
{
namespace gui
{
namespace plugins
{
  /// \brief Configuration properties for grid
  struct GridConfig
  {
    /// \brief Number of horizontal cells
    int honCellCount{DefaultHonCellCount};

    /// \brief Number of vertical cells
    int verCellCount{DefaultVerCellCount};

    /// \brief Cell length for both horizontal and vertical
    double cellLength{DefaultCellLength};

    /// \brief Grid pose
    math::Pose3d pose{DefaultPose};

    /// \brief Grid color
    math::Color color{DefaultColor};
  }

  class GridGUIPrivate
  {
    /// \brief Pointer to current scene
    public: rendering::ScenePtr scene;

    /// \brief list of grids currently on the scene
    public: std::vector<rendering::GridPtr> grids;
  };
}
}
}

using namespace ignition;
using namespace gui;
using namespace plugins;

/////////////////////////////////////////////////
GridGUI::GridGUI() : Plugin(), dataPtr(new GridGUIPrivate)
{
}

/////////////////////////////////////////////////
GridGUI::~GridGUI()
{
}

/////////////////////////////////////////////////
void GridGUI::LoadConfig()
{
  // Load ptr to current scene
  this->dataPtr->scene = rendering::RenderUtil::Scene();
  if (!this->dataPtr->scene)
    return;
  // if no grid is enabled, disable the plugin
  if (!rendering::RenderUtil::Grid())
    return;
}

/////////////////////////////////////////////////
// Register this plugin
IGN_COMMON_REGISTER_SINGLE_PLUGIN(ignition::gui::plugins::GridGUI,
                                  ignition::gui::Plugin)
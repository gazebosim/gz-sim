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
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/rendering.hh>

#include "ignition/gazebo/gui/GuiEvents.hh"
#include "GridConfig.hh"

namespace ignition::gazebo
{
  struct GridParam
  {
    /// \brief Default horizontal cell count
    int hCellCount{20};

    /// \brief Default vertical cell count
    int vCellCount{0};

    /// \brief Default cell length
    double cellLength{1.0};

    /// \brief Default pose of grid
    math::Pose3d pose{math::Pose3d::Zero};

    /// \brief Default color of grid
    math::Color color{math::Color(0.7, 0.7, 0.7, 1.0)};

    /// \brief Default visible state
    bool visible{true};
  };

  class GridConfigPrivate
  {
    /// \brief Assume only one gridptr in a scene
    public: rendering::GridPtr grid;

    /// \brief Default grid parameters
    public: GridParam gridParam;

    /// \brief Flag that indicates whether there are new updates to be rendered.
    public: bool dirty{false};
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
GridConfig::GridConfig()
  : ignition::gui::Plugin(), dataPtr(std::make_unique<GridConfigPrivate>())
{
}

/////////////////////////////////////////////////
GridConfig::~GridConfig() = default;

/////////////////////////////////////////////////
void GridConfig::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Grid config";

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool GridConfig::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    this->UpdateGrid();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void GridConfig::UpdateGrid()
{
  // Load grid if it doesn't already exist
  if (!this->dataPtr->grid)
    this->LoadGrid();

  // If grid was not loaded successfully, don't update
  if (!this->dataPtr->grid)
    return;

  if (!this->dataPtr->dirty)
    return;

  this->dataPtr->grid->SetVerticalCellCount(
    this->dataPtr->gridParam.vCellCount);
  this->dataPtr->grid->SetCellCount(
    this->dataPtr->gridParam.hCellCount);
  this->dataPtr->grid->SetCellLength(
    this->dataPtr->gridParam.cellLength);

  auto visual = this->dataPtr->grid->Parent();
  if (visual)
  {
    visual->SetLocalPose(this->dataPtr->gridParam.pose);

    auto mat = visual->Material();
    mat->SetAmbient(this->dataPtr->gridParam.color);
    mat->SetDiffuse(this->dataPtr->gridParam.color);
    mat->SetSpecular(this->dataPtr->gridParam.color);

    visual->SetVisible(this->dataPtr->gridParam.visible);
  }

  this->dataPtr->dirty = false;
}

/////////////////////////////////////////////////
void GridConfig::LoadGrid()
{
  auto loadedEngNames = rendering::loadedEngines();
  if (loadedEngNames.empty())
    return;

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    igndbg << "More than one engine is available. "
      << "Grid config plugin will use engine ["
        << engineName << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
    return;

  // assume there is only one scene
  // load scene
  auto scene = engine->SceneByIndex(0);
  if (!scene)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (!scene->IsInitialized() || scene->VisualCount() == 0)
  {
    return;
  }

  // load grid
  // if gridPtr found, load the existing gridPtr to class
  for (unsigned int i = 0; i < scene->VisualCount(); ++i)
  {
    auto vis = scene->VisualByIndex(i);
    if (!vis || vis->GeometryCount() == 0)
      continue;
    for (unsigned int j = 0; j < vis->GeometryCount(); ++j)
    {
      auto grid = std::dynamic_pointer_cast<rendering::Grid>(
            vis->GeometryByIndex(j));
      if (grid)
      {
        igndbg << "Attaching to existing grid" << std::endl;
        this->dataPtr->grid = grid;
        return;
      }
    }
  }

  if (this->dataPtr->grid)
    return;

  // Create grid
  igndbg << "Creating grid" << std::endl;

  auto root = scene->RootVisual();
  this->dataPtr->grid = scene->CreateGrid();
  if (!this->dataPtr->grid)
  {
    ignwarn << "Failed to create grid, grid config plugin won't work."
            << std::endl;

    // If we get here, most likely the render engine and scene are fully loaded,
    // but they don't support grids. So stop trying.
    ignition::gui::App()->findChild<
        ignition::gui::MainWindow *>()->removeEventFilter(this);
    return;
  }

  this->dataPtr->grid->SetCellCount(
    this->dataPtr->gridParam.hCellCount);
  this->dataPtr->grid->SetVerticalCellCount(
    this->dataPtr->gridParam.vCellCount);
  this->dataPtr->grid->SetCellLength(
    this->dataPtr->gridParam.cellLength);

  auto vis = scene->CreateVisual();
  root->AddChild(vis);
  vis->SetLocalPose(this->dataPtr->gridParam.pose);
  vis->AddGeometry(this->dataPtr->grid);

  auto mat = scene->CreateMaterial();
  mat->SetAmbient(this->dataPtr->gridParam.color);
  mat->SetDiffuse(this->dataPtr->gridParam.color);
  mat->SetSpecular(this->dataPtr->gridParam.color);
}

/////////////////////////////////////////////////
void GridConfig::UpdateVCellCount(int _cellCount)
{
  this->dataPtr->gridParam.vCellCount = _cellCount;
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void GridConfig::UpdateHCellCount(int _cellCount)
{
  this->dataPtr->gridParam.hCellCount = _cellCount;
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void GridConfig::UpdateCellLength(double _length)
{
  this->dataPtr->gridParam.cellLength = _length;
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void GridConfig::SetPose(
  double _x, double _y, double _z,
  double _roll, double _pitch, double _yaw)
{
  this->dataPtr->gridParam.pose = math::Pose3d(_x, _y, _z, _roll, _pitch, _yaw);
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void GridConfig::SetColor(double _r, double _g, double _b, double _a)
{
  this->dataPtr->gridParam.color = math::Color(_r, _g, _b, _a);
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void GridConfig::OnShow(bool _checked)
{
  this->dataPtr->gridParam.visible = _checked;
  this->dataPtr->dirty = true;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::GridConfig,
                    ignition::gui::Plugin)

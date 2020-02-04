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
#include <ignition/plugin/Register.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/rendering.hh>

#include "GridConfig.hh"

namespace ignition::gazebo
{
  struct GridParam
  {
    /// \brief Default horizontal cell count
    int honCellCount{20};

    /// \brief Default vertical cell count
    int verCellCount{0};

    /// \brief Default cell length
    double cellLength{1.0};

    /// \brief Default pose of grid
    math::Pose3d pose{math::Pose3d::Zero};

    /// \brief Default color of grid
    math::Color color{math::Color(0.7, 0.7, 0.7, 1.0)};
  };

  class GridConfigPrivate
  {
    /// \brief Ptr to singleton engine
    public: rendering::RenderEngine *engine;

    /// \brief Assume only one gridptr in a scene
    public: rendering::GridPtr grid;

    /// \brief Default grid parameters
    public: GridParam gridParam;

    /// \brief Timer to search for scene
    public: QTimer *timer;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
GridConfig::GridConfig()
  : gui::Plugin(), dataPtr(std::make_unique<GridConfigPrivate>())
{
}

/////////////////////////////////////////////////
GridConfig::~GridConfig() = default;

/////////////////////////////////////////////////
void GridConfig::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Grid config";

  // wait for engine to be created
  this->dataPtr->timer = new QTimer(this);
  this->dataPtr->timer->setInterval(100);
  this->connect(this->dataPtr->timer,
      &QTimer::timeout,
      this,
      &GridConfig::SearchEngine);
  this->dataPtr->timer->start();
}

/////////////////////////////////////////////////
void GridConfig::SearchEngine()
{
  auto loadedEngNames = rendering::loadedEngines();
  if (!loadedEngNames.empty())
  {
    // assume there is only one engine loaded
    auto engineName = loadedEngNames[0];
    if (loadedEngNames.size() > 1)
    {
      igndbg << "More than one engine is available. "
        << "Grid config plugin will use engine ["
          << engineName << "]" << std::endl;
    }
    this->dataPtr->engine = rendering::engine(engineName);
    if (!this->dataPtr->engine)
    {
      ignerr << "Internal error: failed to load engine [" << engineName
        << "]. Grid plugin won't work." << std::endl;
      return;
    }
    if (this->dataPtr->engine->SceneCount() != 0)
    {
      // assume there is only one scene
      // load scene
      auto scene = this->dataPtr->engine->SceneByIndex(0);
      if (!scene)
      {
        ignerr << "No scene found. Grid plugin won't work." << std::endl;
        return;
      }

      if (!scene->IsInitialized() || scene->VisualCount() == 0)
      {
        // Try again next timer tick
        return;
      }

      // stop the timer if both engine and scene found
      this->dataPtr->timer->stop();
      this->disconnect(this->dataPtr->timer, 0, 0, 0);

      // load grid
      this->LoadGrid(scene);
      if (!this->dataPtr->grid)
        this->CreateGrid(scene);
    }
  }
}

/////////////////////////////////////////////////
void GridConfig::LoadGrid(rendering::ScenePtr _scene)
{
  // if gridPtr found, load the existing gridPtr to class
  for (unsigned int i = 0; i < _scene->VisualCount(); ++i)
  {
    auto vis = _scene->VisualByIndex(i);
    if (!vis || vis->GeometryCount() == 0)
      continue;
    for (unsigned int j = 0; j < vis->GeometryCount(); ++j)
    {
      auto grid = std::dynamic_pointer_cast<rendering::Grid>(
            vis->GeometryByIndex(j));
      if (grid)
      {
        this->dataPtr->grid = grid;
        return;
      }
    }
  }
}

/////////////////////////////////////////////////
void GridConfig::CreateGrid(rendering::ScenePtr _scene)
{
  // FIXME(louise) This currently crashes. Must be called from RenderThread.
  // reloading or no existing grid found
  auto root = _scene->RootVisual();
  this->dataPtr->grid = _scene->CreateGrid();
  this->dataPtr->grid->SetCellCount(
    this->dataPtr->gridParam.honCellCount);
  this->dataPtr->grid->SetVerticalCellCount(
    this->dataPtr->gridParam.verCellCount);
  this->dataPtr->grid->SetCellLength(
    this->dataPtr->gridParam.cellLength);

  auto vis = _scene->CreateVisual();
  root->AddChild(vis);
  vis->SetLocalPose(this->dataPtr->gridParam.pose);
  vis->AddGeometry(this->dataPtr->grid);

  auto mat = _scene->CreateMaterial();
  mat->SetAmbient(this->dataPtr->gridParam.color);
  mat->SetDiffuse(this->dataPtr->gridParam.color);
  mat->SetSpecular(this->dataPtr->gridParam.color);

  vis->SetMaterial(mat);
}

/////////////////////////////////////////////////
void GridConfig::UpdateVerCellCount(int _c)
{
  this->dataPtr->gridParam.verCellCount = _c;
  this->dataPtr->grid->SetVerticalCellCount(
    this->dataPtr->gridParam.verCellCount);
}

/////////////////////////////////////////////////
void GridConfig::UpdateHonCellCount(int _c)
{
  this->dataPtr->gridParam.honCellCount = _c;
  this->dataPtr->grid->SetCellCount(
    this->dataPtr->gridParam.honCellCount);
}

/////////////////////////////////////////////////
void GridConfig::UpdateCellLength(double _l)
{
  this->dataPtr->gridParam.cellLength = _l;
  this->dataPtr->grid->SetCellLength(
    this->dataPtr->gridParam.cellLength);
}

/////////////////////////////////////////////////
void GridConfig::SetPose(double _x,
  double _y, double _z, double _roll, double _pitch, double _yaw)
{
  this->dataPtr->gridParam.pose = math::Pose3d(_x, _y, _z, _roll, _pitch, _yaw);
  auto visual = this->dataPtr->grid->Parent();
  if (visual)
  {
    visual->SetLocalPose(this->dataPtr->gridParam.pose);
  }
}

/////////////////////////////////////////////////
void GridConfig::SetColor(double _r, double _g, double _b, double _a)
{
  this->dataPtr->gridParam.color = math::Color(_r, _g, _b, _a);
  auto visual = this->dataPtr->grid->Parent();
  if (visual)
  {
    auto mat = visual->Material();
    mat->SetAmbient(this->dataPtr->gridParam.color);
    mat->SetDiffuse(this->dataPtr->gridParam.color);
    mat->SetSpecular(this->dataPtr->gridParam.color);
    visual->SetMaterial(mat);
  }
  else
  {
    std::cout << "no visual ptr found" << std::endl;
  }
}

/////////////////////////////////////////////////
void GridConfig::OnShow(bool _checked)
{
  auto visual = this->dataPtr->grid->Parent();
  if (visual)
  {
    visual->SetVisible(_checked);
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::GridConfig,
                    ignition::gui::Plugin)

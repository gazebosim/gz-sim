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

    /// \brief Default color of grid - black
    math::Color color{math::Color(0.7, 0.7, 0.7, 1.0)};
  };

  class GridConfigPrivate
  {
    /// \brief Ptr to singleton engine
    public: rendering::RenderEngine *engine;

    public: std::string sceneName{"scene"};

    /// \brief Default grid parameters
    public: GridParam gridParam;

    /// \brief Timer to search for scene
    public: QTimer *timer;

    /// \brief Assume only one gridptr in a scene
    /// store ptr without sharedptr conflict
    public: std::vector<rendering::GridPtr> grids;

    /// \brief Assume only one visualptr in a scene
    /// store ptr without sharedptr conflict
    public: std::vector<rendering::VisualPtr> visuals;

    /// \brief Assume only one materialptr in a scene
    /// store ptr without sharedptr conflict
    public: std::vector<rendering::MaterialPtr> materials;
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
void GridConfig::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  std::string error{""};
  // Default engine name when loading from menu dropdown
  // Crash if actual engine is different, ex. ogre2 vs. ogre
  std::string engineName{"ogre"};

  // Get engine name and scene name from sdf file
  if (this->title.empty())
    this->title = "Grid Config";
  if (_pluginElem)
  {
    if (auto elem = _pluginElem->FirstChildElement("engine"))
    {
      engineName = elem->GetText();
    }
    if (auto elem = _pluginElem->FirstChildElement("scene"))
    {
      this->dataPtr->sceneName = elem->GetText();
    }
  }

  // Retrieve pointer to singleton engine by name
  this->dataPtr->engine = rendering::engine(engineName);
  if (!this->dataPtr->engine)
  {
    error = "Engine "  + engineName
           + " not found, Grid plugin won't work.";
    ignwarn << error << std::endl;
    return;
  }

  // Trigger timer to periodically (/0.1s) search for scene
  // Retrieve grid ptr once scene is created
  this->dataPtr->timer = new QTimer(this);
  this->dataPtr->timer->setInterval(100);
  this->connect(this->dataPtr->timer,
    &QTimer::timeout,
    this,
    &GridConfig::SearchScene);
  this->dataPtr->timer->start();
}

/////////////////////////////////////////////////
void GridConfig::SearchScene()
{
  // stop search if there is a scene found
  if (this->dataPtr->engine->SceneCount() != 0)
  {
    if (this->dataPtr->timer != nullptr)
    {
      this->dataPtr->timer->stop();
      this->disconnect(this->dataPtr->timer, 0, 0, 0);
      rendering::ScenePtr scene =
        this->dataPtr->engine->SceneByName(this->dataPtr->sceneName);

      // Load grid ptr is a scene is created
      if (scene)
      {
        this->InitGrid(scene);
      }
      else
      {
        ignwarn << "No scene found. Grid plugin won't work." << std::endl;
        return;
      }
    }
  }
}

/////////////////////////////////////////////////
void GridConfig::InitGrid(rendering::ScenePtr _scene, bool _reload)
{
  // if gridPtr found, load the existing gridPtr to class
  rendering::GridPtr grid;
  rendering::VisualPtr vis;
  rendering::MaterialPtr mat;

  // first time loading grid
  if (!_reload)
  {
    // search for existing gridptr
    for (unsigned int i = 0; i < _scene->VisualCount(); ++i)
    {
      vis = _scene->VisualByIndex(i);
      if (!vis || vis->GeometryCount() == 0)
        continue;
      mat = vis->Material();
      for (unsigned int j = 0; j < vis->GeometryCount(); ++j)
      {
        grid = std::dynamic_pointer_cast<rendering::Grid>(
              vis->GeometryByIndex(j));
        if (grid)
        {
          this->dataPtr->grids.push_back(grid);
          this->dataPtr->visuals.push_back(vis);
          this->dataPtr->materials.push_back(mat);
          break;
        }
      }
    }
  }
  // reloading or no existing grid found
  if (this->dataPtr->grids.size() == 0)
  {
    auto root = _scene->RootVisual();
    grid = _scene->CreateGrid();
    grid->SetCellCount(this->dataPtr->gridParam.honCellCount);
    grid->SetVerticalCellCount(this->dataPtr->gridParam.verCellCount);
    grid->SetCellLength(this->dataPtr->gridParam.cellLength);

    vis = _scene->CreateVisual();
    root->AddChild(vis);
    vis->SetLocalPose(this->dataPtr->gridParam.pose);
    vis->AddGeometry(grid);

    mat = _scene->CreateMaterial();
    mat->SetAmbient(this->dataPtr->gridParam.color);
    vis->SetMaterial(mat);
    this->dataPtr->grids.push_back(grid);
    this->dataPtr->visuals.push_back(vis);
    this->dataPtr->materials.push_back(mat);
  }
}

/////////////////////////////////////////////////
rendering::VisualPtr GridConfig::GetVisual()
{
  return this->dataPtr->visuals.front();
}

/////////////////////////////////////////////////
rendering::MaterialPtr GridConfig::GetMaterial()
{
  return this->dataPtr->materials.front();
}

/////////////////////////////////////////////////
void GridConfig::UpdateVerCellCount(int _c)
{
  this->dataPtr->gridParam.verCellCount = _c;
  for (auto g : this->dataPtr->grids)
  {
    g->SetVerticalCellCount(this->dataPtr->gridParam.verCellCount);
    break;
  }
}

/////////////////////////////////////////////////
void GridConfig::UpdateHonCellCount(int _c)
{
  this->dataPtr->gridParam.honCellCount = _c;
  for (auto g : this->dataPtr->grids)
  {
    g->SetCellCount(this->dataPtr->gridParam.honCellCount);
    break;
  }
}

/////////////////////////////////////////////////
void GridConfig::UpdateCellLength(double _l)
{
  this->dataPtr->gridParam.cellLength = _l;
  for (auto g : this->dataPtr->grids)
  {
    g->SetCellLength(this->dataPtr->gridParam.cellLength);
    break;
  }
}

/////////////////////////////////////////////////
void GridConfig::SetPose(double _x,
  double _y, double _z, double _roll, double _pitch, double _yaw)
{
  this->dataPtr->gridParam.pose = math::Pose3d(_x, _y, _z, _roll, _pitch, _yaw);
  auto visual = this->GetVisual();
  if (visual)
  {
    visual->SetLocalPose(this->dataPtr->gridParam.pose);
  }
}

/////////////////////////////////////////////////
void GridConfig::SetColor(double _r, double _g, double _b, double _a)
{
  this->dataPtr->gridParam.color = math::Color(_r, _g, _b, _a);
  auto visual = this->GetVisual();
  if (visual)
  {
    auto mat = this->GetMaterial();
    mat->SetAmbient(this->dataPtr->gridParam.color);
    visual->SetMaterial(mat);
  }
}

/////////////////////////////////////////////////
void GridConfig::OnShow(bool _checked)
{
  rendering::ScenePtr scene = this->dataPtr->engine->SceneByName(
    this->dataPtr->sceneName);
  if (_checked)
  {
    this->InitGrid(scene, _checked);
  }
  else
  {
    this->DestroyGrid();
  }
}

/////////////////////////////////////////////////
void GridConfig::DestroyGrid()
{
  for (auto g : this->dataPtr->grids)
  {
    g->Scene()->DestroyVisual(g->Parent());
    this->dataPtr->grids.erase(std::remove(this->dataPtr->grids.begin(),
                                           this->dataPtr->grids.end(), g),
                               this->dataPtr->grids.end());
    break;
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::GridConfig,
                    ignition::gui::Plugin)

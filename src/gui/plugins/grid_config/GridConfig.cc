/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "ignition/gazebo/rendering/RenderUtil.hh"

#include "GridConfig.hh"

namespace ignition::gazebo
{
  struct GridParam
  {
    /// \brief Default horizontal cell count
    int honCellCount{20};

    /// \brief Default vertical cell count
    int verCellCount{1};

    /// \brief Default cell length
    double cellLength{1.0};

    /// \brief Default pose of grid
    math::Pose3d pose{math::Pose3d::Zero};

    /// \brief Default color of grid - black
    math::Color color{math::Color::Black};
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

    /// \brief ptr to grid node
    public: std::vector<rendering::GridPtr> grids;
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
  // will crash if instantiated from dropdown and sdf engine is set to ogre2
  std::string engineName{"ogre"};

  // Get engine name and scene name from sdf file
  if (this->title.empty())
    this->title = "Grid Config";
  if (_pluginElem)
  {
    if (auto elem = _pluginElem->FirstChildElement("engine"))
    {
      engineName = elem->GetText();
      // std::cout << engineName << std::endl;
    } 
    if (auto elem = _pluginElem->FirstChildElement("scene"))
    {
      this->dataPtr->sceneName = elem->GetText();
      // std::cout << this->dataPtr->sceneName << std::endl;
    }
  }
  // get ptr to engine that is being used
  this->dataPtr->engine = rendering::engine(engineName);
  if (!this->dataPtr->engine)
  {
    error = "Engine "  + engineName
           + " not found, Grid plugin won't work.";
    std::cout << error << std::endl;
    return;
  }

  // Periodically search for scene, stop if found
  // this->dataPtr->timer->setInterval(1000);
  // this->connect(this->dataPtr->timer, SIGNAL(timeout()), this, SLOT(this->SearchScene()));
  // this->dataPtr->timer->start();

  rendering::ScenePtr scene = this->dataPtr->engine->SceneByName(this->dataPtr->sceneName);
  if (!scene)
  {
    error = "No scene found. Grid plugin won't work.";
    std::cout << error << std::endl;
    return;
  }
  
  std::cout << "LoadConfig: " << this->dataPtr->grids.size() << std::endl;
  for (auto g : this->dataPtr->grids)
  {
    if (g)
      return;
  }
  // this->InitGrid(scene);
}

/////////////////////////////////////////////////
void GridConfig::SearchScene()
{
  if (this->dataPtr->engine->SceneCount() != 0)
  {
    std::cout << "found a scene" << std::endl;
  }
}

/////////////////////////////////////////////////
void GridConfig::InitGrid(rendering::ScenePtr scene)
{
  // if gridPtr found, load the existing gridPtr to class
  rendering::GridPtr grid;
  for (unsigned int i = 0; i < scene->VisualCount(); ++i)
  {
    auto vis = scene->VisualByIndex(i);
    if (!vis || vis->GeometryCount() == 0)
      continue;

    for (unsigned int j = 0; j < vis->GeometryCount(); ++j)
    {
      grid = std::dynamic_pointer_cast<rendering::Grid>(
                         vis->GeometryByIndex(j));
      if (grid)
        break;
    }
  }
  // else, init a default grid and load that ptr to class
  if (!grid)
  {
    std::cout << "creating new grid..." << std::endl;
    auto root = scene->RootVisual();
    grid = scene->CreateGrid();
    grid->SetCellCount(this->dataPtr->gridParam.honCellCount);
    grid->SetVerticalCellCount(this->dataPtr->gridParam.verCellCount);
    grid->SetCellLength(this->dataPtr->gridParam.cellLength);

    auto visual = scene->CreateVisual();
    root->AddChild(visual);
    visual->SetLocalPose(this->dataPtr->gridParam.pose);
    visual->AddGeometry(grid);

    auto mat = scene->CreateMaterial();
    mat->SetAmbient(this->dataPtr->gridParam.color);
    visual->SetMaterial(mat);
  }
  this->dataPtr->grids.push_back(grid);
  std::cout << "InitGrid: " << this->dataPtr->grids.size() << std::endl;
}

/////////////////////////////////////////////////
void GridConfig::UpdateCellCount(int c)
{
  std::cout << "Update: " << this->dataPtr->grids.size() << std::endl;
  for (auto g : this->dataPtr->grids)
  {
    std::cout << "Updating vertical cell count" << std::endl;
    g->SetCellCount(c);
    break;
  }
}

/////////////////////////////////////////////////
void GridConfig::setPose(double x, double y, double z,
                         double roll, double pitch, double yaw)
{
  this->dataPtr->gridParam.pose = math::Pose3d(x, y, z, roll, pitch, yaw);
}

/////////////////////////////////////////////////
void GridConfig::setColor(double r, double g, double b, double a)
{
  this->dataPtr->gridParam.color = math::Color(r, g, b, a);
}

/////////////////////////////////////////////////
void GridConfig::OnShow(bool checked)
{
  // if checked, show grid
  if (checked)
  {
    std::cout << "Grid is enabled" << std::endl;
  }
  else
  {
    std::cout << "Grid is disabled" << std::endl;
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::GridConfig,
                    ignition::gui::Plugin)

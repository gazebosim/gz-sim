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

    public: std::vector<rendering::VisualPtr> visuals;

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
  
  // trigger the timer if no scene is created yet
  this->dataPtr->timer = new QTimer(this);
  this->dataPtr->timer->setInterval(100);
  this->connect(this->dataPtr->timer, &QTimer::timeout, this, &GridConfig::SearchScene);
  this->dataPtr->timer->start();

  // std::cout << "LoadConfig: " << this->dataPtr->grids.size() << std::endl;
  // for (auto g : this->dataPtr->grids)
  // {
  //   if (g)
  //     return;
  // }
  // this->InitGrid(scene);
}

/////////////////////////////////////////////////
void GridConfig::SearchScene()
{
  // search every 1s for sceneptr
  std::cout << this->dataPtr->engine->SceneCount() <<std::endl;
  // stop search if there is a scene found
  if (this->dataPtr->engine->SceneCount() != 0)
  {
    if (this->dataPtr->timer != nullptr)
    {
      std::cout << "Found scene! Stopping timer..." << std::endl;
      this->dataPtr->timer->stop();
      this->disconnect(this->dataPtr->timer, 0, 0, 0);
      rendering::ScenePtr scene = this->dataPtr->engine->SceneByName(this->dataPtr->sceneName);
      // Initialize grid ptr is a scene is created
      if (!scene)
      {
        std::cout << "No scene found. Grid plugin won't work." << std::endl;
        return;
      }
      else
      {
        this->InitGrid(scene);
      }
    }
  }
}

/////////////////////////////////////////////////
void GridConfig::InitGrid(rendering::ScenePtr scene, bool reload)
{
  // if gridPtr found, load the existing gridPtr to class
  rendering::GridPtr grid;
  rendering::VisualPtr vis;
  rendering::MaterialPtr mat;
  if (!reload)
  {
    for (unsigned int i = 0; i < scene->VisualCount(); ++i)
    {
      vis = scene->VisualByIndex(i);
      if (!vis || vis->GeometryCount() == 0)
        continue;
      mat = vis->Material();
      for (unsigned int j = 0; j < vis->GeometryCount(); ++j)
      {
        // std::cout << "grid count: " << vis->GeometryCount() << std::endl;
        grid = std::dynamic_pointer_cast<rendering::Grid>(
                          vis->GeometryByIndex(j));
        if (grid)
        {
          std::cout << "found existing grid" << std::endl;
          this->dataPtr->grids.push_back(grid);
          this->dataPtr->visuals.push_back(vis);
          this->dataPtr->materials.push_back(mat);
          break;
        }
      }
    }
  }
  else{
    // else, init a default grid and load that ptr to class
    if (this->dataPtr->grids.size() == 0)
    {
      std::cout << "creating new grid..." << std::endl;
      auto root = scene->RootVisual();
      grid = scene->CreateGrid();
      grid->SetCellCount(this->dataPtr->gridParam.honCellCount);
      grid->SetVerticalCellCount(this->dataPtr->gridParam.verCellCount);
      grid->SetCellLength(this->dataPtr->gridParam.cellLength);

      vis = scene->CreateVisual();
      root->AddChild(vis);
      vis->SetLocalPose(this->dataPtr->gridParam.pose);
      vis->AddGeometry(grid);

      mat = scene->CreateMaterial();
      mat->SetAmbient(this->dataPtr->gridParam.color);
      vis->SetMaterial(mat);
      this->dataPtr->grids.push_back(grid);
      this->dataPtr->visuals.push_back(vis);
      this->dataPtr->materials.push_back(mat);
    }
  }
}

/////////////////////////////////////////////////
rendering::VisualPtr GridConfig::GetVisual()
{
  return this->dataPtr->visuals.front();
}

rendering::MaterialPtr GridConfig::GetMaterial()
{
  return this->dataPtr->materials.front();
}

/////////////////////////////////////////////////
void GridConfig::UpdateVerCellCount(int c)
{
  for (auto g : this->dataPtr->grids)
  {
    std::cout << "Updating vertical cell count" << std::endl;
    g->SetVerticalCellCount(c);
    break;
  }
  this->dataPtr->gridParam.verCellCount = c;
}

/////////////////////////////////////////////////
void GridConfig::UpdateHonCellCount(int c)
{
  for (auto g : this->dataPtr->grids)
  {
    std::cout << "Updating horizontal cell count" << std::endl;
    g->SetCellCount(c);
    break;
  }
  this->dataPtr->gridParam.honCellCount = c;
}

/////////////////////////////////////////////////
void GridConfig::UpdateCellLength(double l)
{
  for (auto g : this->dataPtr->grids)
  {
    std::cout << "Updating cell length" <<std::endl;
    g->SetCellLength(l);
    break;
  }
  this->dataPtr->gridParam.cellLength = l;
}

/////////////////////////////////////////////////
void GridConfig::SetPose(double x, double y, double z, double roll, double pitch, double yaw)
{
  this->dataPtr->gridParam.pose = math::Pose3d(x, y, z, roll, pitch, yaw);
  auto visual = this->GetVisual();
  if (visual)
  {
    std::cout << "Updating pose" << std::endl;
    visual->SetLocalPose(this->dataPtr->gridParam.pose);
  }
  
}

/////////////////////////////////////////////////
void GridConfig::SetColor(double r, double g, double b, double a)
{
  this->dataPtr->gridParam.color = math::Color(r, g, b, a);
  // rendering::ScenePtr scene = this->dataPtr->engine->SceneByName(this->dataPtr->sceneName);
  // scene->DestroyMaterials();
  auto visual = this->GetVisual();
  if (visual)
  {
    std::cout << "Updating color" << std::endl;
    auto mat = this->GetMaterial();
    mat->SetAmbient(this->dataPtr->gridParam.color);
    visual->SetMaterial(mat);
  }
}

/////////////////////////////////////////////////
void GridConfig::SetCustomColor(math::Color c)
{
  this->dataPtr->gridParam.color = c;
  // rendering::ScenePtr scene = this->dataPtr->engine->SceneByName(this->dataPtr->sceneName);
  // scene->DestroyMaterials();
  auto visual = this->GetVisual();
  if (visual)
  {
    std::cout << "Updating color" << std::endl;
    auto mat = this->GetMaterial();
    mat->SetAmbient(this->dataPtr->gridParam.color);
    visual->SetMaterial(mat);
  }
}

/////////////////////////////////////////////////
void GridConfig::OnShow(bool checked)
{
  rendering::ScenePtr scene = this->dataPtr->engine->SceneByName(this->dataPtr->sceneName);
  // if checked, show grid
  if (checked)
  {
    std::cout << "Grid is enabled" << std::endl;
    this->InitGrid(scene, checked);
  }
  else
  {
    std::cout << "Grid is disabled" << std::endl;
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

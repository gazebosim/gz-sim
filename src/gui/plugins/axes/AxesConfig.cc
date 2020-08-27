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
#include <ignition/rendering/AxisVisual.hh>

#include "ignition/gazebo/gui/GuiEvents.hh"
#include "AxesConfig.hh"

namespace ignition::gazebo
{
  const std::string worldOriginAxes = std::string("worldOriginAxes");
  class AxesConfigPrivate
  {
    /// \brief axes ptr in a scene
    public: rendering::AxisVisualPtr axes = nullptr;

    /// \brief Default visible state
    bool visible{true};

    /// \brief length of axes
    double length{1};

    /// \brief length of axes
    bool isArrow{true};

    /// \brief Default pose of grid
    math::Pose3d pose{math::Pose3d::Zero};

    /// \brief Scene pointer
    rendering::ScenePtr scene;

    /// \brief name of the axes
    public: std::string name_axes;

    /// \brief structure to save active axes
    public: std::map<std::string, math::Pose3d> activeAxesMap;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
AxesConfig::AxesConfig()
  : ignition::gui::Plugin(), dataPtr(std::make_unique<AxesConfigPrivate>())
{
  this->dataPtr->name_axes = std::string(worldOriginAxes);
}

/////////////////////////////////////////////////
AxesConfig::~AxesConfig() = default;

/////////////////////////////////////////////////
void AxesConfig::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Axes";

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);

  auto loadedEngNames = rendering::loadedEngines();
  if (loadedEngNames.empty())
    return;

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    igndbg << "More than one engine is available. "
      << "Origin axes config plugin will use engine ["
        << engineName << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. Origin axes plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
    return;

  // assume there is only one scene
  // load scene
  this->dataPtr->scene = engine->SceneByIndex(0);
  if (!this->dataPtr->scene)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (!this->dataPtr->scene->IsInitialized() ||
    this->dataPtr->scene->VisualCount() == 0)
  {
    return;
  }

  EntitiesInScene();
}

/////////////////////////////////////////////////
bool AxesConfig::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    UpdateOriginArrows();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

void AxesConfig::UpdateActiveAxes()
{
  for (auto vis : this->dataPtr->activeAxesMap)
  {
    auto visAxes = std::dynamic_pointer_cast<rendering::AxisVisual>(
          this->dataPtr->scene->VisualByName(vis.first + "Axes"));

    auto visEntity = std::dynamic_pointer_cast<rendering::Visual>(
          this->dataPtr->scene->VisualByName(vis.first));

    if (visEntity && visAxes)
    {
      math::Pose3d pose_entity = visEntity->LocalPose();
      visAxes->SetLocalPose(vis.second + pose_entity);
    }
  }
}

/////////////////////////////////////////////////
void AxesConfig::UpdateOriginArrows()
{
  // Load axes if they don't already exist
  this->LoadAxesbyName(this->dataPtr->name_axes);

  // If axes were not loaded successfully, don't update
  if (!this->dataPtr->axes)
    return;

  // Save the axesVisual in the structure if it doesn't exist or
  // update the pose
  auto it = this->dataPtr->activeAxesMap.find(this->dataPtr->name_axes);
  if (it == this->dataPtr->activeAxesMap.end())
  {
    this->dataPtr->activeAxesMap.insert(
      std::pair<std::string, math::Pose3d>(
        this->dataPtr->name_axes,
        this->dataPtr->pose));
  }
  else
  {
    it->second = this->dataPtr->pose;
  }

  // Updating the poses
  this->UpdateActiveAxes();

  // update visibility
  this->dataPtr->axes->SetVisible(this->dataPtr->visible);
  // Update type: arrow or line
  if (this->dataPtr->visible)
  {
    this->dataPtr->axes->ShowAxisHead(this->dataPtr->isArrow);
  }
  // update scale
  this->dataPtr->axes->SetLocalScale(1, 1, this->dataPtr->length * 2);
}

const QStringList AxesConfig::comboList()
{
  return itemComboList;
}

void AxesConfig::SetComboList(const QStringList &comboList)
{
  if (itemComboList != comboList)
  {
    itemComboList = comboList;
    emit ComboListChanged();
  }
}

void AxesConfig::LoadAxesbyName(const std::string & name)
{
  if (this->dataPtr->name_axes.compare(name) == 0 && this->dataPtr->axes)
    return;

  this->dataPtr->name_axes = name;
  if (this->dataPtr->name_axes.compare(worldOriginAxes) != 0)
  {
    this->dataPtr->axes = std::dynamic_pointer_cast<rendering::AxisVisual>(
        this->dataPtr->scene->VisualByName(this->dataPtr->name_axes + "Axes"));
  } else {
    this->dataPtr->axes = std::dynamic_pointer_cast<rendering::AxisVisual>(
        this->dataPtr->scene->VisualByName(worldOriginAxes));
  }

  if (!this->dataPtr->axes)
  {
    rendering::VisualPtr root = this->dataPtr->scene->RootVisual();

    auto axes = this->dataPtr->scene->CreateAxisVisual(
      this->dataPtr->name_axes + "Axes");
    root->AddChild(axes);
  }
}

void AxesConfig::onCurrentIndexChanged(int _index)
{
  std::string name_entity = itemComboList[_index].toStdString();
  LoadAxesbyName(name_entity);
}

void AxesConfig::EntitiesInScene()
{
  std::set<std::string> set_entities;

  for (unsigned int i = 0; i < this->dataPtr->scene->VisualCount(); ++i)
  {
    auto vis = this->dataPtr->scene->VisualByIndex(i);
    if (!vis)
      continue;
    std::string vis_name = vis->Name();
    std::vector<std::string> tokens = ignition::common::split(vis_name, "::");
    if (tokens.size() == 1)
    {
      set_entities.insert(tokens[0]);
    }
  }

  QStringList localCombolist;
  for (auto s : set_entities)
  {
    localCombolist << s.c_str();
  }

  SetComboList(localCombolist);
}

/////////////////////////////////////////////////
void AxesConfig::UpdateLength(double _length)
{
  this->dataPtr->length = _length;
}

/////////////////////////////////////////////////
void AxesConfig::SetPose(
  double _x, double _y, double _z,
  double _roll, double _pitch, double _yaw)
{
  this->dataPtr->pose = math::Pose3d(_x, _y, _z, _roll, _pitch, _yaw);
}

void AxesConfig::OnTypeAxes(bool _checked)
{
  this->dataPtr->isArrow = _checked;
}

/////////////////////////////////////////////////
void AxesConfig::OnShow(bool _checked)
{
  this->dataPtr->visible = _checked;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::AxesConfig,
                    ignition::gui::Plugin)

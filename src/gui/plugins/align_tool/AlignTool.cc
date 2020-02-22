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

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/vector3d.pb.h>

#include <iostream>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/rendering.hh>
#include <ignition/rendering/RenderTypes.hh>
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Name.hh"

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"
#include "AlignTool.hh"

namespace ignition::gazebo
{
  class AlignToolPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect angle mode
    public: std::mutex mutex;

    /// \brief Align Tool service name
    public: std::string service;

    // TODO need to set the pose cmd service to the world topic
    public: std::string poseCmdService;

    public: std::string worldName;

    public: AlignAxis axis{AlignAxis::ALIGN_X};

    public: AlignConfig config{AlignConfig::ALIGN_MIN};
    
    public: AlignStatus status{AlignStatus::NONE};

    public: bool first{true};

    public: bool reverse{false};

    public: bool hovered{false};

    public: bool paused{false};

    public: bool reset{false};

    std::vector<Entity> selectedEntities;

    std::vector<math::Vector3d> prevPositions;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
AlignTool::AlignTool()
  : GuiSystem(), dataPtr(std::make_unique<AlignToolPrivate>())
{
  // Deselect all entities upon loading plugin
  auto deselectEvent = new gui::events::DeselectAllEntities(true);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      deselectEvent);
}

/////////////////////////////////////////////////
AlignTool::~AlignTool() = default;

/////////////////////////////////////////////////
void AlignTool::LoadConfig(const tinyxml2::XMLElement *)
{
  // For align tool requests
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void AlignTool::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  this->dataPtr->paused = _info.paused;
  if (this->dataPtr->worldName.empty())
  {
    // TODO(anyone) Only one scene is supported for now
    _ecm.Each<components::World, components::Name>(
        [&](const Entity &/*_entity*/,
          const components::World * /* _world */ ,
          const components::Name *_name)->bool
        {
          this->dataPtr->worldName = _name->Data();
          return true;
        });
  }
}

/////////////////////////////////////////////////
bool AlignTool::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::Render::Type)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    if ((this->dataPtr->hovered || this->dataPtr->reset)
         && this->dataPtr->selectedEntities.size() > 1)
    {
      this->Align();
      this->dataPtr->reset = false;
      this->dataPtr->hovered = false;
    }
  }
  else if (_event->type() == ignition::gazebo::gui::events::EntitiesSelected::Type)
  {
    auto selectedEvent = reinterpret_cast<gui::events::EntitiesSelected *>(_event);
    if (selectedEvent && !selectedEvent->Data().empty() && !selectedEvent->FromUser())
    {
      this->dataPtr->selectedEntities = selectedEvent->Data();
    }
  }
  else if (_event->type() == ignition::gazebo::gui::events::DeselectAllEntities::Type)
  {
    this->dataPtr->selectedEntities.clear();
  }
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void AlignTool::SetAlignStatus(const QString &_status)
{
  std::string newStatus = _status.toStdString();  
  std::transform(newStatus.begin(), newStatus.end(), newStatus.begin(), ::tolower);
  if (!this->dataPtr->paused)
  {
    if (newStatus == "no_align")
    {
      this->dataPtr->status = AlignStatus::NO_ALIGN;
    }
    else if (newStatus == "align")
    {
      this->dataPtr->status = AlignStatus::ALIGN;
    }
    else if (newStatus == "none")
    {
      this->dataPtr->status = AlignStatus::NONE;
    }
    else
    {
      ignwarn << "Invalid align status string: " << newStatus << "\n";
      ignwarn << "The valid options are:\n";
      ignwarn << " - no_align\n";
      ignwarn << " - align\n";
      ignwarn << " - none\n";
    }
  }
}

/////////////////////////////////////////////////
// TODO(john) split the following function into two separate
// enter and exit functions
void AlignTool::SetHovered(bool _hovered)
{
  if (!this->dataPtr->paused)
  {
    this->dataPtr->hovered = _hovered;

    // If we exit hovering and the button has not been clicked, reset nodes
    if (!this->dataPtr->hovered && this->dataPtr->status == AlignStatus::NO_ALIGN)
    {
      // TODO set entities back to prev pos
      
      this->dataPtr->reset = true;
      this->SetAlignStatus("none");
    }
    if (!this->dataPtr->hovered)
    {
      this->SetAlignStatus("none");
      this->dataPtr->prevPositions.clear();
    }
    else if (this->dataPtr->hovered)
    {
      this->SetAlignStatus("no_align");
    }
  }
}

/////////////////////////////////////////////////
void AlignTool::OnAlignAxis(const QString &_axis)
{
  std::string newAxis = _axis.toStdString();  
  std::transform(newAxis.begin(), newAxis.end(), newAxis.begin(), ::tolower);

  if (newAxis == "x")
  {
    this->dataPtr->axis = AlignAxis::ALIGN_X;
  }
  else if (newAxis == "y")
  {
    this->dataPtr->axis = AlignAxis::ALIGN_Y;
  }
  else if (newAxis == "z")
  {
    this->dataPtr->axis = AlignAxis::ALIGN_Z;
  }
  else
  {
    ignwarn << "Invalid align axis string: " << newAxis << "\n";
    ignwarn << "The valid options are:\n";
    ignwarn << " - X\n";
    ignwarn << " - Y\n";
    ignwarn << " - Z\n";
  }
}

/////////////////////////////////////////////////
void AlignTool::OnAlignTarget(const QString &_target)
{
  std::string newTarget = _target.toStdString();
  std::transform(newTarget.begin(), newTarget.end(), newTarget.begin(), ::tolower);

  if (newTarget == "first")
  {
    this->dataPtr->first = true;
  }
  else if (newTarget == "last")
  {
    this->dataPtr->first = false;
  }
  else
  {
    ignwarn << "Invalid align target string: " << newTarget << "\n";
    ignwarn << "The valid options are:\n";
    ignwarn << " - first\n";
    ignwarn << " - last\n"; 
  }
}

/////////////////////////////////////////////////
void AlignTool::OnReverse(bool _reverse)
{
  this->dataPtr->reverse = _reverse;
}

/////////////////////////////////////////////////
void AlignTool::OnAlignConfig(const QString &_config)
{
  std::string newConfig = _config.toStdString();
  std::transform(newConfig.begin(), newConfig.end(), newConfig.begin(), ::tolower);

  if (newConfig == "min")
  {
    this->dataPtr->config = AlignConfig::ALIGN_MIN;
  }
  else if (newConfig == "center")
  {
    this->dataPtr->config = AlignConfig::ALIGN_CENTER;
  }
  else if (newConfig == "max")
  {
    this->dataPtr->config = AlignConfig::ALIGN_MAX;
  }
  else
  {
    ignwarn << "Invalid align config string: " << newConfig << "\n";
    ignwarn << "The valid options are:\n";
    ignwarn << " - min\n"; 
    ignwarn << " - center\n"; 
    ignwarn << " - max\n"; 
  }
}

/////////////////////////////////////////////////
void AlignTool::Align()
{
  auto loadedEngNames = rendering::loadedEngines();
  if (loadedEngNames.empty())
    return;

  // Assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    ignerr << "Internal error: failed to load engine [" << engineName

      << "]. Grid plugin won't work." << std::endl;

    return;
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

  // Get current list of selected entities
  std::vector<ignition::rendering::VisualPtr> selectedList;
  ignition::rendering::VisualPtr relativeVisual;

  for (const auto &entityId : this->dataPtr->selectedEntities)
  {
    for (unsigned int i = 0; i < scene->VisualCount(); i++)
    {
      ignition::rendering::VisualPtr vis = scene->VisualByIndex(i);
      if (!vis)
        continue;
      if (std::get<int>(vis->UserData("gazebo-entity")) == static_cast<int>(entityId))
      {
        selectedList.push_back(vis);
      }
    }
  }

  // Set relative visual to move the others around
  this->dataPtr->first ?
    (relativeVisual = selectedList.front()) :
    (relativeVisual = selectedList.back());

  // Callback function for ignition node request
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/* _rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting pose" << std::endl;
  };

  // Set service topic
  if (this->dataPtr->poseCmdService.empty())
  {
    this->dataPtr->poseCmdService = "/world/" + this->dataPtr->worldName
        + "/set_pose";
  }

  int axisIndex = static_cast<int>(this->dataPtr->axis);
  ignition::msgs::Pose req;

  for (unsigned int i = 0; i < selectedList.size(); i++)
  {
    rendering::VisualPtr vis = selectedList[i];

    // Set new position according to the chosen axis
    math::Vector3d newPos = vis->WorldPosition();
    if (this->dataPtr->reset)
    {
      newPos = this->dataPtr->prevPositions[i];
    }
    else
    {
      double relativeCoord = relativeVisual->WorldPosition()[axisIndex];
      newPos[axisIndex] = relativeCoord;

      // Store the vis's world positions in a vector
      this->dataPtr->prevPositions.push_back(vis->WorldPosition());
    }

    // Send new position request 
    req.set_name(vis->Name());
    msgs::Set(req.mutable_position(), newPos);
    msgs::Set(req.mutable_orientation(), vis->WorldRotation());
    this->dataPtr->node.Request(this->dataPtr->poseCmdService, req, cb);
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::AlignTool,
                    ignition::gui::Plugin)

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

#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/rendering.hh>

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

    public: AlignAxis axis{AlignAxis::ALIGN_X};

    public: AlignConfig config{AlignConfig::ALIGN_MIN};

    public: AlignTarget target{AlignTarget::FIRST};

    public: bool reverse{false};

    std::vector<Entity> selectedEntities;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
AlignTool::AlignTool()
  : ignition::gui::Plugin(), dataPtr(std::make_unique<AlignToolPrivate>())
{
  // TODO deselect all selected here
}

/////////////////////////////////////////////////
AlignTool::~AlignTool() = default;

/////////////////////////////////////////////////
void AlignTool::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Align Tool";

  // For align tool requests
  this->dataPtr->service = "/gui/align_tool";
}

// TODO add eventFilter function with updates
/////////////////////////////////////////////////
bool AlignTool::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::Render::Type)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    this->Align();
  }
  else if (_event->type() == ignition::gazebo::gui::events::EntitiesSelected::Type)
  {
    auto selectedEvent = reinterpret_cast<gui::events::EntitiesSelected *>(_event);
    if (selectedEvent && !selectedEvent->Data().empty() && !selectedEvent->FromUser())
    {
      this->dataPtr->selectedEntities = selectedEvent->Data();
      for (const auto &i : this->dataPtr->selectedEntities)
      {
        ignwarn << i << "\n";
      }
    }
  }
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void AlignTool::OnAlignAxis(const QString &_axis)
{
  std::string newAxis = _axis.toStdString();  
  std::transform(newAxis.begin(), newAxis.end(), newAxis.begin(), ::tolower);
  ignwarn << "Received " << newAxis << "\n";

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

void AlignTool::OnAlignTarget(const QString &_target)
{
  std::string newTarget = _target.toStdString();
  std::transform(newTarget.begin(), newTarget.end(), newTarget.begin(), ::tolower);

  ignwarn << "Received " << newTarget << "\n";

  if (newTarget == "first")
  {
    this->dataPtr->target = AlignTarget::FIRST;
  }
  else if (newTarget == "last")
  {
    this->dataPtr->target = AlignTarget::LAST;
  }
  else
  {
    ignwarn << "Invalid align target string: " << newTarget << "\n";
    ignwarn << "The valid options are:\n";
    ignwarn << " - first\n";
    ignwarn << " - last\n"; 
  }
}

void AlignTool::OnReverse(bool _reverse)
{
  ignwarn << "Received " << _reverse << "\n";

  this->dataPtr->reverse = _reverse;
}

void AlignTool::OnAlignConfig(const QString &_config)
{
  std::string newConfig = _config.toStdString();
  std::transform(newConfig.begin(), newConfig.end(), newConfig.begin(), ::tolower);

  ignwarn << "Received " << newConfig << "\n";

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

  // TODO get current list of selected entities

 // auto runners = ignition::gui::App()->findChildren<RenderUtil *>();
  //auto renderUtil = mainWindow->findChildren<ignition::gazebo::RenderUtil *>();
  /*
  if (!renderUtil)
  {
    ignwarn << "renderutil is null\n";
  }
  else
  {
    ignwarn << "found renderutil\n";
  }
  */
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::AlignTool,
                    ignition::gui::Plugin)

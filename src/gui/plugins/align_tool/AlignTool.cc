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

    /// \brief The service call string for requesting a new pose for an entity
    public: std::string poseCmdService;

    /// \brief The current world name
    public: std::string worldName;

    /// \brief The current selected axis about which to align
    public: AlignAxis axis{AlignAxis::ALIGN_X};

    /// \brief The current align configuration
    public: AlignConfig config{AlignConfig::ALIGN_CENTER};

    /// \brief The current align status
    public: AlignStatus currentStatus{AlignStatus::NONE};

    /// \brief Flag to indicate if the entities to should aligned to the first
    /// or last entity selected
    public: bool first{true};

    /// \brief Flag to indicate if the reverse align should occur
    public: bool reverse{false};

    /// \brief Flag to indicate if the rendering thread should update the scene
    /// with newly provided information.
    public: bool alignDirty{false};

    /// \brief Flag to indicate if the simulation is currently paused
    public: bool paused{false};

    /// \brief The current selected entities
    public: std::vector<Entity> selectedEntities;

    /// \brief the previous positions of all the selected nodes.  Should always
    /// be equal to the number of selected entities
    public: std::vector<math::Vector3d> prevPositions;

    public: std::queue<AlignStatus> statuses;

    public: std::map<std::string, double> originalTransparency;
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
    // TODO dequeue from the status queue and execute below if statement if there
    // is a state
    
    if (!this->dataPtr->statuses.empty())
    {
      this->dataPtr->currentStatus = this->dataPtr->statuses.front();
      this->dataPtr->statuses.pop();
      this->Align();
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

void AlignTool::OnHoveredEntered()
{
  if (!this->dataPtr->paused)
  {
    this->AddStatus(AlignStatus::HOVER);
  }
}

void AlignTool::OnHoveredExited()
{
  if (!this->dataPtr->paused)
  {
    // If no align has occurred, reset entities to start positions,
    // otherwise, set status back to none
    if (this->dataPtr->currentStatus == AlignStatus::HOVER)
      this->AddStatus(AlignStatus::RESET);
    else
      this->AddStatus(AlignStatus::NONE);
    this->dataPtr->prevPositions.clear();
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
void AlignTool::AddStatus(const AlignStatus &_status)
{
  this->dataPtr->statuses.push(_status);
}

/////////////////////////////////////////////////
void AlignTool::AddStatus(const QString &_status)
{
  std::string newStatus = _status.toStdString();
  std::transform(newStatus.begin(), newStatus.end(), newStatus.begin(), ::tolower);

  if (newStatus == "hover")
  {
    this->AddStatus(AlignStatus::HOVER);
  }
  else if (newStatus == "reset")
  {
    this->AddStatus(AlignStatus::RESET);
  }
  else if (newStatus == "align")
  {
    this->AddStatus(AlignStatus::ALIGN);
  }
  else if (newStatus == "none")
  {
    this->AddStatus(AlignStatus::NONE);
  }
  else
  {
    ignwarn << "Invalid align status: " << newStatus << "\n";
    ignwarn << "The valid options are:\n";
    ignwarn << " - hover\n";
    ignwarn << " - reset\n"; 
    ignwarn << " - align\n";
    ignwarn << " - none\n"; 
  }
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

  // Selected links will result in this list being empty as they won't be 
  // found in the above VisualByIndex call
  if (selectedList.size() < 2)
    return;

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

  // Index math to avoid iterating through the selected node
  for (unsigned int i = this->dataPtr->first; i < selectedList.size() + this->dataPtr->first - 1; i++)
  {
    rendering::VisualPtr vis = selectedList[i];

    // TODO check here to see if visual is top level or not, continue if not
    rendering::VisualPtr topLevelVis = this->TopLevelVisual(scene, vis);
    if (topLevelVis != vis)
    {
      ignwarn << "top level: " << topLevelVis << "\n";
      ignwarn << "vis: " << vis << "\n";
      ignwarn << "visual is not top level\n";
      continue;
    }
    math::Vector3d newPos = vis->WorldPosition();

    // If a reset is occuring, the user has not clicked align,
    // so pull all of the previous positions and set
    if (this->dataPtr->currentStatus == AlignStatus::RESET)
    {
      this->MakeSolid(vis);
      newPos = this->dataPtr->prevPositions[i];
      vis->SetWorldPosition(newPos);
      vis->SetUserData("pause-update", static_cast<int>(0));
    }
    // If an align is occurring, the user has clicked the align button,
    // make a request to the backend to send the node to the new position
    else if (this->dataPtr->currentStatus == AlignStatus::ALIGN)
    {
      this->MakeSolid(vis);
      req.set_name(vis->Name());
      msgs::Set(req.mutable_position(), newPos);
      msgs::Set(req.mutable_orientation(), vis->WorldRotation());
      this->dataPtr->node.Request(this->dataPtr->poseCmdService, req, cb);
      vis->SetUserData("pause-update", static_cast<int>(0));
    }
    // Hover state has been entered, update visuals to relative visual,
    // prevent RenderUtil from updating the visual to its actual position,
    // and store the current position of the nodes
    else if (this->dataPtr->currentStatus == AlignStatus::HOVER)
    {
      newPos[axisIndex] = relativeVisual->WorldPosition()[axisIndex];

      // Store the vis's world positions in a vector
      this->dataPtr->prevPositions.push_back(vis->WorldPosition());

      // Make the visual transparent and update to new position
      this->MakeTransparent(vis);
      vis->SetUserData("pause-update", static_cast<int>(1));
      vis->SetWorldPosition(newPos);
    }
  }
  
  // If reset has iterated once, set status back to none as job
  // has been completed, and reset transparency map
  if (this->dataPtr->currentStatus == AlignStatus::RESET)
  {
    this->dataPtr->originalTransparency.clear();
    this->dataPtr->currentStatus = AlignStatus::NONE;
  }
  // Also reset transparency map if status is align
  else if (this->dataPtr->currentStatus == AlignStatus::ALIGN)
  {
    this->dataPtr->originalTransparency.clear();
  }
}

void AlignTool::MakeSolid(const rendering::NodePtr &_node)
{
  if (!_node)
    return;

  for (auto n = 0u; n < _node->ChildCount(); ++n)
  {
    auto child = _node->ChildByIndex(n);
    this->MakeSolid(child);
  }

  auto vis = std::dynamic_pointer_cast<rendering::Visual>(_node);
  if (nullptr == vis)
    return;

  // Visual material
  auto visMat = vis->Material();
  if (nullptr != visMat)
  {
    auto visTransparency = this->dataPtr->originalTransparency.find(vis->Name());
    if (visTransparency != this->dataPtr->originalTransparency.end())
    {
      visMat->SetTransparency(visTransparency->second);
      vis->SetMaterial(visMat);
    }
  }

  for (auto g = 0u; g < vis->GeometryCount(); ++g)
  {
    auto geom = vis->GeometryByIndex(g);

    // Geometry material
    auto geomMat = geom->Material();
    if (nullptr == geomMat)
      continue;

    auto geomTransparency = this->dataPtr->originalTransparency.find(geom->Name());
    if (geomTransparency != this->dataPtr->originalTransparency.end())
    {
      geomMat->SetTransparency(geomTransparency->second);
      geom->SetMaterial(geomMat);
    }
  }
}

////////////////////////////////////////////////
void AlignTool::MakeTransparent(const rendering::NodePtr &_node)
{
  if (!_node)
    return;

  for (auto n = 0u; n < _node->ChildCount(); ++n)
  {
    auto child = _node->ChildByIndex(n);
    this->MakeTransparent(child);
  }

  auto vis = std::dynamic_pointer_cast<rendering::Visual>(_node);
  if (nullptr == vis)
    return;

  // Visual material
  auto visMat = vis->Material();
  if (nullptr != visMat)
  {
    // If the entity isn't already highlighted, highlight it
    if (this->dataPtr->originalTransparency.find(vis->Name()) ==
        this->dataPtr->originalTransparency.end())
    {
      this->dataPtr->originalTransparency[vis->Name()] = visMat->Transparency();
      visMat->SetTransparency(visMat->Transparency() + 0.5);
      vis->SetMaterial(visMat);
    }
  }

  for (auto g = 0u; g < vis->GeometryCount(); ++g)
  {
    auto geom = vis->GeometryByIndex(g);

    // Geometry material
    auto geomMat = geom->Material();
    if (nullptr == geomMat)
      continue;

    // If the entity isn't already highlighted, highlight it
    if (this->dataPtr->originalTransparency.find(geom->Name()) ==
        this->dataPtr->originalTransparency.end())
    {
      this->dataPtr->originalTransparency[geom->Name()] = geomMat->Transparency();
      geomMat->SetTransparency(geomMat->Transparency() + 0.5);
      geom->SetMaterial(geomMat);
    }
  }
}

/////////////////////////////////////////////////
rendering::VisualPtr AlignTool::TopLevelVisual(rendering::ScenePtr &_scene,
    rendering::VisualPtr &_visual) const
{
  auto visNode = std::dynamic_pointer_cast<rendering::Node>(_visual);
  auto node = this->TopLevelNode(_scene, visNode);
  return std::dynamic_pointer_cast<rendering::Visual>(node);
}

/////////////////////////////////////////////////
rendering::NodePtr AlignTool::TopLevelNode(rendering::ScenePtr &_scene,
    rendering::NodePtr &_node) const
{
  rendering::NodePtr rootNode =
      _scene->RootVisual();

  rendering::NodePtr node;

  if (_node)
  {
    node = std::dynamic_pointer_cast<rendering::Node>(_node->Parent());
    
    // Be sure not to skip checking the node before iterating through
    // its ancestors below
    if (node && node == rootNode)
      return _node;
  }
  else
    ignwarn << "Node is null\n";

  while (node && node->Parent() != rootNode)
  {
    node =
      std::dynamic_pointer_cast<rendering::Node>(node->Parent());
  }
  ignwarn << "node after loop " << node << "\n";
  return node;
}


// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::AlignTool,
                    ignition::gui::Plugin)

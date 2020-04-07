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
#include <ignition/msgs/pose.pb.h>

#include <iostream>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Geometry.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/Scene.hh>
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

    /// \brief The service call string for requesting a new pose for an entity.
    public: std::string poseCmdService;

    /// \brief Mutex to protect align tool
    public: std::mutex mutex;

    /// \brief The current world name.
    public: std::string worldName;

    /// \brief The current selected axis about which to align.
    public: AlignAxis axis{AlignAxis::ALIGN_X};

    /// \brief The current align state.
    public: AlignState currentState{AlignState::NONE};

    /// \brief Flag to indicate if the entities should be aligned to the first
    /// or last entity selected.
    public: bool first{true};

    /// \brief The current selected entities
    public: std::vector<Entity> selectedEntities;

    /// \brief The previous positions of all the selected nodes.  Should always
    /// be equal to the number of selected entities.
    public: std::vector<math::Vector3d> prevPositions;

    /// \brief The current queue of states to be executed.
    public: std::queue<AlignState> states;

    /// \brief The map of the original transparency values for the nodes.
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
  if (this->title.empty())
    this->title = "Align tool";

  // For align tool requests
  ignition::gui::App()->findChild
      <ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void AlignTool::Update(const UpdateInfo &/* _info */,
    EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
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
void AlignTool::OnAlignAxis(const QString &_axis)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
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
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  std::string newTarget = _target.toStdString();
  std::transform(newTarget.begin(), newTarget.end(),
                 newTarget.begin(), ::tolower);

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
void AlignTool::OnHoveredEntered()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->AddState(AlignState::HOVER);
}

/////////////////////////////////////////////////
void AlignTool::OnHoveredExited()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // Always reset after an exit, if an align call is made, it
  // will have updated the prev positions of the entities so a
  // reset after align will result in no change
  this->AddState(AlignState::RESET);
}

/////////////////////////////////////////////////
void AlignTool::OnAlign()
{
  this->AddState(AlignState::ALIGN);
}

/////////////////////////////////////////////////
void AlignTool::AddState(const AlignState &_state)
{
  this->dataPtr->states.push(_state);
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
    // If the entity isn't already transparent, make it transparent
    if (this->dataPtr->originalTransparency.find(vis->Name()) ==
        this->dataPtr->originalTransparency.end())
    {
      this->dataPtr->originalTransparency[vis->Name()] = visMat->Transparency();
      visMat->SetTransparency(visMat->Transparency() + 0.5);
    }
  }

  for (auto g = 0u; g < vis->GeometryCount(); ++g)
  {
    auto geom = vis->GeometryByIndex(g);

    // Geometry material
    auto geomMat = geom->Material();
    if (nullptr == geomMat)
      continue;

    // If the entity isn't already transparent, make it transparent
    if (this->dataPtr->originalTransparency.find(geom->Name()) ==
        this->dataPtr->originalTransparency.end())
    {
      this->dataPtr->originalTransparency[geom->Name()] =
          geomMat->Transparency();
      geomMat->SetTransparency(geomMat->Transparency() + 0.5);
    }
  }
}

/////////////////////////////////////////////////
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
    auto visTransparency =
        this->dataPtr->originalTransparency.find(vis->Name());
    if (visTransparency != this->dataPtr->originalTransparency.end())
    {
      visMat->SetTransparency(visTransparency->second);
    }
  }

  for (auto g = 0u; g < vis->GeometryCount(); ++g)
  {
    auto geom = vis->GeometryByIndex(g);

    // Geometry material
    auto geomMat = geom->Material();
    if (nullptr == geomMat)
      continue;

    auto geomTransparency =
        this->dataPtr->originalTransparency.find(geom->Name());
    if (geomTransparency != this->dataPtr->originalTransparency.end())
    {
      geomMat->SetTransparency(geomTransparency->second);
    }
  }
}

/////////////////////////////////////////////////
void AlignTool::Align()
{
  if (this->dataPtr->currentState == AlignState::NONE)
    return;

  auto loadedEngNames = rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    ignerr << "Internal error: engine should be loaded at this point."
      << std::endl;
    return;
  }

  // Assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    ignwarn << "Found more than one loaded engine "
      "- using " << engineName << "." << std::endl;
  }
  auto engine = rendering::engine(engineName);

  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. Align tool plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    ignerr<< "Internal error: no scenes are available with the loaded engine."
      << std::endl;
    return;
  }
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
    ignerr << "Internal error: scene is either not initialized "
      "or there are no visuals within it." << std::endl;
    return;
  }

  // Get current list of selected entities
  std::vector<ignition::rendering::VisualPtr> selectedList;
  ignition::rendering::VisualPtr relativeVisual;

  for (const auto &entityId : this->dataPtr->selectedEntities)
  {
    for (auto i = 0u; i < scene->VisualCount(); ++i)
    {
      ignition::rendering::VisualPtr vis = scene->VisualByIndex(i);
      if (!vis)
        continue;
      if (std::get<int>(vis->UserData("gazebo-entity")) ==
          static_cast<int>(entityId))
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
  for (unsigned int i = this->dataPtr->first;
       i < selectedList.size() + this->dataPtr->first - 1; i++)
  {
    rendering::VisualPtr vis = selectedList[i];

    // Check here to see if visual is top level or not, continue if not
    rendering::VisualPtr topLevelVis = this->TopLevelVisual(scene, vis);
    if (topLevelVis != vis)
      continue;

    math::Vector3d newPos = vis->WorldPosition();

    // If a reset is occuring, the user has not clicked align,
    // so pull all of the previous positions and set
    if (this->dataPtr->currentState == AlignState::RESET)
    {
      // Index offset given the relative node does not get added to the
      // prevPositions vector
      newPos = this->dataPtr->prevPositions[i-this->dataPtr->first];

      this->MakeSolid(vis);
      vis->SetWorldPosition(newPos);
      vis->SetUserData("pause-update", static_cast<int>(0));
    }
    // If an align is occurring, the user has clicked the align button,
    // make a request to the backend to send the node to the new position
    else if (this->dataPtr->currentState == AlignState::ALIGN)
    {
      this->MakeSolid(vis);
      req.set_name(vis->Name());
      msgs::Set(req.mutable_position(), newPos);
      msgs::Set(req.mutable_orientation(), vis->WorldRotation());
      this->dataPtr->node.Request(this->dataPtr->poseCmdService, req, cb);
      vis->SetUserData("pause-update", static_cast<int>(0));
      this->dataPtr->prevPositions[i-this->dataPtr->first] = newPos;
    }
    // Hover state has been entered, update visuals to relative visual,
    // prevent RenderUtil from updating the visual to its actual position,
    // and store the current position of the nodes
    else if (this->dataPtr->currentState == AlignState::HOVER)
    {
      newPos[axisIndex] = relativeVisual->WorldPosition()[axisIndex];

      // Store the vis's world positions in a vector
      this->dataPtr->prevPositions.push_back(vis->WorldPosition());

      // Make the visual transparent and update to new position
      this->MakeTransparent(vis);
      vis->SetWorldPosition(newPos);
      vis->SetUserData("pause-update", static_cast<int>(1));
    }
  }

  // If reset has iterated once, set state back to none as job
  // has been completed, and reset transparency map
  if (this->dataPtr->currentState == AlignState::RESET)
  {
    this->dataPtr->originalTransparency.clear();
    this->dataPtr->currentState = AlignState::NONE;
    this->dataPtr->prevPositions.clear();
  }
  // Also reset transparency map if state is align
  else if (this->dataPtr->currentState == AlignState::ALIGN)
  {
    this->dataPtr->originalTransparency.clear();
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

  while (node && node->Parent() != rootNode)
  {
    node =
      std::dynamic_pointer_cast<rendering::Node>(node->Parent());
  }

  return node;
}

/////////////////////////////////////////////////
bool AlignTool::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here

    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    // If there's a state available, pop it from the queue and execute it
    if (!this->dataPtr->states.empty())
    {
      this->dataPtr->currentState = this->dataPtr->states.front();
      this->dataPtr->states.pop();
      this->Align();
    }
  }
  else if (_event->type() ==
           ignition::gazebo::gui::events::EntitiesSelected::kType)
  {
    auto selectedEvent =
        reinterpret_cast<gui::events::EntitiesSelected *>(_event);

    // Only update if a valid cast, the data isn't empty, and
    // the command is not from user (sent from backend)
    if (selectedEvent && !selectedEvent->Data().empty())
    {
      for (const auto &_entity : selectedEvent->Data())
      {
        // If the element already exists in the selected entities vector,
        // continue
        if (std::find(this->dataPtr->selectedEntities.begin(),
              this->dataPtr->selectedEntities.end(),
              _entity) != this->dataPtr->selectedEntities.end())
          continue;
        this->dataPtr->selectedEntities.push_back(_entity);
      }
    }
  }
  else if (_event->type() ==
           ignition::gazebo::gui::events::DeselectAllEntities::kType)
  {
    this->dataPtr->selectedEntities.clear();
  }
  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::AlignTool,
                    ignition::gui::Plugin)

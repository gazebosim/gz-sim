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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/WireBox.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/gui/GuiEvents.hh"
#include "gz/sim/rendering/RenderUtil.hh"

#include "AlignTool.hh"

namespace gz::sim
{
  class AlignToolPrivate
  {
    /// \brief Gazebo communication node.
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

    public: AlignConfig config{AlignConfig::ALIGN_MID};

    /// \brief Flag to indicate if the align direction should be reversed.
    public: bool reverse{false};

    /// \brief Flag to indicate if the entities to should aligned to the first
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

    /// \brief Pointer to the scene.
    public: rendering::ScenePtr scene{nullptr};
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
AlignTool::AlignTool()
  : GuiSystem(), dataPtr(std::make_unique<AlignToolPrivate>())
{
  // Deselect all entities upon loading plugin
  gui::events::DeselectAllEntities deselectEvent(true);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &deselectEvent);
}

/////////////////////////////////////////////////
AlignTool::~AlignTool() = default;

/////////////////////////////////////////////////
void AlignTool::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Align tool";

  // For align tool requests
  gz::gui::App()->findChild
      <gz::gui::MainWindow *>()->installEventFilter(this);
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
void AlignTool::OnAlignConfig(const QString &_config)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  std::string newConfig = _config.toStdString();
  std::transform(newConfig.begin(), newConfig.end(),
                 newConfig.begin(), ::tolower);

  if (newConfig == "min")
  {
    this->dataPtr->config = AlignConfig::ALIGN_MIN;
  }
  else if (newConfig == "mid")
  {
    this->dataPtr->config = AlignConfig::ALIGN_MID;
  }
  else if (newConfig == "max")
  {
    this->dataPtr->config = AlignConfig::ALIGN_MAX;
  }
  else
  {
    gzwarn << "Invalid align axis config: " << newConfig << "\n";
    gzwarn << "The valid options are:\n";
    gzwarn << " - min\n";
    gzwarn << " - mid\n";
    gzwarn << " - max\n";
  }
}

/////////////////////////////////////////////////
void AlignTool::OnReverse(bool _reverse)
{
  this->dataPtr->reverse = _reverse;
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
    gzwarn << "Invalid align axis string: " << newAxis << "\n";
    gzwarn << "The valid options are:\n";
    gzwarn << " - X\n";
    gzwarn << " - Y\n";
    gzwarn << " - Z\n";
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
    gzwarn << "Invalid align target string: " << newTarget << "\n";
    gzwarn << "The valid options are:\n";
    gzwarn << " - first\n";
    gzwarn << " - last\n";
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
void AlignTool::UpdateTransparency(const rendering::NodePtr &_node,
    bool _makeTransparent)
{
  if (!_node)
    return;

  for (auto n = 0u; n < _node->ChildCount(); ++n)
  {
    auto child = _node->ChildByIndex(n);
    this->UpdateTransparency(child, _makeTransparent);
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
    if (_makeTransparent)
    {
      // If the entity isn't already transparent, make it transparent
      if (visTransparency == this->dataPtr->originalTransparency.end())
      {
        this->dataPtr->originalTransparency[vis->Name()] =
          visMat->Transparency();
        visMat->SetTransparency(1.0 - ((1.0 - visMat->Transparency()) * 0.5));
      }
    }
    else
    {
      if (visTransparency != this->dataPtr->originalTransparency.end())
      {
        visMat->SetTransparency(visTransparency->second);
      }
    }
  }

  for (auto g = 0u; g < vis->GeometryCount(); ++g)
  {
    auto geom = vis->GeometryByIndex(g);
    auto wireBox = std::dynamic_pointer_cast<rendering::WireBox>(geom);

    // Skip wirebox geometry
    if (wireBox)
      continue;

    // Geometry material
    auto geomMat = geom->Material();
    if (nullptr == geomMat)
      continue;
    auto geomTransparency =
        this->dataPtr->originalTransparency.find(geom->Name());

    if (_makeTransparent)
    {
      // If the entity isn't already transparent, make it transparent
      if (geomTransparency == this->dataPtr->originalTransparency.end())
      {
        this->dataPtr->originalTransparency[geom->Name()] =
            geomMat->Transparency();
        geomMat->SetTransparency(1.0 - ((1.0 - geomMat->Transparency()) * 0.5));
      }
    }
    else
    {
      if (geomTransparency != this->dataPtr->originalTransparency.end())
      {
        geomMat->SetTransparency(geomTransparency->second);
      }
    }
  }
}

/////////////////////////////////////////////////
void AlignTool::Align()
{
  if (this->dataPtr->currentState == AlignState::NONE)
    return;

  // load scene
  if (!this->dataPtr->scene)
    this->dataPtr->scene = rendering::sceneFromFirstRenderEngine();

  // Get current list of selected entities
  std::vector<rendering::VisualPtr> selectedList;
  rendering::VisualPtr relativeVisual;

  for (const auto &entityId : this->dataPtr->selectedEntities)
  {
    for (auto i = 0u; i < this->dataPtr->scene->VisualCount(); ++i)
    {
      rendering::VisualPtr vis =
        this->dataPtr->scene->VisualByIndex(i);
      if (!vis)
        continue;

      if (vis->HasUserData("gazebo-entity") &&
          std::get<uint64_t>(vis->UserData("gazebo-entity")) == entityId)
      {
        // Check here to see if visual is top level or not, continue if not
        auto topLevelVis = this->TopLevelVisual(this->dataPtr->scene, vis);
        if (topLevelVis != vis)
          continue;

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

  // Callback function for Gazebo node request
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/* _rep*/, const bool _result)
  {
    if (!_result)
      gzerr << "Error setting pose" << std::endl;
  };

  // Set service topic
  if (this->dataPtr->poseCmdService.empty())
  {
    this->dataPtr->poseCmdService = "/world/" + this->dataPtr->worldName
        + "/set_pose";
  }

  int axisIndex = static_cast<int>(this->dataPtr->axis);
  msgs::Pose req;

  gz::math::AxisAlignedBox targetBox = relativeVisual->BoundingBox();
  gz::math::Vector3d targetMin = targetBox.Min();
  gz::math::Vector3d targetMax = targetBox.Max();

  // Index math to avoid iterating through the selected node
  for (unsigned int i = this->dataPtr->first;
       i < selectedList.size() + this->dataPtr->first - 1; i++)
  {
    rendering::VisualPtr vis = selectedList[i];
    if (!vis)
      continue;

    gz::math::AxisAlignedBox box = vis->BoundingBox();
    gz::math::Vector3d min = box.Min();
    gz::math::Vector3d max = box.Max();

    // Check here to see if visual is top level or not, continue if not
    rendering::VisualPtr topLevelVis = this->TopLevelVisual(
        this->dataPtr->scene, vis);
    if (topLevelVis != vis)
      continue;

    math::Vector3d newPos = vis->WorldPosition();

    // If a reset is occurring, pull all of the previous positions and set
    if (this->dataPtr->currentState == AlignState::RESET)
    {
      // Index offset given the relative node does not get added to the
      // prevPositions vector
      newPos = this->dataPtr->prevPositions[i-this->dataPtr->first];

      this->UpdateTransparency(vis, false /* opaque */);
      vis->SetWorldPosition(newPos);
      vis->SetUserData("pause-update", static_cast<int>(0));
    }
    // If an align is occurring, the user has clicked the align button,
    // make a request to the backend to send the node to the new position
    else if (this->dataPtr->currentState == AlignState::ALIGN)
    {
      this->UpdateTransparency(vis, false /* opaque */);
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
      double translation = 0.0;
      if (this->dataPtr->config == AlignConfig::ALIGN_MID)
      {
        translation = (targetMin[axisIndex] + (targetMax[axisIndex]
                                 - targetMin[axisIndex]) / 2) - (min[axisIndex]
                                 + (max[axisIndex] - min[axisIndex]) / 2);
      }
      else
      {
        if (this->dataPtr->reverse)
        {
          if (this->dataPtr->config == AlignConfig::ALIGN_MIN)
            translation = targetMin[axisIndex] - max[axisIndex];
          else if (this->dataPtr->config == AlignConfig::ALIGN_MAX)
            translation = targetMax[axisIndex] - min[axisIndex];
        }
        else
        {
          if (this->dataPtr->config == AlignConfig::ALIGN_MIN)
            translation = targetMin[axisIndex] - min[axisIndex];
          else if (this->dataPtr->config == AlignConfig::ALIGN_MAX)
            translation = targetMax[axisIndex] - max[axisIndex];
        }
      }

      // Add calculated translation to the chosen index
      newPos[axisIndex] += translation;

      // Store the vis's world positions in a vector
      this->dataPtr->prevPositions.push_back(vis->WorldPosition());

      // Make the visual transparent and update to new position
      this->UpdateTransparency(vis, true /* transparent */);
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
  if (_event->type() == gz::gui::events::Render::kType)
  {
    // This event is called in the RenderThread, so it's safe to make
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
           gui::events::EntitiesSelected::kType)
  {
    auto selectedEvent =
        reinterpret_cast<sim::gui::events::EntitiesSelected *>(
        _event);

    // Only update if a valid cast, the data isn't empty, and
    // the command is not from user (sent from backend)
    if (selectedEvent && !selectedEvent->Data().empty())
    {
      for (const auto &_entity : selectedEvent->Data())
      {
        // If the element already exists in the selected entity's
        // vector, then continue
        if (std::find(this->dataPtr->selectedEntities.begin(),
              this->dataPtr->selectedEntities.end(),
              _entity) != this->dataPtr->selectedEntities.end())
          continue;
        this->dataPtr->selectedEntities.push_back(_entity);
      }
    }
  }
  else if (_event->type() ==
           gui::events::DeselectAllEntities::kType)
  {
    this->dataPtr->selectedEntities.clear();
  }
  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
GZ_ADD_PLUGIN(AlignTool,
              gz::gui::Plugin)

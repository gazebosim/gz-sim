/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ignition/common/MouseEvent.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/WireBox.hh>
#include "ignition/rendering/Camera.hh"

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"

#include "SelectEntities.hh"

namespace ignition
{
namespace gazebo
{
/// \brief Helper to store selection requests to be handled in the render
/// thread by `IgnRenderer::HandleEntitySelection`.
struct SelectionHelper
{
  /// \brief Entity to be selected
  Entity selectEntity{kNullEntity};

  /// \brief Deselect all entities
  bool deselectAll{false};

  /// \brief True to send an event and notify all widgets
  bool sendEvent{false};
};
}
}

/// \brief Private data class for SelectEntities
class ignition::gazebo::plugins::SelectEntitiesPrivate
{
  public: void Initialize();

  public: void HandleEntitySelection();

  public: void UpdateSelectedEntity(const rendering::VisualPtr &_visual,
    bool _sendEvent);

  public: void HighlightNode(const rendering::VisualPtr &_visual);

  public: void SetSelectedEntity(const rendering::VisualPtr &_visual);

  public: void DeselectAllEntities();

  public: void LowlightNode(const rendering::VisualPtr &_visual);

  public: rendering::NodePtr TopLevelNode(
      const rendering::NodePtr &_node);

  /// \brief Helper object to select entities. Only the latest selection
  /// request is kept.
  public: SelectionHelper selectionHelper;

  /// \brief Currently selected entities, organized by order of selection.
  public: std::vector<Entity> selectedEntities;

  /// \brief Currently selected entities, organized by order of selection.
  public: std::vector<Entity> selectedEntitiesID;

  /// \brief New entities received from other plugins.
  public: std::vector<Entity> selectedEntitiesIDNew;

  //// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene = nullptr;

  /// \brief A map of entity ids and wire boxes
  public: std::unordered_map<Entity,
    ignition::rendering::WireBoxPtr> wireBoxes;

  /// \brief MouseEvent
  public: ignition::common::MouseEvent mouseEvent;

  /// \brief is the mouse modify ?
  public: bool mouseDirty = false;

  /// \brief selected entities from other plugins (for example: entityTree)
  public: bool recievedSelectedEntities = false;

  /// \brief User camera
  public: rendering::CameraPtr camera = nullptr;

  /// \brief is transform control active ?
  public: bool transformControlActive = false;
};

using namespace ignition;
using namespace gazebo;
using namespace plugins;

/////////////////////////////////////////////////
void SelectEntitiesPrivate::HandleEntitySelection()
{
  if (this->recievedSelectedEntities)
  {
    if (!(QGuiApplication::keyboardModifiers() & Qt::ControlModifier))
    {
      this->DeselectAllEntities();
    }

    for (unsigned int i = 0; i < this->selectedEntitiesIDNew.size(); i++)
    {
      auto visualToHighLight = this->scene->VisualById(
        this->selectedEntitiesIDNew[i]);
      this->selectedEntitiesID.push_back(this->selectedEntitiesIDNew[i]);
      this->selectedEntities.push_back(
        std::get<int>(visualToHighLight->UserData("gazebo-entity")));
      this->HighlightNode(visualToHighLight);
      ignition::gazebo::gui::events::EntitiesSelected selectEvent(
          this->selectedEntities);
      ignition::gui::App()->sendEvent(
          ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
          &selectEvent);
    }
    this->recievedSelectedEntities = false;
    this->selectionHelper = SelectionHelper();
    this->selectedEntitiesIDNew.clear();
  }

  if (!mouseDirty)
    return;

  this->mouseDirty = false;

  rendering::VisualPtr visual = this->scene->VisualAt(
        this->camera,
        this->mouseEvent.Pos());

  if (!visual)
  {
    this->DeselectAllEntities();
    return;
  }

  this->selectionHelper.selectEntity =
    std::get<int>(visual->UserData("gazebo-entity"));

  if (this->selectionHelper.deselectAll)
  {
    this->DeselectAllEntities();

    this->selectionHelper = SelectionHelper();
  }
  else if (this->selectionHelper.selectEntity != kNullEntity)
  {
    this->UpdateSelectedEntity(visual,
        this->selectionHelper.sendEvent);

    this->selectionHelper = SelectionHelper();
  }
}

////////////////////////////////////////////////
void SelectEntitiesPrivate::LowlightNode(const rendering::VisualPtr &_visual)
{
  Entity entityId = kNullEntity;
  if (_visual)
    entityId = std::get<int>(_visual->UserData("gazebo-entity"));
  if (this->wireBoxes.find(entityId) != this->wireBoxes.end())
  {
    ignition::rendering::WireBoxPtr wireBox =
      this->wireBoxes[entityId];
    auto visParent = wireBox->Parent();
    if (visParent)
      visParent->SetVisible(false);
  }
}

////////////////////////////////////////////////
void SelectEntitiesPrivate::HighlightNode(const rendering::VisualPtr &_visual)
{
  Entity entityId = kNullEntity;

  if (_visual)
    entityId = std::get<int>(_visual->UserData("gazebo-entity"));

  // If the entity is not found in the existing map, create a wire box
  auto wireBoxIt = this->wireBoxes.find(entityId);
  if (wireBoxIt == this->wireBoxes.end())
  {
    auto white = this->scene->Material("highlight_material");
    if (!white)
    {
      white = this->scene->CreateMaterial("highlight_material");
      white->SetAmbient(1.0, 1.0, 1.0);
      white->SetDiffuse(1.0, 1.0, 1.0);
      white->SetSpecular(1.0, 1.0, 1.0);
      white->SetEmissive(1.0, 1.0, 1.0);
    }

    ignition::rendering::WireBoxPtr wireBox =
      this->scene->CreateWireBox();
    ignition::math::AxisAlignedBox aabb = _visual->LocalBoundingBox();
    wireBox->SetBox(aabb);

    // Create visual and add wire box
    ignition::rendering::VisualPtr wireBoxVis =
      this->scene->CreateVisual();
    wireBoxVis->SetInheritScale(false);
    wireBoxVis->AddGeometry(wireBox);
    wireBoxVis->SetMaterial(white, false);
    _visual->AddChild(wireBoxVis);

    // Add wire box to map for setting visibility
    this->wireBoxes.insert(
        std::pair<Entity, ignition::rendering::WireBoxPtr>(entityId, wireBox));
  }
  else
  {
    ignition::rendering::WireBoxPtr wireBox = wireBoxIt->second;
    ignition::math::AxisAlignedBox aabb = _visual->LocalBoundingBox();
    wireBox->SetBox(aabb);
    auto visParent = wireBox->Parent();
    if (visParent)
      visParent->SetVisible(true);
  }
}

/////////////////////////////////////////////////
rendering::NodePtr SelectEntitiesPrivate::TopLevelNode(
    const rendering::NodePtr &_node)
{
  if (!this->scene)
    return rendering::NodePtr();

  rendering::NodePtr rootNode = this->scene->RootVisual();

  rendering::NodePtr nodeTmp = _node;
  while (nodeTmp && nodeTmp->Parent() != rootNode)
  {
    nodeTmp =
      std::dynamic_pointer_cast<rendering::Node>(nodeTmp->Parent());
  }

  return nodeTmp;
}

/////////////////////////////////////////////////
void SelectEntitiesPrivate::SetSelectedEntity(
  const rendering::VisualPtr &_visual)
{
  Entity entityId = kNullEntity;

  auto topLevelNode = this->TopLevelNode(_visual);
  auto topLevelVisual = std::dynamic_pointer_cast<rendering::Visual>(
    topLevelNode);

  if (topLevelVisual)
  {
    entityId = std::get<int>(topLevelVisual->UserData("gazebo-entity"));
  }

  if (entityId == kNullEntity)
    return;

  this->selectedEntities.push_back(entityId);
  this->selectedEntitiesID.push_back(_visual->Id());
  this->HighlightNode(_visual);
  ignition::gazebo::gui::events::EntitiesSelected entitiesSelected(
    this->selectedEntities);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      &entitiesSelected);
}

/////////////////////////////////////////////////
void SelectEntitiesPrivate::DeselectAllEntities()
{
  if (this->scene)
  {
    for (const auto &entity : this->selectedEntitiesID)
    {
      auto node = this->scene->VisualById(entity);
      auto vis = std::dynamic_pointer_cast<rendering::Visual>(node);
      this->LowlightNode(vis);
    }
    this->selectedEntities.clear();
    this->selectedEntitiesID.clear();

    ignition::gazebo::gui::events::DeselectAllEntities deselectEvent(true);
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &deselectEvent);
  }
}

/////////////////////////////////////////////////
void SelectEntitiesPrivate::UpdateSelectedEntity(
  const rendering::VisualPtr &_visual, bool _sendEvent)
{
  bool deselectedAll{false};

  // Deselect all if control is not being held
  if ((!(QGuiApplication::keyboardModifiers() & Qt::ControlModifier) &&
      !this->selectedEntitiesID.empty()) || this->transformControlActive)
  {
    // Notify other widgets regardless of _sendEvent, because this is a new
    // decision from this widget
    this->DeselectAllEntities();
    deselectedAll = true;
  }

  // Select new entity
  this->SetSelectedEntity(_visual);

  // Notify other widgets of the currently selected entities
  if (_sendEvent || deselectedAll)
  {
    ignition::gazebo::gui::events::EntitiesSelected selectEvent(
        this->selectedEntities);
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &selectEvent);
  }
}

/////////////////////////////////////////////////
void SelectEntitiesPrivate::Initialize()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
      return;

    for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        this->scene->NodeByIndex(i));
      if (cam)
      {
        if (cam->Name().find("scene::Camera") != std::string::npos)
        {
          this->camera = cam;
          igndbg << "InteractiveViewControl plugin is moving camera ["
                 << this->camera->Name() << "]" << std::endl;
          break;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
SelectEntities::SelectEntities()
  : GuiSystem(), dataPtr(new SelectEntitiesPrivate)
{
}

/////////////////////////////////////////////////
SelectEntities::~SelectEntities()
{
}

/////////////////////////////////////////////////
void SelectEntities::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Select entities";

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool SelectEntities::eventFilter(QObject *_obj, QEvent *_event)
{
  // std::cerr << "_obj " << _obj << " this " << this << '\n';
  if (_event->type() == ignition::gui::events::LeftClickOnScene::kType)
  {
    ignition::gui::events::LeftClickOnScene *_e =
      static_cast<ignition::gui::events::LeftClickOnScene*>(_event);
    this->dataPtr->mouseEvent = _e->Mouse();
    // handle transform control
    if (this->dataPtr->mouseEvent.Button() ==
      ignition::common::MouseEvent::LEFT)
    {
      if (this->dataPtr->mouseEvent.Type() ==
        ignition::common::MouseEvent::PRESS)
      {
        this->dataPtr->mouseDirty = true;
      }
    }
  }
  else if (_event->type() == ignition::gui::events::Render::kType)
  {
    this->dataPtr->Initialize();
    this->dataPtr->HandleEntitySelection();
  }
  else if (_event->type() ==
    ignition::gazebo::gui::events::TransformControlMode::kType)
  {
    auto transformControlMode =
      reinterpret_cast<ignition::gazebo::gui::events::TransformControlMode *>(
        _event);
    this->dataPtr->transformControlActive =
      transformControlMode->TransformControl();
  }
  else if (_event->type() ==
    ignition::gazebo::gui::events::EntitiesSelected::kType)
  {
    auto selectedEvent =
        reinterpret_cast<gui::events::EntitiesSelected *>(_event);
    if (selectedEvent && !selectedEvent->Data().empty() &&
      selectedEvent->FromUser())
    {
      for (const auto &entity : selectedEvent->Data())
      {
        for (unsigned int i = 0; i < this->dataPtr->scene->VisualCount(); i++)
        {
          auto visual = this->dataPtr->scene->VisualByIndex(i);
          auto entityId = static_cast<unsigned int>(
              std::get<int>(visual->UserData("gazebo-entity")));
          if (entityId == entity)
          {
            this->dataPtr->selectedEntitiesIDNew.push_back(visual->Id());
            this->dataPtr->recievedSelectedEntities = true;
            break;
          }
        }
      }
    }
  }
  else if (_event->type() ==
           ignition::gazebo::gui::events::DeselectAllEntities::kType)
  {
    this->dataPtr->selectedEntitiesID.clear();
    this->dataPtr->selectedEntities.clear();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::plugins::SelectEntities,
                    ignition::gui::Plugin)

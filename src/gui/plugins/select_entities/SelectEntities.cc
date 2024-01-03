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

#include <QQmlProperty>

#include <gz/common/MouseEvent.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/WireBox.hh>
#include "gz/rendering/Camera.hh"

#include "gz/sim/Entity.hh"
#include "gz/sim/gui/GuiEvents.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/rendering/RenderUtil.hh"

#include "SelectEntities.hh"

namespace gz
{
namespace sim
{
namespace gui
{
/// \brief Helper to store selection requests to be handled in the render
/// thread by `GzRenderer::HandleEntitySelection`.
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
}

/// \brief Private data class for SelectEntities
class gz::sim::gui::SelectEntitiesPrivate
{
  /// \brief Initialize the plugin, attaching to a camera.
  public: void Initialize();

  /// \brief Handle entity selection in the render thread.
  public: void HandleEntitySelection();

  /// \brief Select new entity
  /// \param[in] _visual Visual that was clicked
  /// \param[in] _sendEvent True to send an event and notify other widgets
  public: void UpdateSelectedEntity(const rendering::VisualPtr &_visual,
    bool _sendEvent);

  /// \brief Highlight a selected rendering node
  /// \param[in] _visual Node to be highlighted
  public: void HighlightNode(const rendering::VisualPtr &_visual);

  /// \brief Remove highlight from a rendering node that's no longer selected
  /// \param[in] _visual Node to be lowlighted
  public: void LowlightNode(const rendering::VisualPtr &_visual);

  /// \brief Select the entity for the given visual
  /// \param[in] _visual Visual to select
  public: void SetSelectedEntity(const rendering::VisualPtr &_visual);

  /// \brief Deselect all selected entities.
  public: void DeselectAllEntities();

  /// \brief Get the ancestor of a given node which is a direct child of the
  /// world.
  /// \param[in] _node Node to get ancestor of
  /// \return Top level node.
  public: rendering::NodePtr TopLevelNode(
      const rendering::NodePtr &_node);

  /// \brief Helper object to select entities. Only the latest selection
  /// request is kept.
  public: SelectionHelper selectionHelper;

  /// \brief Currently selected entities, organized by order of selection.
  /// These are gz-sim IDs
  public: std::vector<Entity> selectedEntities;

  /// \brief Currently selected entities, organized by order of selection.
  /// These are gz-rendering IDs
  public: std::vector<unsigned int> selectedEntitiesID;

  /// \brief New entities received from other plugins.
  /// These are gz-rendering IDs
  public: std::vector<unsigned int> selectedEntitiesIDNew;

  //// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene = nullptr;

  /// \brief A map of entity ids and wire boxes
  public: std::unordered_map<Entity,
    gz::rendering::WireBoxPtr> wireBoxes;

  /// \brief MouseEvent
  public: gz::common::MouseEvent mouseEvent;

  /// \brief is the mouse modify ?
  public: bool mouseDirty = false;

  /// \brief selected entities from other plugins (for example: entityTree)
  public: bool receivedSelectedEntities = false;

  /// \brief User camera
  public: rendering::CameraPtr camera = nullptr;

  /// \brief is transform control active ?
  public: bool transformControlActive = false;

  /// \brief Is an entity being spawned
  public: bool isSpawning{false};
};

using namespace gz;
using namespace sim;
using namespace sim::gui;

/////////////////////////////////////////////////
void SelectEntitiesPrivate::HandleEntitySelection()
{
  if (this->receivedSelectedEntities)
  {
    if (!(QGuiApplication::keyboardModifiers() & Qt::ControlModifier))
    {
      this->DeselectAllEntities();
    }

    for (unsigned int i = 0; i < this->selectedEntitiesIDNew.size(); i++)
    {
      auto visualToHighLight = this->scene->VisualById(
        this->selectedEntitiesIDNew[i]);

      if (nullptr == visualToHighLight)
      {
        gzerr << "Failed to get visual with ID ["
               << this->selectedEntitiesIDNew[i] << "]" << std::endl;
        continue;
      }

      this->selectedEntitiesID.push_back(this->selectedEntitiesIDNew[i]);

      Entity entityId = kNullEntity;
      try
      {
        entityId = std::get<uint64_t>(
          visualToHighLight->UserData("gazebo-entity"));
      }
      catch(std::bad_variant_access &_e)
      {
        // It's ok to get here
      }

      this->selectedEntities.push_back(entityId);

      this->HighlightNode(visualToHighLight);

      gz::sim::gui::events::EntitiesSelected selectEvent(
          this->selectedEntities);
      gz::gui::App()->sendEvent(
          gz::gui::App()->findChild<gz::gui::MainWindow *>(),
          &selectEvent);
    }
    this->receivedSelectedEntities = false;
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

  Entity entityId = kNullEntity;
  try
  {
    entityId = std::get<uint64_t>(visual->UserData("gazebo-entity"));
  }
  catch(std::bad_variant_access &e)
  {
    // It's ok to get here
  }

  this->selectionHelper.selectEntity = entityId;

  if (this->selectionHelper.deselectAll)
  {
    this->DeselectAllEntities();

    this->selectionHelper = SelectionHelper();
  }
  else if (this->selectionHelper.selectEntity != kNullEntity)
  {
    this->UpdateSelectedEntity(visual, this->selectionHelper.sendEvent);

    this->selectionHelper = SelectionHelper();
  }
}

////////////////////////////////////////////////
void SelectEntitiesPrivate::LowlightNode(const rendering::VisualPtr &_visual)
{
  Entity entityId = kNullEntity;
  if (_visual)
  {
    try
    {
      entityId = std::get<uint64_t>(_visual->UserData("gazebo-entity"));
    }
    catch(std::bad_variant_access &)
    {
      // It's ok to get here
    }
  }
  if (this->wireBoxes.find(entityId) != this->wireBoxes.end())
  {
    gz::rendering::WireBoxPtr wireBox = this->wireBoxes[entityId];
    auto visParent = wireBox->Parent();
    if (visParent)
      visParent->SetVisible(false);
  }
}

////////////////////////////////////////////////
void SelectEntitiesPrivate::HighlightNode(const rendering::VisualPtr &_visual)
{
  if (nullptr == _visual)
  {
    gzerr << "Failed to highlight null visual." << std::endl;
    return;
  }

  Entity entityId = kNullEntity;
  try
  {
    entityId = std::get<uint64_t>(_visual->UserData("gazebo-entity"));
  }
  catch(std::bad_variant_access &)
  {
    // It's ok to get here
  }

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

    gz::rendering::WireBoxPtr wireBox = this->scene->CreateWireBox();
    gz::math::AxisAlignedBox aabb = _visual->LocalBoundingBox();
    wireBox->SetBox(aabb);

    // Create visual and add wire box
    gz::rendering::VisualPtr wireBoxVis = this->scene->CreateVisual();
    wireBoxVis->SetInheritScale(false);
    wireBoxVis->AddGeometry(wireBox);
    wireBoxVis->SetMaterial(white, false);
    wireBoxVis->SetUserData("gui-only", static_cast<bool>(true));
    _visual->AddChild(wireBoxVis);

    // Add wire box to map for setting visibility
    this->wireBoxes.insert(
        std::pair<Entity, gz::rendering::WireBoxPtr>(entityId, wireBox));
  }
  else
  {
    gz::rendering::WireBoxPtr wireBox = wireBoxIt->second;
    gz::math::AxisAlignedBox aabb = _visual->LocalBoundingBox();
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
  if (nullptr == _visual)
  {
    gzerr << "Failed to select null visual" << std::endl;
    return;
  }

  Entity entityId = kNullEntity;

  auto topLevelNode = this->TopLevelNode(_visual);
  auto topLevelVisual = std::dynamic_pointer_cast<rendering::Visual>(
    topLevelNode);

  if (topLevelVisual)
  {
    try
    {
      entityId = std::get<uint64_t>(topLevelVisual->UserData("gazebo-entity"));
    }
    catch(std::bad_variant_access &)
    {
      // It's ok to get here
    }
  }

  if (entityId == kNullEntity)
    return;

  this->selectedEntities.push_back(entityId);
  this->selectedEntitiesID.push_back(topLevelVisual->Id());
  this->HighlightNode(topLevelVisual);
  gz::sim::gui::events::EntitiesSelected entitiesSelected(
    this->selectedEntities);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &entitiesSelected);
}

/////////////////////////////////////////////////
void SelectEntitiesPrivate::DeselectAllEntities()
{
  if (nullptr == this->scene)
    return;

  for (const auto &entity : this->selectedEntitiesID)
  {
    auto node = this->scene->VisualById(entity);
    auto vis = std::dynamic_pointer_cast<rendering::Visual>(node);
    this->LowlightNode(vis);
  }
  this->selectedEntities.clear();
  this->selectedEntitiesID.clear();

  gz::sim::gui::events::DeselectAllEntities deselectEvent(true);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &deselectEvent);
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
    gz::sim::gui::events::EntitiesSelected selectEvent(
        this->selectedEntities);
    gz::gui::App()->sendEvent(
        gz::gui::App()->findChild<gz::gui::MainWindow *>(),
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

    for (unsigned int i = 0; i < scene->NodeCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        scene->NodeByIndex(i));
      if (cam && cam->HasUserData("user-camera") &&
          std::get<bool>(cam->UserData("user-camera")))
      {
        this->camera = cam;
        gzdbg << "SelectEntities plugin is using camera ["
               << this->camera->Name() << "]" << std::endl;
        break;
      }
    }

    if (!this->camera)
    {
      gzerr << "TransformControl camera is not available" << std::endl;
      return;
    }
  }
}

/////////////////////////////////////////////////
SelectEntities::SelectEntities()
  : dataPtr(std::make_unique<SelectEntitiesPrivate>())
{
}

/////////////////////////////////////////////////
SelectEntities::~SelectEntities() = default;

/////////////////////////////////////////////////
void SelectEntities::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Select entities";

  static bool done{false};
  if (done)
  {
    std::string msg{"Only one SelectEntities plugin is supported at a time."};
    gzerr << msg << std::endl;
    QQmlProperty::write(this->PluginItem(), "message",
        QString::fromStdString(msg));
    return;
  }
  done = true;

  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool SelectEntities::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::LeftClickOnScene::kType)
  {
    gz::gui::events::LeftClickOnScene *_e =
      static_cast<gz::gui::events::LeftClickOnScene*>(_event);
    this->dataPtr->mouseEvent = _e->Mouse();

    if (this->dataPtr->mouseEvent.Button() == common::MouseEvent::LEFT &&
        this->dataPtr->mouseEvent.Type() == common::MouseEvent::RELEASE)
    {
      if (this->dataPtr->isSpawning)
      {
        this->dataPtr->isSpawning = false;
      }
      else
      {
        this->dataPtr->mouseDirty = true;
      }
    }
  }
  else if (_event->type() == gz::gui::events::Render::kType)
  {
    this->dataPtr->Initialize();
    this->dataPtr->HandleEntitySelection();
  }
  else if (_event->type() ==
    gz::sim::gui::events::TransformControlModeActive::kType)
  {
    auto transformControlMode =
      reinterpret_cast<sim::gui::events::TransformControlModeActive *>(
        _event);
    this->dataPtr->transformControlActive =
      transformControlMode->TransformControlActive();
  }
  else if (_event->type() ==
    gz::sim::gui::events::EntitiesSelected::kType)
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

          Entity entityId = kNullEntity;
          try
          {
            entityId = std::get<uint64_t>(visual->UserData("gazebo-entity"));
          }
          catch(std::bad_variant_access &)
          {
            // It's ok to get here
          }

          if (entityId == entity)
          {
            this->dataPtr->selectedEntitiesIDNew.push_back(visual->Id());
            this->dataPtr->receivedSelectedEntities = true;
            break;
          }
        }
      }
    }
  }
  else if (_event->type() ==
           gz::sim::gui::events::DeselectAllEntities::kType)
  {
    this->dataPtr->selectedEntitiesID.clear();
    this->dataPtr->selectedEntities.clear();
  }
  else if (_event->type() ==
    gz::gui::events::SpawnFromDescription::kType ||
    _event->type() == gz::gui::events::SpawnFromPath::kType)
  {
    this->dataPtr->isSpawning = true;
    this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() == gz::gui::events::KeyReleaseOnScene::kType)
  {
    gz::gui::events::KeyReleaseOnScene *_e =
      static_cast<gz::gui::events::KeyReleaseOnScene*>(_event);
    if (_e->Key().Key() == Qt::Key_Escape)
    {
      this->dataPtr->mouseDirty = true;
      this->dataPtr->selectionHelper.deselectAll = true;
      this->dataPtr->isSpawning = false;
    }
  }
  else if (_event->type() ==
           gz::sim::gui::events::NewRemovedEntities::kType)
  {
    if (!this->dataPtr->wireBoxes.empty())
    {
      auto event =
          reinterpret_cast<gui::events::NewRemovedEntities *>(_event);
      for (auto &entity : event->RemovedEntities())
      {
        auto wireBoxIt = this->dataPtr->wireBoxes.find(entity);
        if (wireBoxIt != this->dataPtr->wireBoxes.end())
        {
          this->dataPtr->scene->DestroyVisual(wireBoxIt->second->Parent());
          this->dataPtr->wireBoxes.erase(wireBoxIt);
        }
      }

    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::gui::SelectEntities,
                    gz::gui::Plugin)

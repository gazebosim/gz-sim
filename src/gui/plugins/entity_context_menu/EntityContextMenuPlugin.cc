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

#include "EntityContextMenuPlugin.hh"

#include <memory>
#include <utility>

#include <QtQml>

#include <gz/common/Console.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>

#include <gz/plugin/Register.hh>

#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Scene.hh>

namespace gz::sim
{
  class EntityContextMenuPrivate
  {
    /// \brief Perform operations in the render thread.
    public: void OnRender();

    /// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene{nullptr};

    /// \brief User camera
    public: rendering::CameraPtr camera{nullptr};

    /// \brief Entity context menu handler
    public: EntityContextMenuHandler entityContextMenuHandler;
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
void EntityContextMenuPrivate::OnRender()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
    {
      return;
    }

    for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        this->scene->NodeByIndex(i));
      if (cam && cam->HasUserData("user-camera") &&
          std::get<bool>(cam->UserData("user-camera")))
      {
        this->camera = cam;

        gzdbg << "Entity context menu plugin is using camera ["
               << this->camera->Name() << "]" << std::endl;
        break;
      }
    }
  }
}

/////////////////////////////////////////////////
EntityContextMenu::EntityContextMenu()
  : gui::Plugin(), dataPtr(std::make_unique<EntityContextMenuPrivate>())
{
  qmlRegisterType<EntityContextMenuItem>(
    "RenderWindowOverlay", 1, 0, "RenderWindowOverlay");
}

EntityContextMenu::~EntityContextMenu() = default;

/////////////////////////////////////////////////
void EntityContextMenu::LoadConfig(const tinyxml2::XMLElement *)
{
  EntityContextMenuItem *renderWindowOverlay =
      this->PluginItem()->findChild<EntityContextMenuItem *>();
  if (!renderWindowOverlay)
  {
    gzerr << "Unable to find Render Window Overlay item. "
           << "Render window overlay will not be created" << std::endl;
    return;
  }

  renderWindowOverlay->SetEntityContextMenuHandler(
    this->dataPtr->entityContextMenuHandler);

  if (this->title.empty())
    this->title = "Entity Context Menu";

  gz::gui::App()->findChild
    <gz::gui::MainWindow *>()->installEventFilter(this);
}

////////////////////////////////////////////////
bool EntityContextMenu::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  else if (_event->type() == gz::gui::events::RightClickOnScene::kType)
  {
    gz::gui::events::RightClickOnScene *_e =
      static_cast<gz::gui::events::RightClickOnScene*>(_event);
    if (_e)
    {
      this->dataPtr->entityContextMenuHandler.HandleMouseContextMenu(
        _e->Mouse(), this->dataPtr->camera);
    }
  }

  return QObject::eventFilter(_obj, _event);
}


/////////////////////////////////////////////////
EntityContextMenuItem::EntityContextMenuItem(QQuickItem *_parent)
  : QQuickItem(_parent)
{
  this->setAcceptedMouseButtons(Qt::AllButtons);
  this->setFlag(ItemHasContents);
}

/////////////////////////////////////////////////
void EntityContextMenuItem::SetEntityContextMenuHandler(
  const EntityContextMenuHandler &_entityContextMenuHandler)
{
  this->connect(
    &_entityContextMenuHandler,
    &EntityContextMenuHandler::ContextMenuRequested,
    this,
    &EntityContextMenuItem::OnContextMenuRequested,
    Qt::QueuedConnection);
}

///////////////////////////////////////////////////
void EntityContextMenuItem::OnContextMenuRequested(
  QString _entity, int _mouseX, int _mouseY)
{
  emit openContextMenu(std::move(_entity), _mouseX, _mouseY);
}

/////////////////////////////////////////////////
EntityContextMenuHandler::EntityContextMenuHandler()
{
}

void EntityContextMenuHandler::HandleMouseContextMenu(
  const common::MouseEvent &_mouseEvent, const rendering::CameraPtr &_camera)
{
  if (!_mouseEvent.Dragging() &&
      _mouseEvent.Type() == common::MouseEvent::RELEASE &&
      _mouseEvent.Button() == common::MouseEvent::RIGHT)
  {
    math::Vector2i dt =
      _mouseEvent.PressPos() - _mouseEvent.Pos();

    // check for click with some tol for mouse movement
    if (dt.Length() > 5.0)
      return;

    rendering::VisualPtr visual = _camera->Scene()->VisualAt(
          _camera,
          _mouseEvent.Pos());

    if (!visual)
      return;

    // get model visual
    while (visual->HasParent() && visual->Parent() !=
        visual->Scene()->RootVisual())
    {
      visual = std::dynamic_pointer_cast<rendering::Visual>(visual->Parent());
    }

    emit ContextMenuRequested(
      visual->Name().c_str(), _mouseEvent.Pos().X(), _mouseEvent.Pos().Y());
  }
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::EntityContextMenu,
                    gz::gui::Plugin)

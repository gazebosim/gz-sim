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

#ifndef GZ_GUI_PLUGINS_ENTITY_CONTEXT_MENU_HH_
#define GZ_GUI_PLUGINS_ENTITY_CONTEXT_MENU_HH_

#include <memory>

#include <gz/common/MouseEvent.hh>

#include <gz/gui/Plugin.hh>

#include <gz/rendering/Camera.hh>

namespace gz
{
namespace sim
{
  class EntityContextMenuPrivate;

  /// \brief This plugin is in charge of showing the entity context menu when
  /// the right button is clicked on a visual.
  class EntityContextMenu : public gz::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: EntityContextMenu();

    /// \brief Destructor
    public: ~EntityContextMenu() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<EntityContextMenuPrivate> dataPtr;
  };

  class EntityContextMenuHandler : public QObject
  {
    Q_OBJECT

    ///  \brief Constructor
    public: EntityContextMenuHandler();

    /// \brief Handle mouse event for context menu
    /// \param[in] _mouseEvent Right click mouse event
    /// \param[in] _camera User camera
    public: void HandleMouseContextMenu(const common::MouseEvent &_mouseEvent,
      const rendering::CameraPtr &_camera);

    /// \brief Signal fired when context menu event is triggered
    /// \param[in] _entity Scoped name of entity.
    /// \param[in] _mouseX X coordinate of the right click
    /// \param[in] _mouseY Y coordinate of the right click
    signals: void ContextMenuRequested(
      QString _entity, int _mouseX, int _mouseY);
  };

  /// \brief A QQUickItem that manages the render window
  class EntityContextMenuItem : public QQuickItem
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _parent Parent item
    public: explicit EntityContextMenuItem(QQuickItem *_parent = nullptr);

    /// \brief Set the entity context menu handler
    /// \param[in] _entityContextMenuHandler Entity context menu handler
    public: void SetEntityContextMenuHandler(
      const EntityContextMenuHandler &_entityContextMenuHandler);

    /// \brief Signal fired to open context menu
    /// Note that the function name needs to start with lowercase in order for
    /// the connection to work on the QML side
    /// \param[in] _entity Scoped name of entity.
    /// \param[in] _mouseX X coordinate of the right click
    /// \param[in] _mouseY Y coordinate of the right click
    signals: void openContextMenu(QString _entity, int _mouseX, int _mouseY); // NOLINT

    /// \brief Qt callback when context menu request is received
    /// \param[in] _entity Scoped name of entity.
    /// \param[in] _mouseX X coordinate of the right click
    /// \param[in] _mouseY Y coordinate of the right click
    public slots: void OnContextMenuRequested(
      QString _entity, int _mouseX, int _mouseY);
  };
}
}

#endif

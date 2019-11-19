/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_GUI_ENTITYCONTEXTMENU_HH_
#define IGNITION_GAZEBO_GUI_ENTITYCONTEXTMENU_HH_

#include <ignition/gui/qt.h>
#include <QtQml/QQmlExtensionPlugin>
#include <memory>

namespace ignition
{
namespace gazebo
{
  class EntityContextMenuPrivate;

  /// \brief IgnGazebo QML Plugin that registers C++ class so that they are
  /// accessible from QML.
  class IgnGazeboPlugin : public QQmlExtensionPlugin
  {
    Q_OBJECT

    // unique id
    Q_PLUGIN_METADATA(IID "IgnGazebo/1.0")

    /// \brief Overrided function that registers C++ class as a QML type
    /// \param[in] _uri Plugin uri.
    public: void registerTypes(const char *_uri) override;
  };

  /// \brief A context menu providing actions that can be invoked on an entity
  class EntityContextMenu : public QQuickItem
  {
    Q_OBJECT

    /// \brief Constructor
    public: EntityContextMenu();

    /// \brief Destructor
    public: ~EntityContextMenu() override;

    /// \brief Callback when a context menu item is invoked
    /// \param[in] _request Request type
    /// \param[in] _data Request data
    /// \param[in] _type Request type
    public: Q_INVOKABLE void OnRemove(const QString &_request,
        const QString &_data, const QString &_type);

    /// \brief Callback when a context menu item is invoked
    /// \param[in] _request Request type
    /// \param[in] _data Request data
    public: Q_INVOKABLE void OnRequest(const QString &_request,
        const QString &_data);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<EntityContextMenuPrivate> dataPtr;
  };
}
}

#endif

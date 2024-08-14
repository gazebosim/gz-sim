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

#ifndef GZ_SIM_GUI_ENTITYCONTEXTMENU_HH_
#define GZ_SIM_GUI_ENTITYCONTEXTMENU_HH_

#include <gz/gui/qt.h>
#include <gz/msgs/cameratrack.pb.h>
#include <QtQml/QQmlExtensionPlugin>
#include <memory>

namespace gz
{
namespace sim
{
  class EntityContextMenuPrivate;

  /// \brief GzSim QML Plugin that registers C++ class so that they are
  /// accessible from QML.
  class GzSimPlugin : public QQmlExtensionPlugin
  {
    Q_OBJECT

    // unique id
    Q_PLUGIN_METADATA(IID "GzSim/1.0")

    /// \brief Overrided function that registers C++ class as a QML type
    /// \param[in] _uri Plugin uri.
    public: void registerTypes(const char *_uri) override;
  };

  /// \brief A context menu providing actions that can be invoked on an entity
  class EntityContextMenu : public QQuickItem
  {
    Q_OBJECT
    /// \brief followingTarget
    Q_PROPERTY(
      bool followingTarget
      READ FollowingTarget
      WRITE SetFollowingTarget
      NOTIFY FollowingTargetChanged
    )

    /// \brief Constructor
    public: EntityContextMenu();

    /// \brief Destructor
    public: ~EntityContextMenu() override;

    /// \brief Get whether it is following target
    /// \return True if followingTarget
    public: Q_INVOKABLE bool FollowingTarget() const;

    /// \brief Set whether followingTarget
    /// \param[in] _followingTarget True if followingTarget
    public: Q_INVOKABLE void SetFollowingTarget(bool &_followingTarget);

    /// \brief Notify that followingTarget has changed
    signals: void FollowingTargetChanged();

    /// \brief Callback function to get data from the message
    /// \param[in] _msg CameraTrack message
    public: void OnCurrentlyTrackedSub(const msgs::CameraTrack &_msg);

    /// \brief Callback when a context menu item is invoked
    /// \param[in] _data Request data
    /// \param[in] _type Entity type
    public: Q_INVOKABLE void OnRemove(
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

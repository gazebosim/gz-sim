/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_GUI_COMPONENTINSPECTOR_POSE3D_HH_
#define IGNITION_GAZEBO_GUI_COMPONENTINSPECTOR_POSE3D_HH_

#include <ignition/math/Pose3.hh>
#include "ignition/gazebo/EntityComponentManager.hh"

#include <QObject>
#include <QStandardItem>

namespace ignition
{
namespace gazebo
{
class ComponentInspector;
namespace inspector
{
  /// \brief Handles components that are displayed as a 3D pose:
  /// * `components::Pose`
  /// * `components::WorldPose`
  /// * `components::WorldPoseCmd`
  class Pose3d : public QObject
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _inspector The component inspector.
    public: explicit Pose3d(ComponentInspector *_inspector);

    /// \brief Callback when there are ECM updates.
    /// \param[in] _ecm Immutable reference to the ECM.
    /// \param[in] _item Item to update.
    public: void UpdateView(const EntityComponentManager &_ecm,
        QStandardItem *_item);

    /// \brief Callback in Qt thread when pose changes.
    /// \param[in] _x X
    /// \param[in] _y Y
    /// \param[in] _z Z
    /// \param[in] _roll Roll
    /// \param[in] _pitch Pitch
    /// \param[in] _yaw Yaw
    public: Q_INVOKABLE void OnPose(double _x, double _y, double _z,
        double _roll, double _pitch, double _yaw);

    /// \brief Pointer to the component inspector. This is used to add
    /// callbacks.
    private: ComponentInspector *inspector{nullptr};
  };
}
}
}
#endif

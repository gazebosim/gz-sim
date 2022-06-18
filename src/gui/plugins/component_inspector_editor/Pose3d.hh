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
#ifndef GZ_SIM_GUI_COMPONENTINSPECTOR_POSE3D_HH_
#define GZ_SIM_GUI_COMPONENTINSPECTOR_POSE3D_HH_

#include <QObject>
#include <gz/math/Pose3.hh>

namespace gz
{
namespace sim
{
  class ComponentInspectorEditor;

  /// \brief A class that handles Pose3d changes.
  class Pose3d : public QObject
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _inspector The component inspector.
    public: explicit Pose3d(ComponentInspectorEditor *_inspector);

    /// \brief Handle pose updates from the GUI.
    /// \param[in] _x New x value.
    /// \param[in] _y New y value.
    /// \param[in] _z New z value.
    /// \param[in] _roll New roll value.
    /// \param[in] _pitch New pitch value.
    /// \param[in] _yaw New yaw value.
    public: Q_INVOKABLE void PoseUpdate(
                double _x, double _y, double _z, double _roll,
                double _pitch, double _yaw);

    /// \brief Notify that simulation paused state has changed.
    signals: void poseChanged();

    /// \brief Pointer to the component inspector. This is used to add
    /// update callbacks that modify the ECM.
    private: ComponentInspectorEditor *inspector{nullptr};

    /// \brief Keep track of current pose so that we can update the pose
    /// spin boxes correctly.
    private: math::Pose3d currentPose;
  };
}
}
#endif

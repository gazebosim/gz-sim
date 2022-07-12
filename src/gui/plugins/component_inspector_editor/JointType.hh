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
#ifndef GZ_SIM_GUI_COMPONENTINSPECTOR_JOINT_HH_
#define GZ_SIM_GUI_COMPONENTINSPECTOR_JOINT_HH_

#include <QObject>

namespace gz
{
namespace sim
{
  class ComponentInspectorEditor;

  /// \brief A class that handles joint changes.
  class JointType : public QObject
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _inspector The component inspector.
    public: explicit JointType(ComponentInspectorEditor *_inspector);

    /// \brief Callback in Qt thread when joint type changes.
    /// \param[in] _surface Surface model
    public: Q_INVOKABLE void OnJointType(QString _jointType);

    /// \brief Pointer to the component inspector. This is used to add
    /// update callbacks that modify the ECM.
    private: ComponentInspectorEditor *inspector{nullptr};
  };
}
}
#endif

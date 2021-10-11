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

#ifndef IGNITION_GAZEBO_GUI_MODELEDITOR_HH_
#define IGNITION_GAZEBO_GUI_MODELEDITOR_HH_

#include <map>
#include <memory>
#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
namespace gazebo
{
  class ModelEditorPrivate;

  /// \brief
  ///
  /// ## Configuration
  /// None
  class ModelEditor : public gazebo::GuiSystem
  {
    Q_OBJECT

    public: ModelEditor();
    public: ~ModelEditor() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ModelEditorPrivate> dataPtr;
  };
}
}

#endif

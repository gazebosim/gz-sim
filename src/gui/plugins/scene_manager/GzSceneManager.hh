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

#ifndef IGNITION_GAZEBO_GUI_GZSCENEMANAGER_HH_
#define IGNITION_GAZEBO_GUI_GZSCENEMANAGER_HH_

#include <memory>

#include <ignition/gazebo/gui/GuiSystem.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  class GzSceneManagerPrivate;

  /// \brief Updates a 3D scene based on information coming from the ECM.
  /// This plugin doesn't instantiate a new 3D scene. Instead, it relies on
  /// another plugin being loaded alongside it that will create and paint the
  /// scene to the window, such as `ignition::gui::plugins::Scene3D`.
  class GzSceneManager : public GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: GzSceneManager();

    /// \brief Destructor
    public: ~GzSceneManager() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem)
        override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;

    // Documentation inherited
    private: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<GzSceneManagerPrivate> dataPtr;
  };
}
}
}

#endif

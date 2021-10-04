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

#ifndef IGNITION_GAZEBO_GUI_VISUALIZATIONCAPABILITIES_HH_
#define IGNITION_GAZEBO_GUI_VISUALIZATIONCAPABILITIES_HH_

#include <memory>

#include <ignition/gazebo/gui/GuiSystem.hh>

namespace ignition
{
namespace gazebo
{
  class VisualizationCapabilitiesPrivate;

  /// \brief Allows to visualize transparent, collisions, inertial,
  /// CoM and more.
  class VisualizationCapabilities : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: VisualizationCapabilities();

    /// \brief Destructor
    public: ~VisualizationCapabilities() override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<VisualizationCapabilitiesPrivate> dataPtr;
  };
}
}

#endif

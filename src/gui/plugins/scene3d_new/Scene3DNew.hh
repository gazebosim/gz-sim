/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GUI_PLUGINS_SCENE3DNEW_HH_
#define IGNITION_GUI_PLUGINS_SCENE3DNEW_HH_

#include <ignition/gui/Plugin.hh>
#include <ignition/gui/Scene3DInterface.hh>
#include <ignition/utilities/SuppressWarning.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>

#include <memory>

namespace ignition
{
namespace gui
{
namespace plugins
{
class Scene3DNewPrivate;
/// \brief Plots fields from Ignition Transport topics.
/// Fields can be dragged from the Topic Viewer or the Component Inspector.
class Scene3DNew : public ignition::gazebo::GuiSystem
{
  Q_OBJECT

  /// \brief Constructor
  public: Scene3DNew();

  /// \brief Destructor
  public: ~Scene3DNew();

  // Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *) override;

  // Documentation inherited
  public: void Update(const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override;

  IGN_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
  private: std::unique_ptr<Scene3DInterface> dataPtr;
  private: std::unique_ptr<Scene3DNewPrivate> dataInternalPtr;
  IGN_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
};

}
}
}
#endif

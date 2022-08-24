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

#ifndef RENDERING_GUI_PLUGIN_HH_
#define RENDERING_GUI_PLUGIN_HH_

#include <gz/gui/qt.h>
#include <gz/gui/Plugin.hh>
#include <gz/rendering/Scene.hh>

/// \brief Example of a GUI plugin that uses Gazebo Rendering.
/// This plugin works with Gazebo GUI's MinimalScene or any plugin providing
/// similar functionality.
class RenderingGuiPlugin : public gz::gui::Plugin
{
  Q_OBJECT

  ///\brief Called once at startup.
  public: void LoadConfig(const tinyxml2::XMLElement *) override;

  /// \brief Callback when user clicks button.
  public slots: void RandomColor();

  /// \brief Callback for all installed event filters.
  /// \param[in] _obj Object that received the event
  /// \param[in] _event Event
  private: bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \brief All rendering operations must happen within this call
  private: void PerformRenderingOperations();

  /// \brief Encapsulates the logic to find the rendering scene through the
  /// render engine singleton.
  private: void FindScene();

  /// \brief Marks when a new change has been requested.
  private: bool dirty{false};

  /// \brief Pointer to the rendering scene.
  private: gz::rendering::ScenePtr scene{nullptr};
};

#endif

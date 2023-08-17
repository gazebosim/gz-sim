/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef GZ_GUI_MOUSEDRAG_HH
#define GZ_GUI_MOUSEDRAG_HH

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>

namespace gz
{
namespace sim
{
  class MouseDragPrivate;

  /// \brief Translate and rotate links by dragging them with the mouse.
  /// Automatically loads the ApplyLinkWrench system.
  ///
  /// ## Configuration
  /// This plugin doesn't accept any custom configuration.
  class MouseDrag : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: MouseDrag();

    /// \brief Destructor
    public: ~MouseDrag() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    /// \brief Callback when COM switch is pressed
    /// \param[in] _checked True if force should be applied to center of mass
    public slots: void OnSwitchCOM(const bool _checked);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<MouseDragPrivate> dataPtr;
  };
}
}

#endif

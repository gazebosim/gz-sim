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

#ifndef IGNITION_GAZEBO_GUI_GRIDGUI_HH_
#define IGNITION_GAZEBO_GUI_GRIDGUI_HH_

#include <memory>

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

namespace ignition
{
namespace gui
{
namespace plugins
{
  class GridGUIPrivate;

  class GridGUI : public Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: GridGUI();

    /// \brief Destructor
    public: ~GridGUI() override;

    // Documentation inherited
    public: void LoadConfig() override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<GridGUIPrivate> dataPtr;
  };
}
}
}

#endif

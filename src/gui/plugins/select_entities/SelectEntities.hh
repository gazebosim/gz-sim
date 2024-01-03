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

#ifndef GZ_SIM_GUI_SELECTENTITIES_HH_
#define GZ_SIM_GUI_SELECTENTITIES_HH_

#include <memory>

#include <gz/gui/Plugin.hh>

namespace gz
{
namespace sim
{
namespace gui
{
  class SelectEntitiesPrivate;

  /// \brief This plugin is in charge of selecting and deselecting the entities
  /// from the 3D scene and emit the corresponding events.
  class SelectEntities : public gz::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: SelectEntities();

    /// \brief Destructor
    public: virtual ~SelectEntities();

    // Documentation inherited
    public: virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem)
        override;

    // Documentation inherited
    private: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<SelectEntitiesPrivate> dataPtr;
  };
}
}
}
#endif

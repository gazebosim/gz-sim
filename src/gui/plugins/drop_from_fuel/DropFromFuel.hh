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

#ifndef IGNITION_GAZEBO_GUI_DROP_FROM_FUEL_HH_
#define IGNITION_GAZEBO_GUI_DROP_FROM_FUEL_HH_

#include <memory>

#include <ignition/gui/Plugin.hh>

namespace ignition
{
namespace gazebo
{
  class DropFromFuelPrivate;

  /// \brief Allows to DropFromFuel models and lights using te gui event
  /// DropFromFuelFromDescription
  class DropFromFuel : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: DropFromFuel();

    /// \brief Destructor
    public: ~DropFromFuel() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<DropFromFuelPrivate> dataPtr;
  };
}
}

#endif

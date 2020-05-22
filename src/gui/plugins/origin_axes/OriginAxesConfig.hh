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

#ifndef IGNITION_GAZEBO_GUI_ORIGINAXESCONFIG_HH_
#define IGNITION_GAZEBO_GUI_ORIGINAXESCONFIG_HH_

#include <memory>

#include <ignition/gui/Plugin.hh>
#include <ignition/rendering.hh>

namespace ignition
{
namespace gazebo
{
  class OriginAxesConfigPrivate;

  class OriginAxesConfig : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: OriginAxesConfig();

    /// \brief Destructor
    public: ~OriginAxesConfig() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Update arrows
    public: void UpdateOriginArrows();

    /// \brief Callback to create a new one.
    public: void LoadOriginAxes();

    /// \brief Callback to update the size of the arrows
    /// \param[in] _size size of the arrows in meters
    public slots: void UpdateLength(double _length);

    /// \brief Callback when checkbox is clicked.
    /// \param[in] _checked indicates show or hide arrows
    public slots: void OnShow(bool _checked);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<OriginAxesConfigPrivate> dataPtr;
  };
}
}

#endif

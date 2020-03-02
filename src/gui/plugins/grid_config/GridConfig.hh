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

#ifndef IGNITION_GAZEBO_GUI_GRIDCONFIG_HH_
#define IGNITION_GAZEBO_GUI_GRIDCONFIG_HH_

#include <memory>

#include <ignition/gui/Plugin.hh>
#include <ignition/rendering.hh>

namespace ignition
{
namespace gazebo
{
  class GridConfigPrivate;

  class GridConfig : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: GridConfig();

    /// \brief Destructor
    public: ~GridConfig() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Update grid
    public: void UpdateGrid();

    /// \brief Callback to retrieve existing grid or create a new one.
    /// \param[in] _scene Scene to look for grid.
    public: void LoadGrid();

    /// \brief Callback to update vertical cell count
    /// \param[in] _cellCount new vertical cell count
    public slots: void UpdateVCellCount(int _cellCount);

    /// \brief Callback to update horizontal cell count
    /// \param[in] _cellCount new horizontal cell count
    public slots: void UpdateHCellCount(int _cellCount);

    /// \brief Callback to update cell length
    /// \param[in] _length new cell length
    public slots: void UpdateCellLength(double _length);

    /// \brief Callback to update grid pose
    /// \param[in] _x, _y, _z cartesion coordinates
    /// \param[in] _roll, _pitch, _yaw principal coordinates
    public slots: void SetPose(double _x, double _y, double _z,
                               double _roll, double _pitch, double _yaw);

    /// \brief Callback to update grid color
    /// \param[in] _r, _g, _b, _a RGB color model with fourth alpha channel
    public slots: void SetColor(double _r, double _g, double _b, double _a);

    /// \brief Callback when checkbox is clicked.
    /// \param[in] _checked indicates show or hide grid
    public slots: void OnShow(bool _checked);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<GridConfigPrivate> dataPtr;
  };
}
}

#endif

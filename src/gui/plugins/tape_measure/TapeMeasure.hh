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

#ifndef IGNITION_GAZEBO_GUI_TAPEMEASURE_HH_
#define IGNITION_GAZEBO_GUI_TAPEMEASURE_HH_

#include <memory>

#include <ignition/gui/Plugin.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>

namespace ignition
{
namespace gazebo
{
  class TapeMeasurePrivate;

  /// \brief Provides buttons for translation, rotation, and scale
  ///
  /// ## Configuration
  /// \<service\> : Set the service to receive transform mode requests.
  class TapeMeasure : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: TapeMeasure();

    /// \brief Destructor
    public: ~TapeMeasure() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Deletes the marker with the provided id within the
    /// "tape_measure" namespace.
    /// \param[in] _id The id of the marker
    public: void DeleteMarker(int _id);

    /// \brief Resets all of the relevant data for this plugin.  Called when
    /// the user clicks the reset button and when the user starts a new
    /// measurement.
    public: void Reset();

    /// \brief Draws a point marker.  Called to display the start and end
    /// point of the tape measure.
    /// \param[in] _id The id of the marker
    /// \param[in] _point The x, y, z coordinates of where to place the marker
    /// \param[in] _color The rgba color to set the marker
    public: void DrawPoint(int _id,
                ignition::math::Vector3d &_point,
                ignition::math::Vector4d &_color);

    /// \brief Draws a line marker.  Called to display the line between the
    /// start and end point of the tape measure.
    /// \param[in] _id The id of the marker
    /// \param[in] _startPoint The x, y, z coordinates of the line start point
    /// \param[in] _endPoint The x, y, z coordinates of the line end point
    /// \param[in] _color The rgba color to set the marker
    public: void DrawLine(int _id,
                ignition::math::Vector3d &_startPoint,
                ignition::math::Vector3d &_endPoint,
                ignition::math::Vector4d &_color);

    /// \brief Callback in Qt thread when the new measurement button is
    /// clicked.
    public slots: void OnMeasure();

    /// \brief Callback in Qt thread when the reset button is clicked.
    public slots: void OnReset();

    /// \brief Callback in Qt thread to get the distance to display in the
    /// gui window.
    /// \return The distance between the start and end point of the measurement
    public slots: double Distance();

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Signal fired when a new tape measure distance is set.
    signals: void newDistance();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<TapeMeasurePrivate> dataPtr;
  };
}
}

#endif

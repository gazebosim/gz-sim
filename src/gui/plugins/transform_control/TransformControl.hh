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

#ifndef GZ_SIM_GUI_TRANSFORMCONTROL_HH_
#define GZ_SIM_GUI_TRANSFORMCONTROL_HH_

#include <memory>

#include <gz/gui/Plugin.hh>

namespace gz
{
namespace sim
{
  class TransformControlPrivate;

  /// \brief Provides buttons for translation, rotation, and scale
  ///
  /// ## Configuration
  /// \<service\> : Set the service to receive transform mode requests.
  class TransformControl : public gz::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: TransformControl();

    /// \brief Destructor
    public: ~TransformControl() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Callback to retrieve existing grid. Should only be called
    /// within the render thread.  If no grid is found, the grid pointer
    /// is not updated.
    public: void LoadGrid();

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Sends an event to update the internal snapping values for
    /// translation, rotation, and scaling.
    /// \param[in] _x The snapping distance along the world's x (red) axis
    /// expressed in meters.
    /// \param[in] _y The snapping distance along the world's y (green) axis
    /// expressed in meters.
    /// \param[in] _z The snapping distance along the world's z (blue) axis
    /// expressed in meters.
    /// \param[in] _roll The snapping distance along the entity's roll (red)
    /// axis expressed in degrees.
    /// \param[in] _pitch The snapping distance along the entity's pitch
    /// (green) axis expressed in degrees.
    /// \param[in] _yaw The snapping distance along the entity's yaw (blue)
    /// axis expressed in degrees.
    /// \param[in] _scaleX The snapping scale along the entity's x (red) axis
    /// \param[in] _scaleY The snapping scale along the entity's y (green) axis
    /// \param[in] _scaleZ The snapping scale along the entity's z (blue) axis
    public slots: void OnSnapUpdate(
        double _x, double _y, double _z,
        double _roll, double _pitch, double _yaw,
        double _scaleX, double _scaleY, double _scaleZ);

    /// \brief Callback in Qt thread when mode changes.
    /// \param[in] _mode New transform mode
    public slots: void OnMode(const QString &_mode);

    /// \brief Sets the custom snap values to the grid values.
    public: void SnapToGrid();

    /// \brief Callback in Qt thread when the snap to grid button is clicked.
    public slots: void OnSnapToGrid();

    /// \brief Callback in Qt thread when the snap to grid button is clicked
    /// in order to update custom snap values menu.
    /// \return The set x translational snapping value
    public slots: double xSnap();

    /// \brief Callback in Qt thread when the snap to grid button is clicked
    /// in order to update custom snap values menu.
    /// \return The set y translational snapping value
    public slots: double ySnap();

    /// \brief Callback in Qt thread when the snap to grid button is clicked
    /// in order to update custom snap values menu.
    /// \return The set z translational snapping value
    public slots: double zSnap();

    /// \brief Callback in Qt thread when the snap to grid button is clicked
    /// in order to update custom snap values menu.
    /// \return The set roll rotational snapping value
    public slots: double rollSnap();

    /// \brief Callback in Qt thread when the snap to grid button is clicked
    /// in order to update custom snap values menu.
    /// \return The set pitch rotational snapping value
    public slots: double pitchSnap();

    /// \brief Callback in Qt thread when the snap to grid button is clicked
    /// in order to update custom snap values menu.
    /// \return The set yaw rotational snapping value
    public slots: double yawSnap();

    /// \brief Callback in Qt thread when the snap to grid button is clicked
    /// in order to update custom snap values menu.
    /// \return The set x scaling snapping value
    public slots: double scaleXSnap();

    /// \brief Callback in Qt thread when the snap to grid button is clicked
    /// in order to update custom snap values menu.
    /// \return The set y scaling snapping value
    public slots: double scaleYSnap();

    /// \brief Callback in Qt thread when the snap to grid button is clicked
    /// in order to update custom snap values menu.
    /// \return The set z scaling snapping value
    public slots: double scaleZSnap();

    /// \brief Notify that new snapping values have been set.
    signals: void newSnapValues();

    /// \brief Notify that selection has been activated
    signals: void activateSelect();

    /// \brief Notify that translation has been activated
    signals: void activateTranslate();

    /// \brief Notify that rotation has been activated
    signals: void activateRotate();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<TransformControlPrivate> dataPtr;
  };
}
}

#endif

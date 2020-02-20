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

#ifndef IGNITION_GAZEBO_GUI_TRANSFORMCONTROL_HH_
#define IGNITION_GAZEBO_GUI_TRANSFORMCONTROL_HH_

#include <memory>

#include <ignition/gui/Plugin.hh>

namespace ignition
{
namespace gazebo
{
  class TransformControlPrivate;

  /// \brief Provides buttons for translation, rotation, and scale
  ///
  /// ## Configuration
  /// \<service\> : Set the service to receive transform mode requests.
  class TransformControl : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: TransformControl();

    /// \brief Destructor
    public: ~TransformControl() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

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

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<TransformControlPrivate> dataPtr;
  };
}
}

#endif

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

#ifndef IGNITION_GAZEBO_GUI_VIEWANGLE_HH_
#define IGNITION_GAZEBO_GUI_VIEWANGLE_HH_

#include <memory>

#include <ignition/gui/Plugin.hh>

namespace ignition
{
namespace gazebo
{
  class ViewAnglePrivate;

  /// \brief Provides buttons for viewing angles
  ///
  /// ## Configuration
  /// \<service\> : Set the service to receive view angle requests.
  class ViewAngle : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: ViewAngle();

    /// \brief Destructor
    public: ~ViewAngle() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Callback in Qt thread when angle mode changes.
    /// \param[in] _x The x component of the directional vector for the camera
    /// to assume.  All 0s for x, y, and z indicate the initial camera pose.
    /// \param[in] _y The y component of the directional vector for the camera
    /// to assume.  All 0s for x, y, and z indicate the initial camera pose.
    /// \param[in] _z The z component of the directional vector for the camera
    /// to assume.  All 0s for x, y, and z indicate the initial camera pose.
    public slots: void OnAngleMode(int _x, int _y, int _z);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ViewAnglePrivate> dataPtr;
  };
}
}

#endif

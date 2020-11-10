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

    public slots: void OnMeasure();
    public slots: void OnReset();
    public slots: double Distance();
    public: void Reset();

    public: void DrawPoint(int id, ignition::math::Vector3d &_point, ignition::math::Vector4d &_color);
    public: void DrawLine(int id, ignition::math::Vector3d &_startPoint, ignition::math::Vector3d &_endPoint, ignition::math::Vector4d &_color);

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    signals: void newDistance();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<TapeMeasurePrivate> dataPtr;
  };
}
}

#endif

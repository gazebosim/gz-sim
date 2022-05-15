/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_GUI_GLOBALILLUMINATIONVCT_HH_
#define IGNITION_GAZEBO_GUI_GLOBALILLUMINATIONVCT_HH_

#include <memory>

#include "ignition/msgs/laserscan.pb.h"
#include "ignition/gazebo/gui/GuiSystem.hh"
#include "ignition/gui/qt.h"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
  class GlobalIlluminationVctPrivate;

  /// \brief Visualize the LaserScan message returned by the sensors. Use the
  /// checkbox to turn visualization of non-hitting rays on or off and
  /// the textfield to select the message to be visualised. The combobox is
  /// used to select the type of visual for the sensor data.
  class GlobalIlluminationVct : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Enabled QML binding.
    /// Anything that isn't GUI-only (i.e. affects simulation) needs it
    Q_PROPERTY(
      bool enabled
      READ Enabled
      WRITE SetEnabled
      NOTIFY EnabledChanged
    )

    /// \brief Constructor
    public: GlobalIlluminationVct();

    /// \brief Destructor
    public: ~GlobalIlluminationVct() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation Inherited
    public: bool eventFilter(QObject *_obj, QEvent *_event) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    /// \brief Callback function to get data from the message
    /// \param[in]_msg LidarSensor message
    public: void OnScan(const msgs::LaserScan &_msg);

    /// \brief Load the scene and attach LidarVisual to the scene
    public: void LoadGlobalIlluminationVct();

    /// \brief Set debug visualization mode GlogbalIllumination
    /// \param[in] _mode Index of selected debug visualization mode
    public: Q_INVOKABLE void UpdateDebugVisualizationMode(int _mode);

    /// \brief Set VCT resolution
    /// \param[in] _axis Axis (width, height, depth). In range [0; 3)
    /// \param[in] _res New resolution
    public: Q_INVOKABLE void UpdateResolution(int _axis, uint32_t _res);

    /// \brief Set topic to subscribe for LidarSensor data
    /// \param[in] _topicName Name of selected topic
    public: Q_INVOKABLE void OnTopic(const QString &_topicName);

    /// \brief See rendering::GlobalIlluminationVct::SetEnabled &
    /// rendering::Scene::SetActiveGlobalIllumination
    /// \param[in] _enabled See GlobalIlluminationVct::SetEnabled
    public: Q_INVOKABLE void SetEnabled(const bool _enabled);

    /// \brief See rendering::GlobalIlluminationVct::Enabled
    /// \return See rendering::GlobalIlluminationVct::Enabled
    public: Q_INVOKABLE bool Enabled() const;

    /// \brief Notify this property has changed
    signals: void EnabledChanged();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<GlobalIlluminationVctPrivate> dataPtr;
  };
}
}
}
#endif

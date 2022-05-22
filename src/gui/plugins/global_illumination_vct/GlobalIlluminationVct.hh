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

    Q_PROPERTY(
      int resolutionX
      READ ResolutionX
      WRITE SetResolutionX
      NOTIFY SettingsChanged
    )
    Q_PROPERTY(
      int resolutionY
      READ ResolutionY
      WRITE SetResolutionY
      NOTIFY SettingsChanged
    )
    Q_PROPERTY(
      int resolutionZ
      READ ResolutionZ
      WRITE SetResolutionZ
      NOTIFY SettingsChanged
    )

    Q_PROPERTY(
      int octantCountX
      READ OctantCountX
      WRITE SetOctantCountX
      NOTIFY SettingsChanged
    )
    Q_PROPERTY(
      int octantCountY
      READ OctantCountY
      WRITE SetOctantCountY
      NOTIFY SettingsChanged
    )
    Q_PROPERTY(
      int octantCountZ
      READ OctantCountZ
      WRITE SetOctantCountZ
      NOTIFY SettingsChanged
    )

    Q_PROPERTY(
      int bounceCount
      READ BounceCount
      WRITE SetBounceCount
      NOTIFY LightingChanged
    )

    Q_PROPERTY(
      bool highQuality
      READ HighQuality
      WRITE SetHighQuality
      NOTIFY LightingChanged
    )

    Q_PROPERTY(
      bool anisotropic
      READ Anisotropic
      WRITE SetAnisotropic
      NOTIFY LightingChanged
    )

    Q_PROPERTY(
      int debugVisualizationMode
      READ DebugVisualizationMode
      WRITE SetDebugVisualizationMode
      NOTIFY DebugVisualizationModeChanged
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

    /// \brief Set VCT octant count
    /// \param[in] _axis Axis (width, height, depth). In range [0; 3)
    /// \param[in] _res New octant count
    public: Q_INVOKABLE void UpdateOctantCount(int _axis, uint32_t _count);

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

    /// \brief Notify various properties may have changed
    signals: void SettingsChanged();

    /// \brief Notify fast-to-rebuild properties may have changed
    signals: void LightingChanged();

    /// \brief Notify debug visualization has changed
    signals: void DebugVisualizationModeChanged();

    /// \brief See rendering::GlobalIlluminationVct::SetResolution
    /// \param[in] _enabled See GlobalIlluminationVct::SetResolution
    public: Q_INVOKABLE void SetResolutionX(const uint32_t _res);

    /// \brief See rendering::GlobalIlluminationVct::Resolution
    /// \return See rendering::GlobalIlluminationVct::Resolution
    public: Q_INVOKABLE uint32_t ResolutionX() const;

    /// \brief See rendering::GlobalIlluminationVct::SetResolution
    /// \param[in] _enabled See GlobalIlluminationVct::SetResolution
    public: Q_INVOKABLE void SetResolutionY(const uint32_t _res);

    /// \brief See rendering::GlobalIlluminationVct::Resolution
    /// \return See rendering::GlobalIlluminationVct::Resolution
    public: Q_INVOKABLE uint32_t ResolutionY() const;

    /// \brief See rendering::GlobalIlluminationVct::SetResolution
    /// \param[in] _enabled See GlobalIlluminationVct::SetResolution
    public: Q_INVOKABLE void SetResolutionZ(const uint32_t _res);

    /// \brief See rendering::GlobalIlluminationVct::Resolution
    /// \return See rendering::GlobalIlluminationVct::Resolution
    public: Q_INVOKABLE uint32_t ResolutionZ() const;

    /// \brief See rendering::GlobalIlluminationVct::SetOctantCount
    /// \param[in] _enabled See GlobalIlluminationVct::SetOctantCount
    public: Q_INVOKABLE void SetOctantCountX(const uint32_t _res);

    /// \brief See rendering::GlobalIlluminationVct::OctantCount
    /// \return See rendering::GlobalIlluminationVct::OctantCount
    public: Q_INVOKABLE uint32_t OctantCountX() const;

    /// \brief See rendering::GlobalIlluminationVct::SetOctantCount
    /// \param[in] _enabled See GlobalIlluminationVct::SetOctantCount
    public: Q_INVOKABLE void SetOctantCountY(const uint32_t _res);

    /// \brief See rendering::GlobalIlluminationVct::OctantCount
    /// \return See rendering::GlobalIlluminationVct::OctantCount
    public: Q_INVOKABLE uint32_t OctantCountY() const;

    /// \brief See rendering::GlobalIlluminationVct::SetOctantCount
    /// \param[in] _enabled See GlobalIlluminationVct::SetOctantCount
    public: Q_INVOKABLE void SetOctantCountZ(const uint32_t _res);

    /// \brief See rendering::GlobalIlluminationVct::OctantCount
    /// \return See rendering::GlobalIlluminationVct::OctantCount
    public: Q_INVOKABLE uint32_t OctantCountZ() const;

    /// \brief See rendering::GlobalIlluminationVct::SetBounceCount
    /// \param[in] _enabled See GlobalIlluminationVct::SetBounceCount
    public: Q_INVOKABLE void SetBounceCount(const uint32_t _bounces);

    /// \brief See rendering::GlobalIlluminationVct::BounceCount
    /// \return See rendering::GlobalIlluminationVct::BounceCount
    public: Q_INVOKABLE uint32_t BounceCount() const;

    /// \brief See rendering::GlobalIlluminationVct::SetHighQuality
    /// \param[in] _enabled See GlobalIlluminationVct::SetHighQuality
    public: Q_INVOKABLE void SetHighQuality(const bool _quality);

    /// \brief See rendering::GlobalIlluminationVct::HighQuality
    /// \return See rendering::GlobalIlluminationVct::HighQuality
    public: Q_INVOKABLE bool HighQuality() const;

    /// \brief See rendering::GlobalIlluminationVct::SetAnisotropic
    /// \param[in] _enabled See GlobalIlluminationVct::SetAnisotropic
    public: Q_INVOKABLE void SetAnisotropic(const bool _anisotropic);

    /// \brief See rendering::GlobalIlluminationVct::Anisotropic
    /// \return See rendering::GlobalIlluminationVct::Anisotropic
    public: Q_INVOKABLE bool Anisotropic() const;

    /// \brief See rendering::GlobalIlluminationVct::SetDebugVisualizationMode
    /// \param[in] _enabled See GlobalIlluminationVct::SetDebugVisualizationMode
    public: Q_INVOKABLE void SetDebugVisualizationMode(const uint32_t _visMode);

    /// \brief See rendering::GlobalIlluminationVct::DebugVisualizationMode
    /// \return See rendering::GlobalIlluminationVct::DebugVisualizationMode
    public: Q_INVOKABLE uint32_t DebugVisualizationMode() const;

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<GlobalIlluminationVctPrivate> dataPtr;
  };
}
}
}
#endif

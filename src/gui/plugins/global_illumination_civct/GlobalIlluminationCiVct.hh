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

#ifndef GZ_SIM_GUI_GLOBALILLUMINATIONCIVCT_HH_
#define GZ_SIM_GUI_GLOBALILLUMINATIONCIVCT_HH_

#include <memory>

#include "gz/sim/gui/GuiSystem.hh"
#include "gz/gui/qt.h"

#include <QtQml/QQmlExtensionPlugin>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  class GlobalIlluminationCiVctPrivate;
  class CiVctCascadePrivate;

  /// \brief Enable and configure Global Illumination using CIVCT
  /// (Cascaded Image Voxel Cone Tracing)
  class GlobalIlluminationCiVct : public gz::sim::GuiSystem
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
      bool cascadesEditable
      READ CascadesEditable
      NOTIFY CascadesEditableChanged
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

    Q_PROPERTY(
      QStringList cameraList
      READ CameraList
      NOTIFY CameraListChanged
    )

    /// \brief Constructor
    public: GlobalIlluminationCiVct();

    /// \brief Destructor
    public: ~GlobalIlluminationCiVct() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation Inherited
    public: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Load the scene and attach LidarVisual to the scene
    public: void LoadGlobalIlluminationCiVct();

    /// \brief Set debug visualization mode GlogbalIllumination
    /// \param[in] _mode Index of selected debug visualization mode
    public: Q_INVOKABLE void UpdateDebugVisualizationMode(int _mode);

    /// \brief See rendering::GlobalIlluminationCiVct::SetEnabled &
    /// rendering::Scene::SetActiveGlobalIllumination
    /// \param[in] _enabled See GlobalIlluminationCiVct::SetEnabled
    /// \return The new setting. We may fail to enable if settings are invalid
    /// See ValidSettings()
    public: Q_INVOKABLE bool SetEnabled(const bool _enabled);

    /// \brief See rendering::GlobalIlluminationCiVct::Enabled
    /// \return See rendering::GlobalIlluminationCiVct::Enabled
    public: Q_INVOKABLE bool Enabled() const;

    /// \brief Returns true when it's possible to add/remove cascades
    /// False when it's not.
    /// \return True if cascades can be added/removed
    public: Q_INVOKABLE bool CascadesEditable() const;

    /// \brief Notify this property has changed
    signals: void EnabledChanged();

    /// \brief Notify this property has changed
    signals: void CascadesEditableChanged();

    /// \brief Notify various properties may have changed
    signals: void SettingsChanged();

    /// \brief Notify fast-to-rebuild properties may have changed
    signals: void LightingChanged();

    /// \brief Notify debug visualization has changed
    signals: void DebugVisualizationModeChanged();

    /// \brief Notify camera list has changed
    signals: void CameraListChanged();

    /// \brief Tells QML to add a cascade from UI thread. MUST start lowercase.
    signals: void qmlAddCascade();

    /// \brief Tells QML to add a cascade from UI thread. MUST start lowercase.
    /// \param[in] _resX Resolution X
    /// \param[in] _resY Resolution Y
    /// \param[in] _resZ Resolution Z
    /// \param[in] _octX Octant Count X
    /// \param[in] _octY Octant Count Y
    /// \param[in] _octZ Octant Count Z
    /// \param[in] _ahsX Area Half Size X
    /// \param[in] _ahsY Area Half Size Y
    /// \param[in] _ahsZ Area Half Size Z
    /// \param[in] _thinWallCounter thinWallCounter parameter
    signals: void qmlAddCascade2(quint32 _resX, quint32 _resY, quint32 _resZ,
                                 quint32 _octX, quint32 _octY, quint32 _octZ,
                                 float _ahsX, float _ahsY, float _ahsZ,
                                 float _thinWallCounter);

    /// \brief See rendering::GlobalIlluminationCiVct::SetBounceCount
    /// \param[in] _bounces See GlobalIlluminationCiVct::SetBounceCount
    public: Q_INVOKABLE void SetBounceCount(const uint32_t _bounces);

    /// \brief See rendering::GlobalIlluminationCiVct::BounceCount
    /// \return See rendering::GlobalIlluminationCiVct::BounceCount
    public: Q_INVOKABLE uint32_t BounceCount() const;

    /// \brief See rendering::GlobalIlluminationCiVct::SetHighQuality
    /// \param[in] _quality See GlobalIlluminationCiVct::SetHighQuality
    public: Q_INVOKABLE void SetHighQuality(const bool _quality);

    /// \brief See rendering::GlobalIlluminationCiVct::HighQuality
    /// \return See rendering::GlobalIlluminationCiVct::HighQuality
    public: Q_INVOKABLE bool HighQuality() const;

    /// \brief See rendering::GlobalIlluminationCiVct::SetAnisotropic
    /// \param[in] _anisotropic See GlobalIlluminationCiVct::SetAnisotropic
    public: Q_INVOKABLE void SetAnisotropic(const bool _anisotropic);

    /// \brief See rendering::GlobalIlluminationCiVct::Anisotropic
    /// \return See rendering::GlobalIlluminationCiVct::Anisotropic
    public: Q_INVOKABLE bool Anisotropic() const;

    /// \brief See rendering::GlobalIlluminationCiVct::SetDebugVisualizationMode
    /// \param[in] _visMode
    /// See GlobalIlluminationCiVct::SetDebugVisualizationMode
    public: Q_INVOKABLE void SetDebugVisualizationMode(const uint32_t _visMode);

    /// \brief See rendering::GlobalIlluminationCiVct::DebugVisualizationMode
    /// \return See rendering::GlobalIlluminationCiVct::DebugVisualizationMode
    public: Q_INVOKABLE uint32_t DebugVisualizationMode() const;

    /// \brief Binds the given camera as active for the center of all cascades
    /// \param[in] _cameraName Name of an existing camera to track
    /// Currently it must be a Camera in the GUI. In the future something
    /// more advanced to synchronize with Server will be needed.
    public: Q_INVOKABLE void OnCamareBind(const QString &_cameraName);

    /// \brief See OnRefreshCameras. Does not lock.
    private: void OnRefreshCamerasImpl();

    /// \brief Populates available cameras
    public: Q_INVOKABLE void OnRefreshCameras();

    /// \brief Populates available cameras
    public: Q_INVOKABLE QStringList CameraList();

    /// \brief Adds a new cascade based on the previous one (if there's any)
    /// \return A CiVctCascade ptr to be used by QML
    public: Q_INVOKABLE QObject* AddCascade();

    /// \brief Pops the last created cascade
    public: Q_INVOKABLE void PopCascade();

    /// \brief Retrieves an existing cascade
    /// \param _idx Index of the cascade. Must be in range (you must track how
    /// many times AddCascade() & PopCascade() has been called).
    /// \return A CiVctCascade ptr to be used by QML
    public: Q_INVOKABLE QObject* GetCascade(int _idx) const;

    /// \brief Disables this GI solution and makes cascades editable again
    public: Q_INVOKABLE void ResetCascades();

    /// \brief Returns true if current UI settings are valid.
    /// Not all settings are valid, e.g.
    ///   - Not having any cascade
    ///   - Not having a camera bound
    ///   - etc
    /// \return Returns true if settings are valid.
    private: bool ValidSettings() const;

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<GlobalIlluminationCiVctPrivate> dataPtr;
  };
}
}
}
#endif

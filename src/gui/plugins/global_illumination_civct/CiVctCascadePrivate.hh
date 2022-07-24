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

#ifndef GZ_SIM_GUI_CIVCTCASCADEPRIVATE_HH_
#define GZ_SIM_GUI_CIVCTCASCADEPRIVATE_HH_

#include <memory>
#include <mutex>

#include "gz/sim/gui/GuiSystem.hh"
#include "gz/gui/qt.h"

#include "Tsa.hh"

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_SIM_VERSION_NAMESPACE
    {
      /// Forward declare the only ptr we need
      class CiVctCascade;
      typedef std::shared_ptr<CiVctCascade> CiVctCascadePtr;
    }  // namespace GZ_SIM_GAZEBO_VERSION_NAMESPACE
  }  // namespace rendering
}  // namespace gz

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  class GlobalIlluminationCiVct;

  /// \brief Cascade container for QML to control all of its settings,
  /// per cascade.
  ///
  /// Due to how QML bindings work, we must split Vectors
  /// into each component so e.g. the following Javascript code:
  ///
  ///   cascade.areaHalfSizeX = 5.0
  ///   cascade.areaHalfSizeY = 6.0
  ///   cascade.areaHalfSizeZ = 7.0
  ///
  /// Will end up calling:
  ///
  ///   cascade->SetAreaHalfSizeX(5.0f);
  ///   cascade->SetAreaHalfSizeY(6.0f);
  ///   cascade->SetAreaHalfSizeZ(7.0f);
  ///
  /// Even though in C++ we would normally do math::Vector3d(5.0f, 6.0f, 7.0f)
  /// The same goes for each property.
  class GZ_SIM_HIDDEN CiVctCascadePrivate : public QObject
  {
    friend class GlobalIlluminationCiVct;

    Q_OBJECT

    Q_PROPERTY(
      float areaHalfSizeX
      READ AreaHalfSizeX
      WRITE SetAreaHalfSizeX
      NOTIFY SettingsChanged
    )
    Q_PROPERTY(
      float areaHalfSizeY
      READ AreaHalfSizeY
      WRITE SetAreaHalfSizeY
      NOTIFY SettingsChanged
    )
    Q_PROPERTY(
      float areaHalfSizeZ
      READ AreaHalfSizeZ
      WRITE SetAreaHalfSizeZ
      NOTIFY SettingsChanged
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
      float thinWallCounter
      READ ThinWallCounter
      WRITE SetThinWallCounter
      // NOTIFY LightingChanged
    )

    /// \brief Constructor
    /// \param _serviceMutex Mutex owned by our creator so we don't access
    /// this data from the UI thread while the render thread is using it
    /// \param _creator Our creator & owner
    /// \param _cascade The cascade we will be manipulating via GUI
    public: CiVctCascadePrivate(std::mutex &_serviceMutex,
                                GlobalIlluminationCiVct &_creator,
                                rendering::CiVctCascadePtr _cascade);

    /// \brief Destructor
    public: ~CiVctCascadePrivate() override;

    /// \brief Set VCT resolution
    /// \param[in] _axis Axis (width, height, depth). In range [0; 3)
    /// \param[in] _res New resolution
    public: Q_INVOKABLE void UpdateResolution(int _axis, uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief Set VCT octant count
    /// \param[in] _axis Axis (width, height, depth). In range [0; 3)
    /// \param[in] _count New octant count
    public: Q_INVOKABLE void UpdateOctantCount(int _axis, uint32_t _count)
        EXCLUDES(serviceMutex);

    /// \brief Set the area half size
    /// \param[in] _axis Axis (width, height, depth). In range [0; 3)
    /// \param[in] _halfSize New half size for that axis
    public: Q_INVOKABLE void UpdateAreaHalfSize(int _axis, float _halfSize)
        EXCLUDES(serviceMutex);

    /// \brief Notify various properties may have changed
    signals: void SettingsChanged();

    /// \brief See rendering::CiVctCascade::SetAreaHalfSize
    /// \param[in] _v See CiVctCascade::SetAreaHalfSize
    public: Q_INVOKABLE void SetAreaHalfSizeX(const float _v)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::AreaHalfSize
    /// Affects X component only
    /// \return See rendering::CiVctCascade::AreaHalfSize
    public: Q_INVOKABLE float AreaHalfSizeX() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::SetAreaHalfSize
    /// Affects Y component only
    /// \param[in] _v See CiVctCascade::SetAreaHalfSize
    public: Q_INVOKABLE void SetAreaHalfSizeY(const float _v)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::AreaHalfSize.
    /// Affects Y component only
    /// \return See rendering::CiVctCascade::AreaHalfSize
    public: Q_INVOKABLE float AreaHalfSizeY() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::SetAreaHalfSize
    /// Affects Z component only
    /// \param[in] _v See CiVctCascade::SetAreaHalfSize
    public: Q_INVOKABLE void SetAreaHalfSizeZ(const float _v)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::AreaHalfSize
    /// Affects Z component only
    /// \return See rendering::CiVctCascade::AreaHalfSize
    public: Q_INVOKABLE float AreaHalfSizeZ() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::SetResolution
    /// \param[in] _res See CiVctCascadePrivate::SetResolution
    public: Q_INVOKABLE void SetResolutionX(const uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::Resolution
    /// \return See rendering::CiVctCascade::Resolution
    public: Q_INVOKABLE uint32_t ResolutionX() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::SetResolution
    /// \param[in] _res See CiVctCascadePrivate::SetResolution
    public: Q_INVOKABLE void SetResolutionY(const uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::Resolution
    /// \return See rendering::CiVctCascade::Resolution
    public: Q_INVOKABLE uint32_t ResolutionY() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::SetResolution
    /// \param[in] _res See CiVctCascadePrivate::SetResolution
    public: Q_INVOKABLE void SetResolutionZ(const uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::Resolution
    /// \return See rendering::CiVctCascade::Resolution
    public: Q_INVOKABLE uint32_t ResolutionZ() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::SetOctantCount
    /// \param[in] _octantCount See CiVctCascadePrivate::SetOctantCount
    public: Q_INVOKABLE void SetOctantCountX(const uint32_t _octantCount)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::OctantCount
    /// \return See rendering::CiVctCascade::OctantCount
    public: Q_INVOKABLE uint32_t OctantCountX() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::SetOctantCount
    /// \param[in] _octantCount See CiVctCascadePrivate::SetOctantCount
    public: Q_INVOKABLE void SetOctantCountY(const uint32_t _octantCount)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::OctantCount
    /// \return See rendering::CiVctCascade::OctantCount
    public: Q_INVOKABLE uint32_t OctantCountY() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::SetOctantCount
    /// \param[in] _octantCount See CiVctCascadePrivate::SetOctantCount
    public: Q_INVOKABLE void SetOctantCountZ(const uint32_t _octantCount)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::OctantCount
    /// \return See rendering::CiVctCascade::OctantCount
    public: Q_INVOKABLE uint32_t OctantCountZ() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::SetThinWallCounter
    /// \param[in] _thinWallCounter See CiVctCascadePrivate::SetThinWallCounter
    public: Q_INVOKABLE void SetThinWallCounter(const float _thinWallCounter)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascade::ThinWallCounter
    /// \return See rendering::CiVctCascade::ThinWallCounter
    public: Q_INVOKABLE float ThinWallCounter() const EXCLUDES(serviceMutex);

    /// \brief Cascade under under control where all settings are stored
    private: rendering::CiVctCascadePtr cascade PT_GUARDED_BY(serviceMutex);

    /// \brief Our creator, to avoid updating settings when it's not
    /// safe to do so
    private: GlobalIlluminationCiVct &creator PT_GUARDED_BY(serviceMutex);

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    public: std::mutex &serviceMutex;
  };
}
}
}
#endif

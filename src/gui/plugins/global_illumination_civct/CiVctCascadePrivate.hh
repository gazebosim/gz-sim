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

#ifndef IGNITION_GAZEBO_GUI_CIVCTCASCADEPRIVATE_HH_
#define IGNITION_GAZEBO_GUI_CIVCTCASCADEPRIVATE_HH_

#include <memory>
#include <mutex>

#include "ignition/gazebo/gui/GuiSystem.hh"
#include "ignition/gui/qt.h"

#include "Tsa.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
    {
      /// Forward declare the only ptr we need
      class CiVctCascade;
      typedef std::shared_ptr<CiVctCascade> CiVctCascadePtr;
    }  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
  }  // namespace rendering
}  // namespace ignition

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
  /// \brief TBD
  class IGNITION_GAZEBO_HIDDEN CiVctCascadePrivate : public QObject
  {
    friend class GlobalIlluminationCiVct;

    Q_OBJECT

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
    public: CiVctCascadePrivate(std::mutex &_serviceMutex,
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
    /// \param[in] _res New octant count
    public: Q_INVOKABLE void UpdateOctantCount(int _axis, uint32_t _count)
        EXCLUDES(serviceMutex);

    /// \brief Notify various properties may have changed
    signals: void SettingsChanged();

    /// \brief See rendering::CiVctCascadePrivate::SetResolution
    /// \param[in] _enabled See CiVctCascadePrivate::SetResolution
    public: Q_INVOKABLE void SetResolutionX(const uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::Resolution
    /// \return See rendering::CiVctCascadePrivate::Resolution
    public: Q_INVOKABLE uint32_t ResolutionX() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::SetResolution
    /// \param[in] _enabled See CiVctCascadePrivate::SetResolution
    public: Q_INVOKABLE void SetResolutionY(const uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::Resolution
    /// \return See rendering::CiVctCascadePrivate::Resolution
    public: Q_INVOKABLE uint32_t ResolutionY() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::SetResolution
    /// \param[in] _enabled See CiVctCascadePrivate::SetResolution
    public: Q_INVOKABLE void SetResolutionZ(const uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::Resolution
    /// \return See rendering::CiVctCascadePrivate::Resolution
    public: Q_INVOKABLE uint32_t ResolutionZ() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::SetOctantCount
    /// \param[in] _enabled See CiVctCascadePrivate::SetOctantCount
    public: Q_INVOKABLE void SetOctantCountX(const uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::OctantCount
    /// \return See rendering::CiVctCascadePrivate::OctantCount
    public: Q_INVOKABLE uint32_t OctantCountX() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::SetOctantCount
    /// \param[in] _enabled See CiVctCascadePrivate::SetOctantCount
    public: Q_INVOKABLE void SetOctantCountY(const uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::OctantCount
    /// \return See rendering::CiVctCascadePrivate::OctantCount
    public: Q_INVOKABLE uint32_t OctantCountY() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::SetOctantCount
    /// \param[in] _enabled See CiVctCascadePrivate::SetOctantCount
    public: Q_INVOKABLE void SetOctantCountZ(const uint32_t _res)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::OctantCount
    /// \return See rendering::CiVctCascadePrivate::OctantCount
    public: Q_INVOKABLE uint32_t OctantCountZ() const
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::SetThinWallCounter
    /// \param[in] _enabled See CiVctCascadePrivate::SetThinWallCounter
    public: Q_INVOKABLE void SetThinWallCounter(const float _thinWallCounter)
        EXCLUDES(serviceMutex);

    /// \brief See rendering::CiVctCascadePrivate::ThinWallCounter
    /// \return See rendering::CiVctCascadePrivate::ThinWallCounter
    public: Q_INVOKABLE float ThinWallCounter() const EXCLUDES(serviceMutex);

    private: rendering::CiVctCascadePtr cascade PT_GUARDED_BY(serviceMutex);

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    public: std::mutex &serviceMutex;
  };
}
}
}
#endif

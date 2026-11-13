/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_GUI_PERFORMANCEOVERLAY_HH_
#define GZ_SIM_GUI_PERFORMANCEOVERLAY_HH_

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>

#include "gz/gui/qt.h"

#ifndef _WIN32
#  define PerformanceOverlay_EXPORTS_API \
     __attribute__ ((visibility ("default")))
#else
#  if (defined(PerformanceOverlay_EXPORTS))
#    define PerformanceOverlay_EXPORTS_API __declspec(dllexport)
#  else
#    define PerformanceOverlay_EXPORTS_API __declspec(dllimport)
#  endif
#endif

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  class PerformanceOverlayPrivate;

  /// \brief Shows over the 3D view what limits the real time factor: whether
  /// it is the CPU or the GPU, where the time of a step goes, and which
  /// sensors, models and systems cost the most. Clicking on a model or a
  /// sensor selects its model in the scene.
  ///
  /// The data comes from the performance monitor of the server. The plugin
  /// starts it when it opens and leaves it as it is when it closes.
  ///
  /// Opened from the plugin menu it floats at the top right corner of the
  /// plugin titled "3D View". To place it in a GUI configuration, give it a
  /// `gz-gui` element as with any other plugin.
  class PerformanceOverlay_EXPORTS_API PerformanceOverlay
      : public sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Last report, empty until the first one arrives
    Q_PROPERTY(
      QVariantMap report
      READ Report
      NOTIFY ReportChanged
    )

    /// \brief Last reports, oldest first
    Q_PROPERTY(
      QVariantList history
      READ History
      NOTIFY ReportChanged
    )

    /// \brief Why there is nothing new to show, empty if there is
    Q_PROPERTY(
      QString notice
      READ Notice
      NOTIFY NoticeChanged
    )

    /// \brief True if the server is monitoring its performance
    Q_PROPERTY(
      bool monitoring
      READ Monitoring
      NOTIFY MonitoringChanged
    )

    /// \brief True if the simulation is paused
    Q_PROPERTY(
      bool paused
      READ Paused
      NOTIFY PausedChanged
    )

    /// \brief Constructor
    public: PerformanceOverlay();

    /// \brief Destructor
    public: ~PerformanceOverlay() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;

    /// \brief Get the last report
    /// \return The report, empty until the first one arrives
    public: Q_INVOKABLE QVariantMap Report() const;

    /// \brief Get the last reports
    /// \return Reports, oldest first
    public: Q_INVOKABLE QVariantList History() const;

    /// \brief Get why there is nothing new to show
    /// \return The text, empty if there is something to show
    public: Q_INVOKABLE QString Notice() const;

    /// \brief Get whether the server is monitoring its performance
    /// \return True if it is
    public: Q_INVOKABLE bool Monitoring() const;

    /// \brief Get whether the simulation is paused
    /// \return True if it is
    public: Q_INVOKABLE bool Paused() const;

    /// \brief Ask the server to start or stop monitoring its performance
    /// \param[in] _enable True to start
    public: Q_INVOKABLE void SetMonitoring(bool _enable);

    /// \brief Select a top level model in the scene
    /// \param[in] _name Name of the model
    public: Q_INVOKABLE void SelectModel(const QString &_name);

    /// \brief Notify that there is a new report
    signals: void ReportChanged();

    /// \brief Notify that the notice changed
    signals: void NoticeChanged();

    /// \brief Notify that the state of the monitor changed
    signals: void MonitoringChanged();

    /// \brief Notify that the simulation was paused or resumed
    signals: void PausedChanged();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<PerformanceOverlayPrivate> dataPtr;
  };
}
}  // namespace sim
}  // namespace gz

#endif

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

#ifndef GZ_SIM_GUI_ENVIRONMENTLOADER_HH_
#define GZ_SIM_GUI_ENVIRONMENTLOADER_HH_

#include <memory>

#include "gz/sim/gui/GuiSystem.hh"
#include "gz/gui/qt.h"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  class EnvironmentLoaderPrivate;

  /// \class EnvironmentLoader EnvironmentLoader.hh
  ///     gz/sim/systems/EnvironmentLoader.hh
  /// \brief A GUI plugin for a user to load an Environment
  /// component into the ECM on a live simulation.
  class EnvironmentLoader : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Data path
    Q_PROPERTY(
      QString dataPath
      READ DataPath
      WRITE SetDataPath
      NOTIFY DataPathChanged
    )

    /// \brief Dimension list
    Q_PROPERTY(
      QStringList dimensionList
      READ DimensionList
      NOTIFY DimensionListChanged
    )

    /// \brief Unit list
    Q_PROPERTY(
      QStringList unitList
      READ UnitList
    )

        /// \brief Unit list
    Q_PROPERTY(
      QString unit
      READ Unit
      WRITE SetUnit
      NOTIFY UnitChanged
    )

    /// \brief Time index
    Q_PROPERTY(
      int timeIndex
      READ TimeIndex
      WRITE SetTimeIndex
      NOTIFY TimeIndexChanged
    )

    /// \brief X dimension
    Q_PROPERTY(
      int xIndex
      READ XIndex
      WRITE SetXIndex
      NOTIFY XIndexChanged
    )

    /// \brief Y dimension
    Q_PROPERTY(
      int yIndex
      READ YIndex
      WRITE SetYIndex
      NOTIFY YIndexChanged
    )

    /// \brief Z dimension
    Q_PROPERTY(
      int zIndex
      READ ZIndex
      WRITE SetZIndex
      NOTIFY ZIndexChanged
    )

    /// \brief Spatial reference type list
    Q_PROPERTY(
      QStringList referenceList
      READ ReferenceList
    )

    /// \brief Spatial reference
    Q_PROPERTY(
      QString reference
      READ Reference
      WRITE SetReference
      NOTIFY ReferenceChanged
    )

    /// \brief Configuration ready
    Q_PROPERTY(
      bool configured
      READ IsConfigured
      NOTIFY IsConfiguredChanged
    )

    /// \brief Constructor
    public: EnvironmentLoader();

    /// \brief Destructor
    public: ~EnvironmentLoader() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    /// \brief Get path to the data file to be loaded
    public: Q_INVOKABLE QString DataPath() const;

    /// \brief Notify that the path to the data file (potentially) changed
    signals: void DataPathChanged();

    /// \brief Set the path to the data file to be loaded
    public: Q_INVOKABLE void SetDataPath(QString _dataPath);

    /// \brief Set the URL pointing to the data file to be loaded
    public: Q_INVOKABLE void SetDataUrl(QUrl _dataUrl);

    /// \brief Get dimensions available in the data file
    public: Q_INVOKABLE QStringList DimensionList() const;

    /// \brief Get available units
    public: Q_INVOKABLE QStringList UnitList() const;

    /// \brief Notify that the list of dimensions has changed
    signals: void DimensionListChanged();

    /// \brief Get index of the time dimension in the list
    public: Q_INVOKABLE int TimeIndex() const;

    /// \brief Set index of the time dimension in the list
    public: Q_INVOKABLE void SetTimeIndex(int _timeIndex);

    /// \brief Notify the time dimension index has changed
    signals: void TimeIndexChanged() const;

    /// \brief Get index of the x dimension in the list
    public: Q_INVOKABLE int XIndex() const;

    /// \brief Set index of the x dimension in the list
    public: Q_INVOKABLE void SetXIndex(int _xIndex);

    /// \brief Notify the x dimension index has changed
    signals: void XIndexChanged() const;

    /// \brief Get index of the y dimension in the list
    public: Q_INVOKABLE int YIndex() const;

    /// \brief Notify the y dimension index has changed
    signals: void YIndexChanged() const;

    /// \brief Set index of the y dimension in the list
    public: Q_INVOKABLE void SetYIndex(int _yIndex);

    /// \brief Get index of the z dimension in the list
    public: Q_INVOKABLE int ZIndex() const;

    /// \brief Set index of the z dimension in the list
    public: Q_INVOKABLE void SetZIndex(int _zIndex);

    /// \brief Notify the z dimension index has changed
    signals: void ZIndexChanged() const;

    /// \brief Get supported spatial references
    public: Q_INVOKABLE QStringList ReferenceList() const;

    /// \brief Get spatial reference
    public: Q_INVOKABLE QString Reference() const;

    /// \brief Set spatial reference
    public: Q_INVOKABLE void SetReference(QString _reference);

    /// \brief Notify the spatial reference has changed
    signals: void ReferenceChanged() const;

    /// \brief Get index of the unit in the list
    public: Q_INVOKABLE QString Unit() const;

    /// \brief Set index of the unit in the list
    public: Q_INVOKABLE void SetUnit(QString _unit);

    /// \brief Notify the unit index has changed
    signals: void UnitChanged() const;

    /// \brief Get configuration status
    public: Q_INVOKABLE bool IsConfigured() const;

    /// \brief Notify configuration status changed
    signals: void IsConfiguredChanged();

    /// \brief Schedule an update
    public: Q_INVOKABLE void ScheduleLoad();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<EnvironmentLoaderPrivate> dataPtr;
  };
}
}
}
#endif

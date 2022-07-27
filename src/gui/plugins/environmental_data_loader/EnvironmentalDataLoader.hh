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

#ifndef GZ_SIM_GUI_ENVIRONMENTALDATALOADER_HH_
#define GZ_SIM_GUI_ENVIRONMENTALDATALOADER_HH_

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
  class EnvironmentalDataLoaderPrivate;

  class EnvironmentalDataLoader : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Dimension list
    Q_PROPERTY(
      QStringList dimensionList
      READ DimensionList
      NOTIFY DimensionListChanged
    )

    /// \brief Time index
    Q_PROPERTY(
      QString timeIndex
      READ TimeIndex
      WRITE SetTimeIndex
      NOTIFY TimeIndexChanged
    )

    /// \brief X dimension
    Q_PROPERTY(
      QString xIndex
      READ XIndex
      WRITE SetXIndex
      NOTIFY XIndexChanged
    )

    /// \brief Y dimension
    Q_PROPERTY(
      QString yIndex
      READ YIndex
      WRITE SetYIndex
      NOTIFY YIndexChanged
    )

    /// \brief Z dimension
    Q_PROPERTY(
      QString zIndex
      READ ZIndex
      WRITE SetZIndex
      NOTIFY ZIndexChanged
    )

    /// \brief Configuration ready
    Q_PROPERTY(
      bool configured
      READ IsConfigured
      NOTIFY IsConfiguredChanged
    )

    /// \brief Constructor
    public: EnvironmentalDataLoader();

    /// \brief Destructor
    public: ~EnvironmentalDataLoader() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &_ecm) override;

    /// \brief Set the path to the data file to be loaded
    public: Q_INVOKABLE void SetDataPath(QUrl _dataPath);

    /// \brief Get dimensions available in the data file
    public: Q_INVOKABLE QStringList DimensionList() const;

    /// \brief Notify that the list of dimensions has changed
    signals: void DimensionListChanged();

    /// \brief Get index of the x dimension in the list
    public: Q_INVOKABLE int XIndex() const;

    /// \brief Set index of the x dimension in the list
    public: Q_INVOKABLE int SetXIndex();

    /// \brief Notify the x dimension index has changed
    signals: void XIndexChanged() const;

    /// \brief Get index of the y dimension in the list
    public: Q_INVOKABLE int YIndex() const;

    /// \brief Set index of the y dimension in the list
    public: Q_INVOKABLE int SetYIndex();

    /// \brief Notify the y dimension index has changed
    signals: void YIndexChanged();

    /// \brief Get index of the z dimension in the list
    public: Q_INVOKABLE int ZIndex() const;

    /// \brief Set index of the z dimension in the list
    public: Q_INVOKABLE int SetZIndex();

    /// \brief Notify the z dimension index has changed
    signals: void ZIndexChanged();

    /// \brief Get configuration status
    public: Q_INVOKABLE bool IsConfigured() const;

    /// \brief Notify configuration status changed
    signals: void IsConfiguredChanged();

    /// \brief Schedule an update
    public: Q_INVOKABLE void ScheduleUpdate();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<EnvironmentalDataLoaderPrivate> dataPtr;
  };
}
}
}
#endif

/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#ifndef GZ_SIM_GUI_EXPORT_OCCUPANCY_HH_
#define GZ_SIM_GUI_EXPORT_OCCUPANCY_HH_

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
  class ExportOccupancyUiPrivate;

  /// \brief A GUI plugin for a user to export the occupancy
  /// grid of the current world.
  class ExportOccupancyUi : public gz::sim::GuiSystem
  {
    Q_OBJECT
    /// \brief Constructor
    public: ExportOccupancyUi();

    /// \brief Destructor
    public: ~ExportOccupancyUi() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    /// Triggers the export of our occupancy grid
    public: Q_INVOKABLE void StartExport(double _samples, double _range,
      double _rangeRes, double _angularRes,
      double _distanceFromGround, double _gridResolution,
      std::size_t _numWidth, std::size_t _numHeight);

    /// Trigger scan start
    public: Q_INVOKABLE void StartExploration();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<ExportOccupancyUiPrivate> dataPtr;
  };
}
}
}
#endif

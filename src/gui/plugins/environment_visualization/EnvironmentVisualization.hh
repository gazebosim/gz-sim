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

#ifndef GZ_SIM_GUI_ENVIRONMENTVISUALIZATION_HH_
#define GZ_SIM_GUI_ENVIRONMENTVISUALIZATION_HH_

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
  class EnvironmentVisualizationTool;

  /// \class EnvironmentVisualization EnvironmentVisualization.hh
  ///     gz/sim/systems/EnvironmentVisualization.hh
  /// \brief A GUI plugin for a user to load an Environment
  /// component into the ECM on a live simulation.
  class EnvironmentVisualization : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Samples along x axis
    Q_PROPERTY(unsigned int xSamples MEMBER xSamples)

    /// \brief Samples along y axis
    Q_PROPERTY(unsigned int ySamples MEMBER ySamples)

    /// \brief Samples along z axis
    Q_PROPERTY(unsigned int zSamples MEMBER zSamples)

    /// \brief Constructor
    public: EnvironmentVisualization();

    /// \brief Destructor
    public: ~EnvironmentVisualization() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    public: Q_INVOKABLE void ResamplePointcloud();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<EnvironmentVisualizationTool> dataPtr;

    public: unsigned int xSamples{10};

    public: unsigned int ySamples{10};

    public: unsigned int zSamples{10};

    private: QTimer* qtimer;
  };
}
}
}
#endif

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
  class EnvironmentVisualizationPrivate;

  /// \class EnvironmentVisualization EnvironmentVisualization.hh
  ///     gz/sim/systems/EnvironmentVisualization.hh
  /// \brief A GUI plugin for a user to load an Environment
  /// component into the ECM on a live simulation.
  class EnvironmentVisualization : public gz::sim::GuiSystem
  {
    Q_OBJECT

    Q_PROPERTY(qreal xResolution MEMBER xResolution NOTIFY xResolutionChanged);
    Q_PROPERTY(qreal yResolution MEMBER yResolution NOTIFY yResolutionChanged);
    Q_PROPERTY(qreal zResolution MEMBER zResolution NOTIFY zResolutionChanged);

    /// \brief Constructor
    public: EnvironmentVisualization();

    /// \brief Destructor
    public: ~EnvironmentVisualization() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    signals: void xResolutionChanged();

    signals: void yResolutionChanged();

    signals: void zResolutionChanged();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<EnvironmentVisualizationPrivate> dataPtr;

    public: qreal xResolution{10};

    public: qreal yResolution{10};

    public: qreal zResolution{10};
  };
}
}
}
#endif

/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_SIM_GUI_GUISYSTEM_HH_
#define GZ_SIM_GUI_GUISYSTEM_HH_

#include <QtCore>

#include <gz/sim/config.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/gui/Export.hh>
#include <gz/gui/Plugin.hh>

#include <sdf/Element.hh>

namespace gz
{
namespace sim
{
  // Inline bracket to help doxygen filtering.
  inline namespace GZ_SIM_VERSION_NAMESPACE {
  /// \brief Base class for a GUI System.
  ///
  /// A System operates on Entities that have certain Components. A System
  /// will only operate on an Entity if it has all of the required
  /// Components.
  ///
  /// GUI systems are different from `gz::sim::System`s because they
  /// don't run in the same process as the physics. Instead, they run in a
  /// separate process that is stepped by updates coming through the network
  class GZ_SIM_GUI_VISIBLE GuiSystem : public gz::gui::Plugin
  {
    Q_OBJECT

    /// \brief Update callback called every time the system is stepped.
    /// This is called at a Gazebo Transport thread, so any interaction
    /// with Qt should be done through signals and slots.
    /// \param[in] _info Current simulation information, such as time.
    /// \param[in] _ecm Mutable reference to the ECM, so the system can read
    /// and write entities and their components.
    public: virtual void Update(const UpdateInfo &_info,
                                EntityComponentManager &_ecm)
    {
      // This will avoid many doxygen warnings
      (void)_info;
      (void)_ecm;
    }
  };
}
}
}
#endif

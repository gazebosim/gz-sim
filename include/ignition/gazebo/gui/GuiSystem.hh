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
#ifndef IGNITION_GAZEBO_GUI_GUISYSTEM_HH_
#define IGNITION_GAZEBO_GUI_GUISYSTEM_HH_

#include <memory>
#include <QtCore>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gui/Plugin.hh>

#include <sdf/Element.hh>

namespace ignition
{
namespace gazebo
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  /// \class System System.hh ignition/gazebo/System.hh
  /// \brief Base class for a System.
  ///
  /// A System operates on Entities that have certain Components. A System
  /// will only operate on an Entity if it has all of the required
  /// Components.
  ///
  /// Systems are executed in three phases:
  ///  * PreUpdate
  ///    * Has read-write access to world entities and components
  ///    * Executed with simulation time at (t0)
  ///    * Can be used to modify state before physics runs, for example for
  ///      applying control signals or performing network syncronization.
  ///  * Update
  ///    * Has read-write access to world entities and components
  ///    * Responsible for propagating time from (t0) to (t0 + dt)
  ///    * Used for physics simulation step
  ///  * PostUpdate
  ///    * Has read-only access to world entities and components
  ///    * Executed with simulation time at (t0 + dt)
  ///    * Used to read out results at the end of a simulation step to be used
  ///      for sensor or controller updates.

  /// \class GuiSystemPreUpdate GuiSystem.hh ignition/gazebo/System.hh
  /// \brief Interface for a system that uses the PreUpdate phase
  class IGNITION_GAZEBO_VISIBLE GuiPlugin : public ignition::gui::Plugin
  {
    Q_OBJECT

    public: virtual void PreUpdate(const UpdateInfo &,
                                   EntityComponentManager &){};

    public: virtual void Update(const UpdateInfo &,
                                EntityComponentManager &){};

    public: virtual void PostUpdate(const UpdateInfo &,
                                    const EntityComponentManager &){};
  };
}
}
}
#endif

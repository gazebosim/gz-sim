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
#ifndef GZ_SIM_EVENTS_HH_
#define GZ_SIM_EVENTS_HH_

#include <sdf/Element.hh>
#include <sdf/Plugin.hh>

#include <gz/common/Event.hh>

#include "gz/sim/config.hh"
#include "gz/sim/Entity.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    /// \brief Namespace for all events. Refer to the EventManager class for
    /// more information about events.
    namespace events
    {
      /// \brief The pause event can be used to pause or unpause simulation.
      /// Emit a value of true to pause simulation, and emit a value of false
      /// to unpause simulation.
      ///
      /// For example, to pause simulation use:
      /// \code
      /// eventManager.Emit<gz::sim::events::Pause>(true);
      /// \endcode
      using Pause = gz::common::EventT<void(bool), struct PauseTag>;

      /// \brief The stop event can be used to terminate simulation.
      /// Emit this signal to terminate an active simulation.
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<gz::sim::events::Stop>();
      /// \endcode
      using Stop = gz::common::EventT<void(void), struct StopTag>;

      /// \brief Event used to load plugins for an entity into simulation.
      /// Pass in the entity which will own the plugins, and an SDF element for
      /// the entity, which may contain multiple `<plugin>` tags.
      /// Makre sure that you don't also connect to the LoadPlugins event.
      using LoadSdfPlugins = common::EventT<void(Entity, sdf::Plugins),
          struct LoadPluginsTag>;
      }
    }  // namespace events
  }  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_EVENTS_HH_

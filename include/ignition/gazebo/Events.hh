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
#ifndef IGNITION_GAZEBO_EVENTS_HH_
#define IGNITION_GAZEBO_EVENTS_HH_

#include <sdf/Element.hh>

#include <ignition/common/Event.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
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
      /// eventManager.Emit<ignition::gazebo::events::Pause>(true);
      /// \endcode
      using Pause = ignition::common::EventT<void(bool), struct PauseTag>;

      /// \brief The stop event can be used to terminate simulation.
      /// Emit this signal to terminate an active simulation.
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<ignition::gazebo::events::Stop>();
      /// \endcode
      using Stop = ignition::common::EventT<void(void), struct StopTag>;

      /// \brief Event used to load plugins for an entity into simulation.
      /// Pass in the entity which will own the plugins, and an SDF element for
      /// the entity, which may contain multiple <plugin> tags.
      using LoadPlugins = common::EventT<void(Entity, sdf::ElementPtr),
          struct LoadPluginsTag>;

      /// \brief Event used to emit a render event when running in one process.
      /// This is required because we have two RenderUtil classes when there
      /// is a render sensor in the scene (camera, depth sensor, etc).
      /// We could only have one thread updating the render scene, with this
      /// signal we are able to call from the GzSceneManager the render calls
      /// required by the sensor
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<ignition::gazebo::events::Render>();
      /// \endcode
      using Render = ignition::common::EventT<void(void), struct RenderTag>;

      /// \brief Event used to emit render event when running in one process
      using EnableSensors =
        ignition::common::EventT<void(bool), struct EnableSensorsTag>;

      /// \brief Event used to emit a render event when running in one process.
      /// Some remove events are lost because of the rate when running in the
      /// same process without sensors. This event is launched in the Simulation
      /// runner when there is any new entity or entity marked to be removed
      /// to remove/add entities in the renderUtil
      using UpdateSystems =
        ignition::common::EventT<void(void), struct UpdateSystemsTag>;
      }
    }  // namespace events
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_EVENTS_HH_

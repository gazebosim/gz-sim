/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_RENDERING_EVENTS_HH_
#define IGNITION_GAZEBO_RENDERING_EVENTS_HH_


#include <ignition/common/Event.hh>

#include "ignition/gazebo/config.hh"

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
      /// \brief The render event is emitted before rendering updates.
      /// The event is emitted in the rendering thread so rendering
      /// calls can ben make in this event callback
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<ignition::gazebo::events::PreRender>();
      /// \endcode
      using PreRender = ignition::common::EventT<void(void),
          struct PreRenderTag>;

      /// \brief The render event is emitted after rendering updates.
      /// The event is emitted in the rendering thread so rendering
      /// calls can ben make in this event callback
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<ignition::gazebo::events::PostRender>();
      /// \endcode
      using PostRender = ignition::common::EventT<void(void),
          struct PostRenderTag>;
      }
    }  // namespace events
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_RENDEREVENTS_HH_

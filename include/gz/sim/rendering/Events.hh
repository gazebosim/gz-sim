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
#ifndef GZ_SIM_RENDERING_EVENTS_HH_
#define GZ_SIM_RENDERING_EVENTS_HH_


#include <gz/common/Event.hh>

#include "gz/sim/config.hh"

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
      /// \brief The render event is emitted when the the scene manager is
      /// updated with contents from the ECM. This event is emitted
      /// before the PreRender event on the server side in the rendering
      /// thread. It is also accessible on the GUI side.
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<gz::sim::events::SceneUpdate>();
      /// \endcode
      using SceneUpdate = gz::common::EventT<void(void),
          struct SceneUpdateTag>;

      /// \brief The pre render event is emitted before rendering updates.
      /// The event is emitted in the rendering thread so rendering
      /// calls can be made in this event callback
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<gz::sim::events::PreRender>();
      /// \endcode
      using PreRender = gz::common::EventT<void(void),
          struct PreRenderTag>;

      /// \brief The render event is emitted during rendering updates.
      /// The event is emitted in the rendering thread so rendering
      /// calls can be made in this event callback
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<gz::sim::events::Render>();
      /// \endcode
      using Render = gz::common::EventT<void(void),
          struct RenderTag>;

      /// \brief The post render event is emitted after rendering updates.
      /// The event is emitted in the rendering thread so rendering
      /// calls can be made in this event callback
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<gz::sim::events::PostRender>();
      /// \endcode
      using PostRender = gz::common::EventT<void(void),
          struct PostRenderTag>;

      /// \brief The render teardown event is emitted right before the
      /// rendering thread is torn down. The event is emitted in the
      /// rendering thread so last minute, cleanup rendering calls can
      /// be made in this event callback.
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<gz::sim::events::RenderTeardown>();
      /// \endcode
      using RenderTeardown = gz::common::EventT<void(void),
          struct RenderTeardownTag>;

      /// \brief The force render event may be emitted outside the
      /// rendering thread to force rendering calls ie. to ensure
      /// rendering occurs even if it wasn't seemingly necessary.
      ///
      /// Useful for custom, out-of-tree rendering sensors that
      /// need the rendering thread to perform an update.
      ///
      /// For example:
      /// \code
      /// eventManager.Emit<gz::sim::events::ForceRender>();
      /// \endcode
      using ForceRender = gz::common::EventT<void(void),
          struct ForceRenderTag>;
      }
    }  // namespace events
  }  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_RENDEREVENTS_HH_

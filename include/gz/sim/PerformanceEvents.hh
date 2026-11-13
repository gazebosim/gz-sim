/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_PERFORMANCEEVENTS_HH_
#define GZ_SIM_PERFORMANCEEVENTS_HH_

#include <chrono>
#include <unordered_map>

#include <gz/common/Event.hh>

#include "gz/sim/config.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/Export.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    /// \brief Check if the simulation performance is being monitored. Systems
    /// that feed the monitor use it to do no extra work otherwise.
    /// \return True if there is a performance monitor collecting
    GZ_SIM_VISIBLE bool PerformanceMonitorEnabled();

    namespace events
    {
      /// \brief Wall time of one pass of the render thread of the Sensors
      /// system. All fields except for gpuWait are consecutive stages.
      struct RenderFrameTimes
      {
        /// \brief Applying the entity changes to the rendering scene
        std::chrono::steady_clock::duration sceneUpdate{0};

        /// \brief Scene::PreRender and its event
        std::chrono::steady_clock::duration preRender{0};

        /// \brief Updating the sensors, and the Render event
        std::chrono::steady_clock::duration sensors{0};

        /// \brief Scene::PostRender and its event
        std::chrono::steady_clock::duration postRender{0};

        /// \brief Part of the stages above spent blocked on the GPU, which
        /// includes transferring the images out of GPU memory
        std::chrono::steady_clock::duration gpuWait{0};

        /// \brief Time that the simulation thread was blocked waiting for the
        /// render thread since the previous pass. Unlike the other fields it
        /// is not time of the render thread.
        std::chrono::steady_clock::duration simThreadWait{0};

        /// \brief True if the render engine is running on a software
        /// rasterizer, in which case GPU waits are really CPU work.
        bool softwareRenderer{false};
      };

      /// \brief Emitted from the render thread after each pass that updated
      /// the rendering scene, if PerformanceMonitorEnabled() is true.
      using RenderFrameStats = gz::common::EventT<
          void(const RenderFrameTimes &), struct RenderFrameStatsTag>;

      /// \brief Emitted from the Physics system, if
      /// PerformanceMonitorEnabled() is true, with a sample of the number of
      /// contact points that each collision entity currently has.
      using PhysicsContactSample = gz::common::EventT<
          void(const std::unordered_map<Entity, unsigned int> &),
          struct PhysicsContactSampleTag>;
    }  // namespace events
    }
  }  // namespace sim
}  // namespace gz
#endif  // GZ_SIM_PERFORMANCEEVENTS_HH_

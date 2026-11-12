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
#ifndef GZ_SIM_PERFORMANCEMONITOR_HH_
#define GZ_SIM_PERFORMANCEMONITOR_HH_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/msgs/diagnostics.pb.h>

#include "gz/sim/config.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/PerformanceEvents.hh"
#include "gz/sim/System.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    //
    class EntityComponentManager;
    class SystemManager;

    /// \brief Resource that limits how fast the simulation can run.
    enum class PerformanceResource
    {
      /// \brief Nothing, the simulation keeps up with its target.
      kNone,

      /// \brief Computation on the CPU, which includes software rasterizers.
      kCpu,

      /// \brief The GPU.
      kGpu,

      /// \brief Both: the render thread spends comparable time computing and
      /// blocked on the GPU, so speeding up only one of them helps little.
      kCpuAndGpu,

      /// \brief Not computation: a system is blocked waiting for something
      /// outside of the simulation, like a lock-step partner or a socket.
      kExternal,

      /// \brief Rendering is the limit but the render engine in use does not
      /// report its GPU waits, so CPU and GPU can't be told apart.
      kUnknown
    };

    /// \brief Part of the simulation where the limiting time is spent.
    enum class PerformanceLocation
    {
      /// \brief Nowhere, the simulation keeps up with its target.
      kNone,

      /// \brief The physics engine step.
      kPhysics,

      /// \brief Waiting for the render thread to finish the sensors.
      kRendering,

      /// \brief A system other than physics and sensors.
      kSystem,

      /// \brief The simulation loop itself, outside of any system.
      kEngine
    };

    /// \brief The system update that is being timed.
    enum class PerformanceSystemPhase
    {
      kPreUpdate,
      kUpdate,
      kPostUpdate
    };

    /// \brief Time used by one system in a report interval.
    struct PerformanceSystemSample
    {
      /// \brief Plugin name of the system
      std::string name;

      /// \brief Top level model the system is attached to. Empty if it is
      /// attached to the world.
      std::string model;

      /// \brief Wall time inside of the system
      std::chrono::steady_clock::duration wall{0};

      /// \brief Part of the wall time that delayed the simulation step. It
      /// only differs from wall if post updates run in parallel, where only
      /// the slowest system of each step delays it.
      std::chrono::steady_clock::duration critical{0};

      /// \brief Its part of the wall time that the steps lost to their
      /// schedule at the target real time factor, see PerformanceSchedule.
      std::chrono::steady_clock::duration late{0};

      /// \brief Wall time of the updates for which CPU time was measured.
      std::chrono::steady_clock::duration sampledWall{0};

      /// \brief CPU time of the updates for which CPU time was measured.
      std::chrono::steady_clock::duration sampledCpu{0};
    };

    /// \brief Time used by one sensor in a report interval.
    struct PerformanceSensorSample
    {
      /// \brief Scoped name of the sensor
      std::string name;

      /// \brief Top level model the sensor belongs to
      std::string model;

      /// \brief Number of updates that generated data
      uint64_t updates{0};

      /// \brief Wall time of the updates
      std::chrono::steady_clock::duration total{0};

      /// \brief Part of the total spent blocked on the GPU
      std::chrono::steady_clock::duration gpuWait{0};
    };

    /// \brief Properties of a top level model that drive its cost.
    struct PerformanceModelSample
    {
      /// \brief Name of the model
      std::string name;

      /// \brief Degrees of freedom. Zero for static models.
      unsigned int dof{0};

      /// \brief Number of collision shapes
      unsigned int collisions{0};

      /// \brief Triangles of the mesh collisions that move and are touching
      /// something, which are the ones checked triangle by triangle.
      uint64_t contactingMeshTriangles{0};

      /// \brief Number of visuals
      uint64_t visuals{0};

      /// \brief Triangles of the mesh visuals
      uint64_t visualTriangles{0};

      /// \brief Average number of contact points of its collisions
      double contacts{0.0};
    };

    /// \brief Everything measured in one report interval.
    struct PerformanceWindow
    {
      /// \brief Wall time of the interval
      std::chrono::steady_clock::duration wall{0};

      /// \brief Wall time that the simulation loop was idle, pacing itself to
      /// not run faster than the target real time factor
      std::chrono::steady_clock::duration sleep{0};

      /// \brief Simulation time that elapsed in the interval
      std::chrono::steady_clock::duration simTime{0};

      /// \brief Number of simulation steps
      uint64_t iterations{0};

      /// \brief Target real time factor, zero means as fast as possible
      double desiredRtf{1.0};

      /// \brief Wall time slot of a step at the target real time factor, zero
      /// if running as fast as possible
      std::chrono::steady_clock::duration period{0};

      /// \brief Part of the wall time lost to the schedule that was spent
      /// outside of any system, see PerformanceSystemSample::late
      std::chrono::steady_clock::duration lateLoop{0};

      /// \brief Sum of the render thread passes
      events::RenderFrameTimes render;

      /// \brief Number of render thread passes
      uint64_t renderFrames{0};

      /// \brief Systems that were updated
      std::vector<PerformanceSystemSample> systems;

      /// \brief Sensors that generated data
      std::vector<PerformanceSensorSample> sensors;

      /// \brief Top level models
      std::vector<PerformanceModelSample> models;
    };

    /// \brief The load attributed to a top level model.
    struct PerformanceModelLoad
    {
      /// \brief Name of the model
      std::string name;

      /// \brief Measured time of the systems attached to it
      std::chrono::steady_clock::duration plugins{0};

      /// \brief Measured time of its sensors
      std::chrono::steady_clock::duration sensors{0};

      /// \brief Estimated share of the physics step. The physics engines
      /// step the whole world at once so this can't be measured, it comes from
      /// the degrees of freedom, collision shapes and contacts of the model.
      std::chrono::steady_clock::duration physics{0};

      /// \brief Estimated share of the time that the render thread spends
      /// on the scene before drawing, based on its number of visuals.
      std::chrono::steady_clock::duration scene{0};

      /// \brief Number of visuals, which the scene estimate is based on
      uint64_t visuals{0};

      /// \brief Degrees of freedom, which the physics estimate is based on
      unsigned int dof{0u};

      /// \brief Average contact points, which the physics estimate is based on
      double contacts{0.0};

      /// \brief Triangles of its mesh visuals
      uint64_t visualTriangles{0};

      /// \brief Sum of plugins, sensors, physics and scene
      std::chrono::steady_clock::duration Total() const
      {
        return this->plugins + this->sensors + this->physics + this->scene;
      }
    };

    /// \brief Wall time that the steps lost to their schedule, split by where
    /// it went.
    struct PerformanceLateTime
    {
      /// \brief In the physics system
      std::chrono::steady_clock::duration physics{0};

      /// \brief In the sensors system: waiting for the render thread, or
      /// copying the entity changes for it
      std::chrono::steady_clock::duration rendering{0};

      /// \brief In the other systems
      std::chrono::steady_clock::duration otherSystems{0};

      /// \brief Outside of any system
      std::chrono::steady_clock::duration loop{0};

      /// \brief Sum of all
      std::chrono::steady_clock::duration Total() const
      {
        return this->physics + this->rendering + this->otherSystems +
            this->loop;
      }
    };

    /// \brief The outcome of analyzing an interval.
    struct GZ_SIM_VISIBLE PerformanceReport
    {
      /// \brief Real time factor achieved in the interval
      double rtf{0.0};

      /// \brief Target real time factor, zero means as fast as possible
      double desiredRtf{1.0};

      /// \brief True if the simulation could not reach its target
      bool limited{false};

      /// \brief Wall time of the interval
      std::chrono::steady_clock::duration wall{0};

      /// \brief Wall time that the simulation loop was not idle
      std::chrono::steady_clock::duration busy{0};

      /// \brief Number of simulation steps
      uint64_t iterations{0};

      /// \brief What limits the simulation
      PerformanceResource resource{PerformanceResource::kNone};

      /// \brief Where the limit is. It is filled in even if the simulation is
      /// not limited, to name its biggest consumer.
      PerformanceLocation location{PerformanceLocation::kNone};

      /// \brief Name of the sensor, system or model responsible. For physics
      /// it is an estimate, see PerformanceModelLoad::physics.
      std::string culprit;

      /// \brief Step time used by physics
      std::chrono::steady_clock::duration physics{0};

      /// \brief Step time spent waiting for the render thread
      std::chrono::steady_clock::duration renderWait{0};

      /// \brief Step time used by other systems
      std::chrono::steady_clock::duration otherSystems{0};

      /// \brief Step time outside of any system
      std::chrono::steady_clock::duration engine{0};

      /// \brief Wall time slot of a step at the target real time factor, zero
      /// if running as fast as possible
      std::chrono::steady_clock::duration period{0};

      /// \brief Wall time that the steps lost to their schedule. At a target
      /// real time factor this is what lowers the RTF, see
      /// PerformanceSchedule.
      PerformanceLateTime late;

      /// \brief True if the location was chosen by the time past the slot
      /// rather than by the step time
      bool rankedByLateTime{false};

      /// \brief Sum of the render thread passes
      events::RenderFrameTimes render;

      /// \brief Number of render thread passes
      uint64_t renderFrames{0};

      /// \brief Systems, biggest first
      std::vector<PerformanceSystemSample> systems;

      /// \brief Sensors, biggest first
      std::vector<PerformanceSensorSample> sensors;

      /// \brief Models, biggest load first
      std::vector<PerformanceModelLoad> models;

      /// \brief Short name of the resource
      /// \return Lower case name
      std::string ResourceName() const;

      /// \brief Short name of the location
      /// \return Lower case name
      std::string LocationName() const;

      /// \brief Write the report as text for the console.
      /// \return Multi line report
      std::string ToString() const;

      /// \brief Convert the report to a message. The verdict goes to the
      /// header data and the times to its time field.
      /// \return The message
      msgs::Diagnostics ToMsg() const;
    };

    /// \brief Find what limits the simulation in an interval.
    /// \param[in] _window Measurements of the interval
    /// \return The analysis
    GZ_SIM_VISIBLE
    PerformanceReport AnalyzePerformance(const PerformanceWindow &_window);

    /// \brief Collect the properties of all the top level models.
    /// \param[in] _ecm Entities of the simulation
    /// \param[in] _contacts Average contact points per collision
    /// \return The models
    GZ_SIM_VISIBLE
    std::vector<PerformanceModelSample> CollectPerformanceModels(
        const EntityComponentManager &_ecm,
        const std::unordered_map<Entity, double> &_contacts);

    /// \brief Measures where the wall time of the simulation goes, finds what
    /// limits the real time factor and reports it periodically.
    ///
    /// The simulation loop creates this object only if monitoring was
    /// requested, the cost otherwise is a null check per system update.
    /// \brief Finds the wall time that the steps of the simulation loop lose
    /// at a target real time factor, and who spent it.
    ///
    /// It follows the pacing of SimulationRunner::Run. The loop sleeps away
    /// the rest of the slot of a step that fits. A step that runs past its
    /// slot leaves the loop behind its schedule, and faster steps after it
    /// can make up for that. Only when the loop falls more than a slot behind
    /// does it start a new schedule from where it is, and the time that it
    /// was behind is lost for good. That time is split among the systems that
    /// ran past their slots, in proportion to what each one still owed.
    class GZ_SIM_VISIBLE PerformanceSchedule
    {
      /// \brief Time spent by each system interface, nullptr for the time
      /// outside of any system
      public: using Shares = std::vector<std::pair<const void *,
                  std::chrono::steady_clock::duration>>;

      /// \brief Add a step.
      /// \param[in] _busy Wall time of the step, from the end of the pacing
      /// of the previous one to the start of its own
      /// \param[in] _period Wall time slot of a step, zero if running as fast
      /// as possible
      /// \param[in] _shares Time of the step in each system interface. The
      /// rest of the step was outside of any system.
      /// \return The wall time lost in this step, split among who spent it
      public: Shares Step(std::chrono::steady_clock::duration _busy,
                  std::chrono::steady_clock::duration _period,
                  const Shares &_shares);

      /// \brief Forget the debt, as the loop does when it stops pacing.
      public: void Clear();

      /// \brief Get the time that the loop is behind its schedule.
      /// \return The time
      public: std::chrono::steady_clock::duration Debt() const;

      /// \brief Time that the loop is behind its schedule
      private: std::chrono::steady_clock::duration debt{0};

      /// \brief Part of the debt owed by each system interface, nullptr for
      /// outside of any system
      private: Shares owed;
    };

    class GZ_SIM_VISIBLE PerformanceMonitor
    {
      /// \brief Measures one update of one system, if there is a monitor.
      public: class SystemTimer
      {
        /// \brief Constructor. Starts timing.
        /// \param[in] _monitor The monitor, or nullptr to do nothing
        /// \param[in] _system Interface of the system being updated
        /// \param[in] _phase Which update it is
        public: SystemTimer(PerformanceMonitor *_monitor, const void *_system,
                    PerformanceSystemPhase _phase);

        /// \brief Destructor. Reports the time to the monitor.
        public: ~SystemTimer();

        /// \brief Not copyable
        public: SystemTimer(const SystemTimer &) = delete;

        /// \brief Not assignable
        public: SystemTimer &operator=(const SystemTimer &) = delete;

        /// \brief The monitor, may be null
        private: PerformanceMonitor *monitor;

        /// \brief Interface of the system
        private: const void *system;

        /// \brief Which update it is
        private: PerformanceSystemPhase phase;

        /// \brief Start of the update
        private: std::chrono::steady_clock::time_point start;

        /// \brief CPU time of the thread at the start. Negative if it is not
        /// being measured in this update.
        private: std::chrono::nanoseconds cpuStart{-1};
      };

      /// \brief Create a monitor, which starts collecting. What gets measured
      /// is process wide, so only one can exist at a time.
      /// \param[in] _reportPeriod Wall time between reports
      /// \param[in] _parallelPostUpdates Whether post updates run in parallel
      /// \return The monitor, null if another one exists
      public: static std::unique_ptr<PerformanceMonitor> Create(
                  std::chrono::steady_clock::duration _reportPeriod,
                  bool _parallelPostUpdates);

      /// \brief Destructor. Stops collecting.
      public: ~PerformanceMonitor();

      /// \brief Not copyable
      public: PerformanceMonitor(const PerformanceMonitor &) = delete;

      /// \brief Not assignable
      public: PerformanceMonitor &operator=(const PerformanceMonitor &) =
                  delete;

      /// \brief Must be called when all post updates of a step are done.
      public: void EndPostUpdate();

      /// \brief Must be called at the end of each simulation loop iteration.
      /// \param[in] _paceStart When the loop started to pace itself
      /// \param[in] _paceEnd When the loop finished pacing itself
      /// \param[in] _info Current simulation state
      /// \param[in] _desiredRtf Target real time factor
      public: void EndIteration(
                  const std::chrono::steady_clock::time_point &_paceStart,
                  const std::chrono::steady_clock::time_point &_paceEnd,
                  const UpdateInfo &_info, double _desiredRtf);

      /// \brief Tell the monitor that the simulation runs again after having
      /// been stopped. The time in between is not part of any interval.
      /// \param[in] _now Current wall time
      public: void StartRun(
                  const std::chrono::steady_clock::time_point &_now);

      /// \brief Check if the current interval is complete.
      /// \return True if a report should be taken
      public: bool ReportDue() const;

      /// \brief Analyze the current interval and start a new one.
      /// \param[in] _systemMgr Systems of the simulation
      /// \param[in] _ecm Entities of the simulation
      /// \return The report
      public: PerformanceReport TakeReport(const SystemManager &_systemMgr,
                  const EntityComponentManager &_ecm);

      /// \brief Add a pass of the render thread to the current interval. Can
      /// be called from any thread.
      /// \param[in] _times Times of the pass
      public: void OnRenderFrame(const events::RenderFrameTimes &_times);

      /// \brief Add a sample of contacts to the current interval. Can be
      /// called from any thread.
      /// \param[in] _contacts Contact points of each collision
      public: void OnContactSample(
                  const std::unordered_map<Entity, unsigned int> &_contacts);

      /// \brief Add one system update to the current interval.
      /// \param[in] _system Interface of the system
      /// \param[in] _phase Which update it is
      /// \param[in] _wall Wall time of the update
      /// \param[in] _cpu CPU time of the update, negative if not measured
      private: void Record(const void *_system, PerformanceSystemPhase _phase,
                  std::chrono::steady_clock::duration _wall,
                  std::chrono::nanoseconds _cpu);

      /// \brief Constructor. Starts collecting.
      /// \param[in] _reportPeriod Wall time between reports
      /// \param[in] _parallelPostUpdates Whether post updates run in parallel
      private: PerformanceMonitor(
                  std::chrono::steady_clock::duration _reportPeriod,
                  bool _parallelPostUpdates);

      /// \brief Discard what was collected and start a new interval.
      private: void Restart();

      /// \brief Start a new interval at the last iteration seen. Only resets
      /// what the simulation thread collects.
      private: void StartInterval();

      /// \brief Wall time between reports
      private: std::chrono::steady_clock::duration reportPeriod;

      /// \brief Whether post updates run in parallel
      private: bool parallelPostUpdates;

      /// \brief True during the iterations in which CPU time is measured.
      private: std::atomic<bool> sampleCpu{false};

      /// \brief Time accumulated for a system.
      private: struct SystemAccumulator
      {
        std::chrono::steady_clock::duration wall{0};
        std::chrono::steady_clock::duration critical{0};
        std::chrono::steady_clock::duration late{0};
        std::chrono::steady_clock::duration sampledWall{0};
        std::chrono::steady_clock::duration sampledCpu{0};
      };

      /// \brief Protects the accumulators, which are written from the post
      /// update threads and from the render thread.
      private: mutable std::mutex mutex;

      /// \brief Accumulated time per system interface
      private: std::unordered_map<const void *, SystemAccumulator> systems;

      /// \brief Post updates of the current step, used to find the slowest
      private: std::vector<std::pair<const void *,
                  std::chrono::steady_clock::duration>> stepPostUpdates;

      /// \brief Critical time of each system interface in the current step,
      /// used to split the time that the step puts the loop behind
      private: PerformanceSchedule::Shares stepCritical;

      /// \brief Follows the schedule of the simulation loop
      private: PerformanceSchedule schedule;

      /// \brief Time past the slot of the steps spent outside of any system
      private: std::chrono::steady_clock::duration lateLoop{0};

      /// \brief Wall time slot of a step at the target real time factor
      private: std::chrono::steady_clock::duration period{0};

      /// \brief Sum of the render thread passes
      private: events::RenderFrameTimes render;

      /// \brief Number of render thread passes
      private: uint64_t renderFrames{0};

      /// \brief Sum of the contact samples per collision
      private: std::unordered_map<Entity, uint64_t> contacts;

      /// \brief Number of contact samples
      private: uint64_t contactSamples{0};

      /// \brief Start of the current interval
      private: std::chrono::steady_clock::time_point windowStart;

      /// \brief Wall time of the last iteration seen
      private: std::chrono::steady_clock::time_point now;

      /// \brief Simulation time at the start of the current interval.
      private: std::chrono::steady_clock::duration windowSimStart{0};

      /// \brief Simulation time at the last iteration seen
      private: std::chrono::steady_clock::duration simTime{0};

      /// \brief Idle time of the current interval
      private: std::chrono::steady_clock::duration sleep{0};

      /// \brief Iterations of the current interval
      private: uint64_t iterations{0};

      /// \brief Target real time factor
      private: double desiredRtf{1.0};

      /// \brief True until the initial warm up, when the first frames of the
      /// sensors would distort the first report, is over.
      private: bool warmingUp{true};

      /// \brief True if the previous iteration was paused
      private: bool wasPaused{true};
    };
    }
  }  // namespace sim
}  // namespace gz
#endif  // GZ_SIM_PERFORMANCEMONITOR_HH_

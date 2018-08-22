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
#ifndef IGNITION_GAZEBO_COMPONENTS_WORLDSTATISTICS_TYPE_HH_
#define IGNITION_GAZEBO_COMPONENTS_WORLDSTATISTICS_TYPE_HH_

#include <memory>

#include <ignition/math/Stopwatch.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
namespace components
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declarations.
  class WorldStatisticsPrivate;

  /// \brief A component type that contains world statistic information.
  class IGNITION_GAZEBO_VISIBLE WorldStatistics
  {
    /// \brief Constructor
    public: explicit WorldStatistics();

    /// \brief Copy Constructor
    /// \param[in] _stats WorldStatistics to copy.
    public: WorldStatistics(const WorldStatistics &_stats);

    /// \brief Move Constructor
    /// \param[in] _stats WorldStatistics to move.
    public: WorldStatistics(WorldStatistics &&_stats) noexcept;

    /// \brief Destructor.
    public: virtual ~WorldStatistics();

    /// \brief Move assignment operator.
    /// \param[in] _stats WorldStatistics component to move.
    /// \return Reference to this.
    public: WorldStatistics &operator=(WorldStatistics &&_stats);

    /// \brief Copy assignment operator.
    /// \param[in] _stats WorldStatistics component to copy.
    /// \return Reference to this.
    public: WorldStatistics &operator=(const WorldStatistics &_stats);

    /// \brief Get the number of iterations.
    /// \return The elapsed number of iterations.
    public: uint64_t Iterations() const;

    /// \brief Set the total number of iterations.
    /// \param[in] _iters The elapsed number of iterations.
    public: void SetIterations(const uint64_t _iters);

    /// \brief Add iterations to the current total.
    /// \param[in] _iters Iterations to add.
    public: void AddIterations(const uint64_t _iters);

    /// \brief Get the elapsed sim time.
    /// \return The elapsed sim time.
    public: const ignition::math::clock::duration &SimTime() const;

    /// \brief Add sim time.
    /// \param[in] _sim The amount of time to add.
    public: void AddSimTime(const ignition::math::clock::duration &_sim);

    /// \brief Set sim time.
    /// \param[in] _sim The total sim time.
    public: void SetSimTime(const ignition::math::clock::duration &_sim);

    /// \brief Get the elapsed real time.
    /// \return The elapsed real time.
    public: const ignition::math::Stopwatch &RealTime() const;

    /// \brief Get a mutable real time.
    /// \return The elapsed real time.
    public: ignition::math::Stopwatch &RealTime();

    /// \brief Private data pointer.
    private: std::unique_ptr<WorldStatisticsPrivate> dataPtr;
  };
  }
}
}
}
#endif

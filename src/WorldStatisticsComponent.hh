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
#ifndef IGNITION_GAZEBO_WORLD_STATISTICS_COMPONENT_TYPE_HH_
#define IGNITION_GAZEBO_WORLD_STATISTICS_COMPONENT_TYPE_HH_

#include <memory>
#include <string>

#include <ignition/math/Stopwatch.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class WorldStatisticsComponentPrivate;

    /// \brief A component type that contains world statistic information.
    class IGNITION_GAZEBO_VISIBLE WorldStatisticsComponent
    {
      /// \brief Constructor
      /// \param[in] _compMgr The entity component manager, which is used to
      /// register the component type.
      public: explicit WorldStatisticsComponent();

      /// \brief Copy Constructor
      /// \param[in] _stats WorldStatisticsComponent to copy.
      public: WorldStatisticsComponent(
                  const WorldStatisticsComponent &_stats);

      /// \brief Move Constructor
      /// \param[in] _stats WorldStatisticsComponent to move.
      public: WorldStatisticsComponent(
                  WorldStatisticsComponent &&_stats) noexcept;

      /// \brief Destructor.
      public: virtual ~WorldStatisticsComponent();

      // Documentation inherited
      public: const std::string &Name() const;

      /// \brief Move assignment operator.
      /// \param[in] _stats WorldStatistics component to move.
      /// \return Reference to this.
      public: WorldStatisticsComponent &operator=(
                  WorldStatisticsComponent &&_stats);

      /// \brief Copy assignment operator.
      /// \param[in] _stats WorldStatistics component to copy.
      /// \return Reference to this.
      public: WorldStatisticsComponent &operator=(
                  const WorldStatisticsComponent &_stats);

      /// \brief Get the number of iterations.
      /// \return The elapsed real time.
      public: uint64_t Iterations() const;
      public: void SetIterations(const uint64_t _iters);
      public: void AddIterations(const uint64_t _iters);

      /// \brief Get the elapsed real time.
      /// \return The elapsed real time.
      public: const ignition::math::clock::duration &SimTime() const;

      public: void AddSimTime(const ignition::math::clock::duration &_sim);

      public: void SetSimTime(const ignition::math::clock::duration &_sim);

      /// \brief Get the elapsed real time.
      /// \return The elapsed real time.
      public: const ignition::math::Stopwatch &RealTime() const;

      public: ignition::math::Stopwatch &RealTime();

      public: void SetRealTime(
                  const ignition::math::clock::duration &_realTime) const;

      /// \brief Private data pointer.
      private: std::unique_ptr<WorldStatisticsComponentPrivate> dataPtr;
    };
    }
  }
}
#endif

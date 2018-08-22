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
#ifndef IGNITION_GAZEBO_WORLD_COMPONENT_TYPE_HH_
#define IGNITION_GAZEBO_WORLD_COMPONENT_TYPE_HH_

#include <memory>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace sdf
{
  class World;
}

namespace ignition
{
namespace gazebo
{
namespace components
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declarations.
  class WorldPrivate;

  /// \brief A component type that contains world statistic information.
  class IGNITION_GAZEBO_VISIBLE World
  {
    /// \brief Constructor
    /// \param[in] _world An SDF world element.
    public: explicit World(const sdf::World *_world);

    /// \brief Copy Constructor
    /// \param[in] _world World to copy.
    public: World(const World &_world);

    /// \brief Move Constructor
    /// \param[in] _world World to move.
    public: World(World &&_world) noexcept;

    /// \brief Destructor.
    public: virtual ~World();

    /// \brief Move assignment operator.
    /// \param[in] _world WorldStatistics component to move.
    /// \return Reference to this.
    public: World &operator=(World &&_world);

    /// \brief Copy assignment operator.
    /// \param[in] _world WorldStatistics component to copy.
    /// \return Reference to this.
    public: World &operator=(const World &_world);

    // Documentation inherited
    public: const std::string &Name() const;

    /// \brief Get the desired real time factor.
    /// \result The desired real time factor.
    public: double DesiredRealTimeFactor() const;

    /// \brief Set the desired real time factor.
    /// \param[in] _factor The new desired real time factor.
    public: void SetDesiredRealTimeFactor(const double _factor);

    /// \brief Get the max step duration.
    /// \return Physics max step duration.
    public: std::chrono::steady_clock::duration MaxStep() const;

    /// \brief Set the max step duration.
    /// \param[in] _step Physics max step duration.
    public: void SetMaxStep(const std::chrono::steady_clock::duration _step);

    /// \brief Private data pointer.
    private: std::unique_ptr<WorldPrivate> dataPtr;
  };
  }
}
}
}
#endif

/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef SIMULATION_DEFAULTS_HH_
#define SIMULATION_DEFAULTS_HH_

#include <memory>
#include <string>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/simulation/Clock.hh>

namespace gz::sim::simulation
{

/// \brief Structure to hold parts of the simulation that were not
/// explicitly initialized by the user of the builder interface.
struct DefaultSimulationInterfaces
{
  std::unique_ptr<gz::sim::EntityComponentManager> ecm {nullptr};

  std::unique_ptr<Clock> simClock {nullptr};

  std::unique_ptr<Clock> wallClock {nullptr};
};


DefaultSimulationInterfaces & defaultSimulationInterfaces();

/// \brief Get the default world as a string.
/// Plugins will be loaded from the server.config file.
/// \return An SDF string that contains the default world.
std::string defaultEmptyWorld();

}  // namespace gz::sim::simulation
//
#endif  // SIMULATION_DEFAULTS_HH_

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
#ifndef GZ_SIM_SIMULATIONBUILDER_HH_
#define GZ_SIM_SIMULATIONBUILDER_HH_

#include <gz/sim/simulation/Clock.hh>

#include <gz/sim/EntityComponentManager.hh>

#include <sdf/Root.hh>

#include <filesystem>
#include <memory>

namespace gz::sim::simulation
{

class Simulation;
class SimulationBuilderPrivate;

class SimulationBuilder
{
  public: SimulationBuilder();

  public: SimulationBuilder & EmptyWorld();

  public: SimulationBuilder & World(sdf::Root *_root);

  public: SimulationBuilder & World(const std::string &_sdfString);

  public: SimulationBuilder & World(const std::filesystem::path &_sdfFilename);

  public: SimulationBuilder & EntityComponentManager(gz::sim::EntityComponentManager *_ecm);

  public: SimulationBuilder & WallClock(Clock *_wallClock);

  public: SimulationBuilder & SimClock(Clock *_simClock);

  public: std::unique_ptr<Simulation> Build() const;

  /// \brief Private data pointer.
  GZ_UTILS_IMPL_PTR_FWD(SimulationBuilderPrivate, dataPtr)
};

}  // namespace gz::sim::simulation

#endif  // GZ_SIM_SIMULATIONBUILDER_HH_

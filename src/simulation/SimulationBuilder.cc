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

#include <gz/sim/simulation/Clock.hh>
#include <gz/sim/simulation/SimulationBuilder.hh>
#include <gz/sim/simulation/Simulation.hh>

#include "SimulationDefaults.hh"
#include "SimulationPrivate.hh"
#include "SimulationBuilderPrivate.hh"

#include <gz/utils/ImplPtr.hh>

namespace gz::sim::simulation
{

SimulationBuilder::SimulationBuilder()
  : dataPtr(utils::MakeImpl<SimulationBuilderPrivate>())
{
}

SimulationBuilder & SimulationBuilder::World(sdf::Root *_root)
{
  this->dataPtr->worldInfo.source = WorldSource::kSdfRoot;
  this->dataPtr->worldInfo.sdfRoot = _root;
  return *this;
}

SimulationBuilder & SimulationBuilder::World(const std::string &_sdfString)
{
  this->dataPtr->worldInfo.source = WorldSource::kSdfString;
  this->dataPtr->worldInfo.sdfString = _sdfString;
  return *this;
}

SimulationBuilder & SimulationBuilder::World(const std::filesystem::path &_sdfPath)
{
  this->dataPtr->worldInfo.source = WorldSource::kSdfFilename;
  this->dataPtr->worldInfo.sdfPath= _sdfPath;
  return *this;
}

SimulationBuilder & SimulationBuilder::EntityComponentManager(gz::sim::EntityComponentManager *_ecm)
{
  this->dataPtr->ecm = _ecm;
  return *this;
}

SimulationBuilder & SimulationBuilder::WallClock(Clock *_wallClock)
{
  this->dataPtr->wallClock = _wallClock;
  return *this;
}

SimulationBuilder & SimulationBuilder::SimClock(Clock *_simClock)
{
  this->dataPtr->simClock= _simClock;
  return *this;
}

std::unique_ptr<Simulation> SimulationBuilder::Build() const
{
  std::unique_ptr<Simulation> ret {new Simulation};
  auto worldRoot = this->dataPtr->worldInfo.Create();

  if (this->dataPtr->ecm)
  {
    ret->dataPtr->ecm = this->dataPtr->ecm;
  }
  else if(!defaultSimulationInterfaces().ecm)
  {
    defaultSimulationInterfaces().ecm = std::make_unique<gz::sim::EntityComponentManager>();
    ret->dataPtr->ecm = defaultSimulationInterfaces().ecm.get();
  }
  else
  {
    return nullptr;
  }

  if (this->dataPtr->simClock)
  {
    ret->dataPtr->simClock = this->dataPtr->simClock;
  }
  else if(!defaultSimulationInterfaces().simClock)
  {
    defaultSimulationInterfaces().simClock = std::make_unique<Clock>();
    ret->dataPtr->simClock = defaultSimulationInterfaces().simClock.get();
  }
  else
  {
    return nullptr;
  }

  if (this->dataPtr->wallClock)
  {
    ret->dataPtr->wallClock= this->dataPtr->wallClock;
  }
  else if (!defaultSimulationInterfaces().wallClock)
  {
    defaultSimulationInterfaces().wallClock = std::make_unique<Clock>();
    ret->dataPtr->wallClock = defaultSimulationInterfaces().wallClock.get();
  }
  else
  {
    return nullptr;
  }

  return ret;
}

}  // namespace gz::sim::simulation

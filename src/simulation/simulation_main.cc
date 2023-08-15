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
#include <gz/sim/simulation/Simulation.hh>
#include <gz/sim/simulation/SimulationBuilder.hh>

using SimulationBuilder = gz::sim::simulation::SimulationBuilder;

void CreateEmptyWorld()
{
  auto sim = SimulationBuilder()
    .EmptyWorld()
    .Build();
}

void CreateWorldFromFile()
{
  auto sim = SimulationBuilder()
    .World(std::string("my_simple_world.sdf"))
    .Build();
}

void DependencyInjection()
{
  auto simClock = gz::sim::simulation::Clock(gz::sim::simulation::ClockType::kSimTime);
  auto wallClock = gz::sim::simulation::Clock(gz::sim::simulation::ClockType::kSystemTime);
  auto ecm = gz::sim::EntityComponentManager();

  auto sim = gz::sim::simulation::SimulationBuilder()
    .WallClock(&wallClock)
    .SimClock(&simClock)
    .EntityComponentManager(&ecm)
    .Build();

  for (size_t ii = 0; ii < 100; ++ii)
  {
    std::cout << wallClock.Now().Seconds() << " " << simClock.Now().Seconds() << std::endl;
    std::cout << sim.IterationCount() << std::endl;

    sim.Step();
  }
}

int main(int argc, char **argv)
{

  CreateEmptyWorld();

  CreateWorldFromFile();

  DependencyInjection();
}

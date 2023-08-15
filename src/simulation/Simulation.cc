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

#include <gz/sim/simulation/Simulation.hh>

namespace gz::sim::simulation
{

Simulation::Simulation() = default;

void Simulation::Reset()
{
}

void Simulation::Step()
{
}

std::string Simulation::WorldName() const
{
  return "";
}

std::size_t Simulation::SystemCount() const
{
  return 0;
}

std::size_t Simulation::EntityCount() const
{
  return 0;
}

std::size_t Simulation::IterationCount() const
{
  return 0;
}

}

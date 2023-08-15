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
#ifndef GZ_SIM_SIMULATION_HH_
#define GZ_SIM_SIMULATION_HH_

#include <cstdint>

#include <gz/sim/simulation/SimulationBuilder.hh>

namespace gz::sim::simulation
{

class Simulation;
class SimulationPrivate;
class Model;
class Link;
class Joint;

class Joint
{
  public: std::string Name() const;

  public: friend Model;

  private: Joint();
};


class Link
{
  public: std::string Name() const;

  public: friend Model;

  private: Link();
};

class Model
{
  public: std::string Name() const;

  public: Link LinkByName(const std::string &_linkName);

  public: Joint JointByName(const std::string &_jointName);

  public: friend Simulation;

  private: Model();
};


class Simulation
{
  public: void Reset();

  public: void Step();


  public: Model ModelByName(const std::string &_modelName);

  public: [[nodiscard]] std::string WorldName() const;

  public: [[nodiscard]] std::size_t SystemCount() const;

  public: [[nodiscard]] std::size_t EntityCount() const;

  public: [[nodiscard]] std::size_t IterationCount() const;

  public: friend SimulationBuilder;

  private: Simulation();

  /// \brief Private data pointer.
  GZ_UTILS_IMPL_PTR_FWD(SimulationPrivate, dataPtr)
};

}  // namespace gz::sim::simulation

#endif  // GZ_SIM_SIMULATION_HH_

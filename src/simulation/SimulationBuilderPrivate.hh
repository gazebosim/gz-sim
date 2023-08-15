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

#ifndef SIMULATIONBUILDERPRIVATE_HH_
#define SIMULATIONBUILDERPRIVATE_HH_

#include <sdf/Root.hh>

#include <filesystem>
#include <string>

#include <gz/fuel_tools/ClientConfig.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/simulation/Clock.hh>

namespace gz::sim::simulation
{

enum class WorldSource
{
  kEmptyWorld,
  kSdfRoot,
  kSdfFilename,
  kSdfString
};

struct WorldInfo
{
  WorldSource source {WorldSource::kEmptyWorld};

  sdf::Root *sdfRoot {nullptr};

  std::string sdfString {""};

  std::filesystem::path sdfPath {""};

  gz::fuel_tools::ClientConfig clientConfig;

  std::function<std::string(const std::string&)> fetchResourceCb;

  std::function<std::string(const gz::common::URI&)> fetchResourceUriCb;

  sdf::Root Create() const;
};

class SimulationBuilderPrivate
{
  public: WorldInfo worldInfo;

  public: gz::sim::EntityComponentManager *ecm {nullptr};

  public: Clock *wallClock {nullptr};

  public: Clock *simClock {nullptr};
};

}  // namespace gz::sim::simulation

#endif  // SIMULATIONBUILDERPRIVATE_HH_

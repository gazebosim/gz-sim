/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_LOCKSTEP_RUNTIME_CONFIG_HH_
#define GZ_SIM_SYSTEMS_LOCKSTEP_RUNTIME_CONFIG_HH_

#include <string>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
#include <sdf/sdf.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {

struct RuntimeConfig {
  /// \brief Service to advertise for Configure call from server LockStep System
  std::string configureService = "/Configure";

  /// \brief Service to advertise for PreUpdate call from server LockStep System
  std::string preupdateService = "/PreUpdate";

  /// \brief Topic on which stats should be advertised
  std::string statsTopic = "";

  /// \brief SDF plugin
  sdf::Plugin plugin;
};

/// \brief Parse config from file.
RuntimeConfig
GZ_SIM_VISIBLE
 parseRuntimeConfig(const std::string& _filename);

}
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_LOCKSTEP_RUNTIME_CONFIG_HH_
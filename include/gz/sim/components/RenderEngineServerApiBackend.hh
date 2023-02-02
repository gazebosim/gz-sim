/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_RENDERENGINESERVERAPIBACKEND_HH_
#define GZ_SIM_COMPONENTS_RENDERENGINESERVERAPIBACKEND_HH_

#include <string>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief Holds the render engine server API backend name.
  using RenderEngineServerApiBackend = Component<std::string,
      class RenderEngineServerApiBackendTag, serializers::StringSerializer>;
  GZ_SIM_REGISTER_COMPONENT(
      "gz_sim_components.RenderEngineServerApiBackend",
      RenderEngineServerApiBackend)
}
}
}
}

#endif

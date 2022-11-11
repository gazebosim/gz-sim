/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_EXTERNALWORLDWRENCHCMD_HH_
#define GZ_SIM_COMPONENTS_EXTERNALWORLDWRENCHCMD_HH_

#include <gz/msgs/wrench.pb.h>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
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
  /// \brief A component type that contains the external wrench to be applied on
  /// an entity expressed in the world frame and represented by
  /// gz::msgs::Wrench.
  /// Currently this is used for applying wrenches on links. Although the
  /// msg::Wrench type has a force_offset member, the value is currently
  /// ignored. Instead, the force is applied at the link origin.
  /// The wrench uses SI units (N for force and N⋅m for torque).
  using ExternalWorldWrenchCmd =
      Component<msgs::Wrench, class ExternalWorldWrenchCmdTag,
      serializers::MsgSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.ExternalWorldWrenchCmd",
                                ExternalWorldWrenchCmd)
}
}
}
}

#endif

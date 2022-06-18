/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_LIGHT_HH_
#define GZ_SIM_COMPONENTS_LIGHT_HH_

#include <gz/msgs/light.pb.h>

#include <sdf/Light.hh>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace serializers
{
  using LightSerializer =
      serializers::ComponentToMsgSerializer<sdf::Light, msgs::Light>;
}

namespace components
{
  /// \brief This component contains light source information. For more
  /// information on lights, see [SDF's Light
  /// element](http://sdformat.org/spec?ver=1.6&elem=light).
  using Light =
      Component<sdf::Light, class LightTag, serializers::LightSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.Light", Light)
}
}
}
}

#endif

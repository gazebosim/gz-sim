/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_AxisAlignedBox_HH_
#define GZ_SIM_COMPONENTS_AxisAlignedBox_HH_

#include <gz/msgs/axis_aligned_box.pb.h>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Conversions.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace serializers
{
  using AxisAlignedBoxSerializer =
      serializers::ComponentToMsgSerializer<math::AxisAlignedBox,
      msgs::AxisAlignedBox>;
}

namespace components
{
  /// \brief A component type that contains axis aligned box,
  /// gz::math::AxisAlignedBox, information.
  /// The axis aligned box is created from collisions in the entity
  using AxisAlignedBox = Component<gz::math::AxisAlignedBox,
      class AxisAlignedBoxTag, serializers::AxisAlignedBoxSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.AxisAlignedBox",
      AxisAlignedBox)
}
}
}
}

#endif

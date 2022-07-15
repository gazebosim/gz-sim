/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_JOINTTRANSMITTEDWRENCH_HH_
#define GZ_SIM_COMPONENTS_JOINTTRANSMITTEDWRENCH_HH_

#include <gz/msgs/wrench.pb.h>

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
/// \brief Joint Transmitted wrench in SI units (Nm for torque, N for force).
/// The wrench is expressed in the Joint frame and the force component is
/// applied at the joint origin.
/// \note The term Wrench is used here to mean a pair of 3D vectors representing
/// torque and force quantities expressed in a given frame and where the force
/// is applied at the origin of the frame. This is different from the Wrench
/// used in screw theory.
/// \note The value of force_offset in msgs::Wrench is ignored for this
/// component. The force is assumed to be applied at the origin of the joint
/// frame.
using JointTransmittedWrench =
    Component<msgs::Wrench, class JointTransmittedWrenchTag,
              serializers::MsgSerializer>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.JointTransmittedWrench",
                              JointTransmittedWrench)
}  // namespace components
}
}
}

#endif

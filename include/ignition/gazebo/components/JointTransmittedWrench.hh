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
#ifndef IGNITION_GAZEBO_COMPONENTS_JOINTTRANSMITTEDWRENCH_HH_
#define IGNITION_GAZEBO_COMPONENTS_JOINTTRANSMITTEDWRENCH_HH_

#include <ignition/msgs/wrench.pb.h>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

namespace components
{
/// \brief Joint Transmitted wrench in SI units (Nm for torque, N for force).
/// The wrench is expressed in the Joint frame and the force component is
/// applied at the joint origin.
/// \note The term Wrench is used here to mean a pair of 3D vectors representing
/// torque and force quantities expessed in a given frame and where the force is
/// applied at the origin of the frame. This is different from the Wrench used
/// in screw theory.
/// \note The value of force_offset in msgs::Wrench is ignored for this
/// component. The force is assumed to be applied at the origin of the joint
/// frame.
using JointTransmittedWrench =
    Component<msgs::Wrench, class JointTransmittedWrenchTag,
              serializers::MsgSerializer>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.JointTransmittedWrench",
                              JointTransmittedWrench)
}  // namespace components
}
}
}

#endif

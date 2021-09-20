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
#ifndef IGNITION_GAZEBO_COMPONENTS_JOINTCONSTRAINTWRENCH_HH_
#define IGNITION_GAZEBO_COMPONENTS_JOINTCONSTRAINTWRENCH_HH_

#include <array>

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
/// \brief Joint Constraint wrench in SI units (Nm for revolute, N for
/// prismatic). The first set of 3 values coorespond to the torque while the
/// second set of 3 correspond to the force. The wrench is expressed in the
/// Joint frame and the force component is applied at the joint origin.
/// \note The term Wrench is used here to mean a 6D vector formed by stacking
/// torque and force vectors. This is different from the Wrench used in screw
/// theory.
using JointConstraintWrench =
    Component<std::array<double, 6>, class JointConstraintWrenchTag,
              serializers::ArrayDoubleSerializer<6>>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.JointConstraintWrench",
                              JointConstraintWrench)
}
}
}
}

#endif

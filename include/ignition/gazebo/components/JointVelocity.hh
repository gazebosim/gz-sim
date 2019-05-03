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
#ifndef IGNITION_GAZEBO_COMPONENTS_JOINTVELOCITY_HH_
#define IGNITION_GAZEBO_COMPONENTS_JOINTVELOCITY_HH_

#include <vector>

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief Base class which can be extended to add serialization
  using JointVelocity = Component<std::vector<double>,
        class JointVelocityTag>;
  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.JointVelocity", JointVelocity)

  /// \brief Velocity of a joint's second axis in SI units (rad/s for revolute,
  /// m/s for prismatic).
  /// \deprecated Use JointVelocity.
  using JointVelocity2 IGN_DEPRECATED(2) =
  Component<double, class JointVelocity2Tag>;
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.JointVelocity2", JointVelocity2)
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

}
}
}
}

#endif

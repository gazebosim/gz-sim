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
#ifndef IGNITION_GAZEBO_COMPONENTS_MAGNETICFIELD_HH_
#define IGNITION_GAZEBO_COMPONENTS_MAGNETICFIELD_HH_

#include <ignition/math/Vector3.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include <ignition/gazebo/components/Factory.hh>
#include "ignition/gazebo/components/Component.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief Stores the 3D magnetic field in teslas.
  using MagneticField = Component<math::Vector3d, class MagneticFieldTag>;
  IGN_GAZEBO_REGISTER_COMPONENT(
      "ign_gazebo_components.MagneticField", MagneticField)
}
}
}
}
#endif

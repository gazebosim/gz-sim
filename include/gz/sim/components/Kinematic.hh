/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_KINEMATIC_HH_
#define GZ_SIM_COMPONENTS_KINEMATIC_HH_

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief A component that holds whether a link is kinematic. A kinematic
  /// link does not respond to forces, e.g. gravity or contacts, and is moved
  /// by setting its pose or velocity.
  ///
  /// SdfEntityCreator creates this component only for links whose SDF sets
  /// `//link/kinematic`, so that the Physics system can apply its default
  /// kinematic state (`<kinematic><enabled>`) to the links that do not.
  using Kinematic = Component<bool, class KinematicTag>;
  GZ_SIM_DECLARE_COMPONENT("gz_sim_components.Kinematic", Kinematic)
}
}
}
}

#endif

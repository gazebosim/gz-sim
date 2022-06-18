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
#ifndef GZ_SIM_SYSTEMS_NAVSAT_HH_
#define GZ_SIM_SYSTEMS_NAVSAT_HH_

#include <gz/utils/ImplPtr.hh>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \class NavSat NavSat.hh gz/sim/systems/NavSat.hh
  /// \brief System that handles navigation satellite sensors, such as GPS,
  /// that reports position and velocity in spherical coordinates (latitude /
  /// longitude) over Gazebo Transport.
  ///
  /// The NavSat sensors rely on the world origin's spherical coordinates
  /// being set, for example through SDF's `<spherical_coordinates>` tag
  /// or the `/world/world_name/set_spherical_coordinates` service.
  class NavSat:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit NavSat();

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;


    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
  }
}
}
}
#endif

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
#ifndef GZ_SIM_SYSTEMS_FOLLOWACTOR_HH_
#define GZ_SIM_SYSTEMS_FOLLOWACTOR_HH_

#include <memory>
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
  // Forward declarations.
  class FollowActorPrivate;

  /// \class FollowActor FollowActor.hh gz/sim/systems/FollowActor.hh
  /// \brief Make an actor follow a target entity in the world.
  ///
  /// ## System Parameters
  ///
  /// - `<target>`: Name of entity to follow.
  ///
  /// - `<min_distance>`: Distance in meters to keep from target's origin.
  ///
  /// - `<max_distance>`: Distance in meters from target's origin when to stop
  ///   following. When the actor is back within range it starts following
  ///   again.
  ///
  /// - `<velocity>`: Actor's velocity in m/s
  ///
  /// - `<animation>`: Actor's animation to play. If empty, the first animation
  ///   in the model will be used.
  ///
  /// - `<animation_x_vel>`: Velocity of the animation on the X axis. Used to
  ///   coordinate translational motion with the actor's animation.
  class FollowActor:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    /// \brief Constructor
    public: explicit FollowActor();

    /// \brief Destructor
    public: ~FollowActor() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<FollowActorPrivate> dataPtr;
  };
  }
}
}
}
#endif

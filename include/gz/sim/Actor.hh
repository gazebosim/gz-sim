/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#ifndef GZ_SIM_ACTOR_HH_
#define GZ_SIM_ACTOR_HH_

#include <memory>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>

#include <gz/utils/ImplPtr.hh>

#include "gz/sim/config.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/Types.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    //
    /// \class Actor Actor.hh gz/sim/Actor.hh
    /// \brief This class provides wrappers around entities and components
    /// which are more convenient and straight-forward to use than dealing
    /// with the `EntityComponentManager` directly.
    /// All the functions provided here are meant to be used with a actor
    /// entity.
    ///
    /// For example, given an actor's entity, find the value of its
    /// name component, one could use the entity-component manager (`ecm`)
    /// directly as follows:
    ///
    ///     std::string name = ecm.Component<components::Name>(entity)->Data();
    ///
    /// Using this class however, the same information can be obtained with
    /// a simpler function call:
    ///
    ///    Actor actor(entity);
    ///    std::string name = actor.Name(ecm);
    ///
    class GZ_SIM_VISIBLE Actor
    {
      /// \brief Constructor
      /// \param[in] _entity Actor entity
      public: explicit Actor(sim::Entity _entity = kNullEntity);

      /// \brief Get the entity which this Actor is related to.
      /// \return Actor entity.
      public: sim::Entity Entity() const;

      /// \brief Reset Entity to a new one
      /// \param[in] _newEntity New actor entity.
      public: void ResetEntity(sim::Entity _newEntity);

      /// \brief Check whether this actor correctly refers to an entity that
      /// has a components::Actor.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if it's a valid actor in the manager.
      public: bool Valid(const EntityComponentManager &_ecm) const;

      /// \brief Get the actor's unscoped name.
      /// \param[in] _ecm Entity-component manager.
      /// \return Actor's name or nullopt if the entity does not have a
      /// components::Name component
      public: std::optional<std::string> Name(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the pose of the actor.
      /// If the actor has a trajectory, this will only return the origin
      /// pose of the trajectory and not the actual world pose of the actor.
      /// \param[in] _ecm Entity-component manager.
      /// \return Pose of the actor or nullopt if the entity does not
      /// have a components::Pose component.
      /// \sa WorldPose
      public: std::optional<math::Pose3d> Pose(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the trajectory pose of the actor. There are two
      /// ways that the actor can follow a trajectory: 1) SDF script,
      /// 2) manually setting trajectory pose. This function retrieves 2) the
      /// manual trajectory pose set by the user. The Trajectory pose is
      /// given relative to the trajectory pose origin returned by Pose().
      /// \param[in] _ecm Entity Component manager.
      /// \return Trajectory pose of the actor w.r.t. to trajectory origin.
      /// \sa Pose
      public: std::optional<math::Pose3d> TrajectoryPose(
          const EntityComponentManager &_ecm) const;

      /// \brief Set the trajectory pose of the actor. There are two
      /// ways that the actor can follow a trajectory: 1) SDF script,
      /// 2) manually setting trajectory pose. This function enables option 2).
      /// Manually setting the trajectory pose will override the scripted
      /// trajectory specified in SDF.
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _pose Trajectory pose w.r.t. to the trajectory origin
      /// \sa Pose
      public: void SetTrajectoryPose(EntityComponentManager &_ecm,
          const math::Pose3d &_pose);

      /// \brief Get the world pose of the actor.
      /// This returns the current world pose of the actor computed by gazebo.
      /// The world pose is the combination of the actor's pose and its
      /// trajectory pose. The currently trajectory pose is either manually set
      /// via SetTrajectoryPose or interpolated from waypoints in the SDF script
      /// based on the current time.
      /// \param[in] _ecm Entity-component manager.
      /// \return World pose of the actor or nullopt if the entity does not
      /// have a components::WorldPose component.
      /// \sa Pose
      public: std::optional<math::Pose3d> WorldPose(
          const EntityComponentManager &_ecm) const;

      /// \brief Set the name of animation to use for this actor.
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _name Animation name
      public: void SetAnimationName(EntityComponentManager &_ecm,
          const std::string &_name);

      /// \brief Set the time of animation for this actor.
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _time Animation time
      public: void SetAnimationTime(EntityComponentManager &_ecm,
          const std::chrono::steady_clock::duration &_time);

     /// \brief Get the name of animation used by the actor
      /// \param[in] _ecm Entity-component manager.
      /// \return Animation name
      public: std::optional<std::string> AnimationName(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the time of animation for this actor
      /// \param[in] _ecm Entity-component manager.
      /// \return Animation time
      public: std::optional<std::chrono::steady_clock::duration> AnimationTime(
          const EntityComponentManager &_ecm) const;

      /// \brief Private data pointer.
      GZ_UTILS_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif

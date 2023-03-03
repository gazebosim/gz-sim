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
#ifndef IGNITION_GAZEBO_ACTOR_HH_
#define IGNITION_GAZEBO_ACTOR_HH_

#include <memory>
#include <optional>
#include <string>

#include <ignition/math/Pose3.hh>

#include <gz/utils/ImplPtr.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    /// \class Actor Actor.hh ignition/gazebo/Actor.hh
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
    class IGNITION_GAZEBO_VISIBLE Actor
    {
      /// \brief Constructor
      /// \param[in] _entity Actor entity
      public: explicit Actor(gazebo::Entity _entity = kNullEntity);

      /// \brief Get the entity which this Actor is related to.
      /// \return Actor entity.
      public: gazebo::Entity Entity() const;

      /// \brief Reset Entity to a new one
      /// \param[in] _newEntity New actor entity.
      public: void ResetEntity(gazebo::Entity _newEntity);

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

      /// \brief Get the pose of the actor
      /// \param[in] _ecm Entity-component manager.
      /// \return Pose of the actor or nullopt if the entity does not
      /// have a components::Pose component.
      public: std::optional<math::Pose3d> Pose(
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
      IGN_UTILS_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif

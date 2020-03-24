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
#ifndef IGNITION_GAZEBO_LINK_HH_
#define IGNITION_GAZEBO_LINK_HH_

#include <memory>
#include <optional>
#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class IGNITION_GAZEBO_HIDDEN LinkPrivate;
    //
    /// \class Link Link.hh ignition/gazebo/Link.hh
    /// \brief This class provides wrappers around entities and components
    /// which are more convenient and straight-forward to use than dealing
    /// with the `EntityComponentManager` directly.
    /// All the functions provided here are meant to be used with a link
    /// entity.
    ///
    /// For example, given a link's entity, to find the value of its
    /// name component, one could use the entity-component manager (`ecm`)
    /// directly as follows:
    ///
    ///     std::string name = ecm.Component<components::Name>(entity)->Data();
    ///
    /// Using this class however, the same information can be obtained with
    /// a simpler function call:
    ///
    ///    Link link(entity);
    ///    std::string name = link.Name(ecm);
    ///
    class IGNITION_GAZEBO_VISIBLE Link
    {
      /// \brief Constructor
      /// \param[in] _entity Link entity
      public: explicit Link(gazebo::Entity _entity = kNullEntity);

      /// \brief Copy constructor
      /// \param[in] _link Link to copy.
      public: Link(const Link &_link);

      /// \brief Move constructor
      /// \param[in] _link Link to move.
      public: Link(Link &&_link) noexcept;

      /// \brief Move assignment operator.
      /// \param[in] _link Link component to move.
      /// \return Reference to this.
      public: Link &operator=(Link &&_link) noexcept;

      /// \brief Copy assignment operator.
      /// \param[in] _link Link to copy.
      /// \return Reference to this.
      public: Link &operator=(const Link &_link);

      /// \brief Destructor
      public: ~Link();

      /// \brief Get the entity which this Link is related to.
      /// \return Link entity.
      public: gazebo::Entity Entity() const;

      /// \brief Reset Entity to a new one
      /// \param[in] _newEntity New link entity.
      public: void ResetEntity(gazebo::Entity _newEntity);

      /// \brief Check whether this link correctly refers to an entity that
      /// has a components::Link.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if it's a valid link in the manager.
      public: bool Valid(const EntityComponentManager &_ecm) const;

      /// \brief Get the link's unscoped name.
      /// \param[in] _ecm Entity-component manager.
      /// \return Link's name or nullopt if the entity does not have a
      /// components::Name component
      public: std::optional<std::string> Name(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the parent model
      /// \param[in] _ecm Entity-component manager.
      /// \return Parent Model or nullopt if the entity does not have a
      /// components::ParentEntity component.
      public: std::optional<Model> ParentModel(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the pose of the link frame in the world coordinate frame.
      /// \param[in] _ecm Entity-component manager.
      /// \return Absolute Pose of the link or nullopt if the entity does not
      /// have a components::WorldPose component.
      public: std::optional<math::Pose3d> WorldPose(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the world pose of the link inertia.
      /// \param[in] _ecm Entity-component manager.
      /// \return Inertial pose in world frame or nullopt if the
      /// link does not have the components components::WorldPose and
      /// components::Inertial.
      public: std::optional<math::Pose3d> WorldInertialPose(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the linear velocity at the origin of of the link frame
      /// expressed in the world frame, using an offset expressed in a
      /// body-fixed frame. If no offset is given, the velocity at the origin of
      /// the Link frame will be returned.
      /// \param[in] _ecm Entity-component manager.
      /// \return Linear velocity of the link or nullopt if the link does not
      /// have components components::WorldPose.
      public: std::optional<math::Vector3d> WorldLinearVelocity(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the linear velocity of a point on the body in the world
      /// frame, using an offset expressed in a body-fixed frame.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _offset Offset of the point from the origin of the Link
      /// frame, expressed in the body-fixed frame.
      /// \return Linear velocity of the point on the body or nullopt if the
      /// link does not have components components::WorldPose,
      /// components::WorldLinearVelocity and components::WorldAngularVelocity.
      public: std::optional<math::Vector3d> WorldLinearVelocity(
          const EntityComponentManager &_ecm,
          const math::Vector3d &_offset) const;

      /// \brief Get the angular velocity of the link in the world frame
      /// \param[in] _ecm Entity-component manager.
      /// \return Angular velocity of the link or nullopt if the link does not
      /// have a components::WorldAngularVelocity component.
      public: std::optional<math::Vector3d> WorldAngularVelocity(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the linear acceleration of the body in the world frame.
      /// \param[in] _ecm Entity-component manager.
      /// \return Linear acceleration of the body in the world frame or nullopt
      /// if the link does not have a components::WorldLinearAcceleration
      /// component.
      public: std::optional<math::Vector3d> WorldLinearAcceleration(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the inertia matrix in the world frame.
      /// \param[in] _ecm Entity-component manager.
      /// \return Inertia matrix in world frame, returns nullopt if link
      /// does not have components components::Inertial and
      /// components::WorldPose.
      public: std::optional<math::Matrix3d> WorldInertiaMatrix(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the rotational and translational kinetic energy of the
      /// link with respect to the world frame.
      /// \param[in] _ecm Entity-component manager.
      /// \return Kinetic energy in world frame, returns nullopt if link
      /// does not have components components::Inertial,
      /// components::WorldAngularVelocity, components::WorldLinearVelocity,
      /// and components::WorldPose.
      public: std::optional<double> WorldKineticEnergy(
          const EntityComponentManager &_ecm) const;

      /// \brief Add a force expressed in world coordinates and applied at the
      /// center of mass of the link.
      /// \param[in] _ecm Mutable Entity-component manager.
      /// \param[in] _force Force to be applied expressed in world coordinates
      public: void AddWorldForce(EntityComponentManager &_ecm,
                                 const math::Vector3d &_force) const;

      /// \brief Add a wrench expressed in world coordinates and applied to
      /// the link at the link's origin. This wrench is applied for one
      /// simulation step.
      /// \param[in] _ecm Mutable Entity-component manager.
      /// \param[in] _force Force to be applied expressed in world coordinates
      /// \param[in] _torque Torque to be applied expressed in world coordinates
      public: void AddWorldWrench(EntityComponentManager &_ecm,
                                 const math::Vector3d &_force,
                                 const math::Vector3d &_torque) const;

      /// \brief Pointer to private data.
      private: std::unique_ptr<LinkPrivate> dataPtr;
    };
    }
  }
}
#endif

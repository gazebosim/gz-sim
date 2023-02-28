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
#ifndef IGNITION_GAZEBO_JOINT_HH_
#define IGNITION_GAZEBO_JOINT_HH_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Export.hh>
// #include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/utils/ImplPtr.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class IGNITION_GAZEBO_HIDDEN JointPrivate;
    //
    /// \class Joint Joint.hh ignition/gazebo/Joint.hh
    /// \brief This class provides wrappers around entities and components
    /// which are more convenient and straight-forward to use than dealing
    /// with the `EntityComponentManager` directly.
    /// All the functions provided here are meant to be used with a joint
    /// entity.
    ///
    /// For example, given a joint's entity, find the value of its
    /// name component, one could use the entity-component manager (`ecm`)
    /// directly as follows:
    ///
    ///     std::string name = ecm.Component<components::Name>(entity)->Data();
    ///
    /// Using this class however, the same information can be obtained with
    /// a simpler function call:
    ///
    ///    Joint joint(entity);
    ///    std::string name = joint.Name(ecm);
    ///
    class IGNITION_GAZEBO_VISIBLE Joint
    {
      /// \brief Constructor
      /// \param[in] _entity Joint entity
      public: explicit Joint(gazebo::Entity _entity = kNullEntity);

      /// \brief Destructor
      public: ~Joint();

      /// \brief Get the entity which this Joint is related to.
      /// \return Joint entity.
      public: gazebo::Entity Entity() const;

      /// \brief Reset Entity to a new one
      /// \param[in] _newEntity New joint entity.
      public: void ResetEntity(gazebo::Entity _newEntity);

      /// \brief Check whether this joint correctly refers to an entity that
      /// has a components::Joint.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if it's a valid joint in the manager.
      public: bool Valid(const EntityComponentManager &_ecm) const;

      /// \brief Get the joint's unscoped name.
      /// \param[in] _ecm Entity-component manager.
      /// \return Joint's name or nullopt if the entity does not have a
      /// components::Name component
      public: std::optional<std::string> Name(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the parent link name
      /// \param[in] _ecm Entity-component manager.
      /// \return Parent link name or nullopt if the entity does not have a
      /// components::ParentLinkName component.
      public: std::optional<std::string> ParentLinkName(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the child link name
      /// \param[in] _ecm Entity-component manager.
      /// \return Child link name or nullopt if the entity does not have a
      /// components::ChildLinkName component.
      public: std::optional<std::string> ChildLinkName(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the pose of the joint
      /// \param[in] _ecm Entity-component manager.
      /// \return Pose of the joint or nullopt if the entity does not
      /// have a components::Pose component.
      public: std::optional<math::Pose3d> Pose(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the thread pitch of the joint
      /// \param[in] _ecm Entity-component manager.
      /// \return Thread pitch of the joint or nullopt if the entity does not
      /// have a components::ThreadPitch component.
      public: std::optional<double> ThreadPitch(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the joint axis
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _index Index of the joint. Supported values are: 0 and 1.
      /// \return Axis of the joint or nullopt if the entity does not
      /// have a components::JointAxis or components::JointAxis2 component.
      public: std::optional<sdf::JointAxis> JointAxis(
          const EntityComponentManager &_ecm, unsigned int _index = 0u) const;

      /// \brief Get the joint type
      /// \param[in] _ecm Entity-component manager.
      /// \return Axis of the joint or nullopt if the entity does not
      /// have a components::JointType component.
      public: std::optional<sdf::JointType> JointType(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the ID of a sensor entity which is an immediate child of
      /// this joint.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _name Sensor name.
      /// \return Sensor entity.
      public: gazebo::Entity SensorByName(const EntityComponentManager &_ecm,
          const std::string &_name) const;

      /// \brief Get all sensors which are immediate children of this joint.
      /// \param[in] _ecm Entity-component manager.
      /// \return All sensors in this joint.
      public: std::vector<gazebo::Entity> Sensors(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the number of sensors which are immediate children of this
      /// joint.
      /// \param[in] _ecm Entity-component manager.
      /// \return Number of sensors in this joint.
      public: uint64_t SensorCount(const EntityComponentManager &_ecm) const;

      /// \brief Set velocity on this joint. Only applied if no forces are set
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _velocities Joint velocities to be applied.
      /// The vector of velocities should have the same size as the degrees of
      /// freedom of the joint.
      public: void SetVelocity(EntityComponentManager &_ecm,
          const std::vector<double> &_velocities);

      /// \brief Set force on this joint. If both forces and velocities are set,
      /// only forces are applied
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _forces Joint forces (or torques) to be applied
      /// The vector of forces should have the same size as the degrees of
      /// freedom of the joint.
      public: void SetForce(EntityComponentManager &_ecm,
          const std::vector<double> &_forces);

      /// \brief Set the velocity limits on a joint axis.
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _limits Joint limits to set to. The X() component of the
      /// Vector2 specifies the minimum velocity limit, the Y() component stands
      /// for maximum limit. The vector of limits should have the same size as
      /// the degrees of freedom of the joint.
      public: void SetVelocityLimit(EntityComponentManager &_ecm,
          const std::vector<math::Vector2d> &_limits);

      /// \brief Set the effort limits on a joint axis.
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _limits Joint limits to set to. The X() component of the
      /// Vector2 specifies the minimum effort limit, the Y() component stands
      /// for maximum limit. The vector of limits should have the same size as
      /// the degrees of freedom of the joint.
      public: void SetEffortLimit(EntityComponentManager &_ecm,
          const std::vector<math::Vector2d> &_limits);

      /// \brief Set the position limits on a joint axis.
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _limits Joint limits to set to. The X() component of the
      /// Vector2 specifies the minimum position limit, the Y() component stands
      /// for maximum limit. The vector of limits should have the same size as
      /// the degrees of freedom of the joint.
      public: void SetPositionLimit(EntityComponentManager &_ecm,
          const std::vector<math::Vector2d> &_limits);

      /// \brief Private data pointer.
      IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif

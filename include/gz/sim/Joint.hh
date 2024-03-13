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
#ifndef GZ_SIM_JOINT_HH_
#define GZ_SIM_JOINT_HH_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz/msgs/wrench.pb.h>
#include <gz/utils/ImplPtr.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>

#include "gz/sim/config.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Types.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    //
    /// \class Joint Joint.hh gz/sim/Joint.hh
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
    class GZ_SIM_VISIBLE Joint
    {
      /// \brief Constructor
      /// \param[in] _entity Joint entity
      public: explicit Joint(sim::Entity _entity = kNullEntity);

      /// \brief Get the entity which this Joint is related to.
      /// \return Joint entity.
      public: sim::Entity Entity() const;

      /// \brief Reset Entity to a new one
      /// \param[in] _newEntity New joint entity.
      public: void ResetEntity(sim::Entity _newEntity);

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
      /// \return Axis of the joint or nullopt if the entity does not
      /// have a components::JointAxis or components::JointAxis2 component.
      public: std::optional<std::vector<sdf::JointAxis>> Axis(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the joint type
      /// \param[in] _ecm Entity-component manager.
      /// \return Type of the joint or nullopt if the entity does not
      /// have a components::JointType component.
      public: std::optional<sdf::JointType> Type(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the ID of a sensor entity which is an immediate child of
      /// this joint.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _name Sensor name.
      /// \return Sensor entity.
      public: sim::Entity SensorByName(const EntityComponentManager &_ecm,
          const std::string &_name) const;

      /// \brief Get all sensors which are immediate children of this joint.
      /// \param[in] _ecm Entity-component manager.
      /// \return All sensors in this joint.
      public: std::vector<sim::Entity> Sensors(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the number of sensors which are immediate children of this
      /// joint.
      /// \param[in] _ecm Entity-component manager.
      /// \return Number of sensors in this joint.
      public: uint64_t SensorCount(const EntityComponentManager &_ecm) const;

      /// \brief Set velocity on this joint. Only applied if no forces are set
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _velocities Joint velocities commands (target velocities).
      /// This is different from ResetVelocity in that this does not modify the
      /// internal state of the joint. Instead, the physics engine is expected
      /// to compute the necessary joint torque for the commanded velocity and
      /// apply it in the next simulation step. The vector of velocities should
      /// have the same size as the degrees of freedom of the joint.
      public: void SetVelocity(EntityComponentManager &_ecm,
          const std::vector<double> &_velocities);

      /// \brief Set force on this joint. If both forces and velocities are set,
      /// only forces are applied
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _forces Joint force or torque commands (target forces
      /// or torques). The vector of forces should have the same size as the
      ///  degrees of freedom of the joint.
      public: void SetForce(EntityComponentManager &_ecm,
          const std::vector<double> &_forces);

      /// \brief Set the velocity limits on a joint axis.
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _limits Joint limits to set to. The X() component of the
      /// Vector2 specifies the minimum velocity limit, the Y() component stands
      /// for maximum limit. The vector of limits should have the same size as
      /// the degrees of freedom of the joint.
      public: void SetVelocityLimits(EntityComponentManager &_ecm,
          const std::vector<math::Vector2d> &_limits);

      /// \brief Set the effort limits on a joint axis.
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _limits Joint limits to set to. The X() component of the
      /// Vector2 specifies the minimum effort limit, the Y() component stands
      /// for maximum limit. The vector of limits should have the same size as
      /// the degrees of freedom of the joint.
      public: void SetEffortLimits(EntityComponentManager &_ecm,
          const std::vector<math::Vector2d> &_limits);

      /// \brief Set the position limits on a joint axis.
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _limits Joint limits to set to. The X() component of the
      /// Vector2 specifies the minimum position limit, the Y() component stands
      /// for maximum limit. The vector of limits should have the same size as
      /// the degrees of freedom of the joint.
      public: void SetPositionLimits(EntityComponentManager &_ecm,
          const std::vector<math::Vector2d> &_limits);

      /// \brief Reset the joint positions
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _positions Joint positions to reset to.
      /// The vector of positions should have the same size as the degrees of
      /// freedom of the joint.
      public: void ResetPosition(EntityComponentManager &_ecm,
          const std::vector<double> &_positions);

      /// \brief Reset the joint velocities
      /// \param[in] _ecm Entity Component manager.
      /// \param[in] _velocities Joint velocities to reset to. This is different
      /// from SetVelocity as this modifies the internal state of the joint.
      /// The vector of velocities should have the same size as the degrees of
      /// freedom of the joint.
      public: void ResetVelocity(EntityComponentManager &_ecm,
          const std::vector<double> &_velocities);

      /// \brief By default, Gazebo will not report velocities for a joint, so
      /// functions like `Velocity` will return nullopt. This
      /// function can be used to enable joint velocity check.
      /// \param[in] _ecm Mutable reference to the ECM.
      /// \param[in] _enable True to enable checks, false to disable. Defaults
      /// to true.
      public: void EnableVelocityCheck(EntityComponentManager &_ecm,
          bool _enable = true) const;

      /// \brief By default, Gazebo will not report positions for a joint, so
      /// functions like `Position` will return nullopt. This
      /// function can be used to enable joint position check.
      /// \param[in] _ecm Mutable reference to the ECM.
      /// \param[in] _enable True to enable checks, false to disable. Defaults
      /// to true.
      public: void EnablePositionCheck(EntityComponentManager &_ecm,
          bool _enable = true) const;

      /// \brief By default, Gazebo will not report transmitted wrench for a
      /// joint, so functions like `TransmittedWrench` will return nullopt. This
      /// function can be used to enable joint transmitted wrench check.
      /// \param[in] _ecm Mutable reference to the ECM.
      /// \param[in] _enable True to enable checks, false to disable. Defaults
      /// to true.
      public: void EnableTransmittedWrenchCheck(EntityComponentManager &_ecm,
          bool _enable = true) const;

      /// \brief Get the velocity of the joint
      /// \param[in] _ecm Entity-component manager.
      /// \return Velocity of the joint or nullopt if velocity check
      /// is not enabled.
      /// \sa EnableVelocityCheck
      public: std::optional<std::vector<double>> Velocity(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the position of the joint
      /// \param[in] _ecm Entity-component manager.
      /// \return Position of the joint or nullopt if position check
      /// is not enabled.
      /// \sa EnablePositionCheck
      public: std::optional<std::vector<double>> Position(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the transmitted wrench of the joint
      /// \param[in] _ecm Entity-component manager.
      /// \return Transmitted wrench of the joint or nullopt if transmitted
      /// wrench check is not enabled.
      /// \sa EnableTransmittedWrenchCheck
      public: std::optional<std::vector<msgs::Wrench>> TransmittedWrench(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the parent model
      /// \param[in] _ecm Entity-component manager.
      /// \return Parent Model or nullopt if the entity does not have a
      /// components::ParentEntity component.
      public: std::optional<Model> ParentModel(
          const EntityComponentManager &_ecm) const;

      /// \brief Private data pointer.
      GZ_UTILS_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif

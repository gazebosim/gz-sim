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
#ifndef GZ_SIM_SENSOR_HH_
#define GZ_SIM_SENSOR_HH_

#include <memory>
#include <optional>
#include <string>

#include <gz/utils/ImplPtr.hh>

#include <gz/math/Pose3.hh>

#include <sdf/Sensor.hh>

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
    /// \class Sensor Sensor.hh gz/sim/Sensor.hh
    /// \brief This class provides wrappers around entities and components
    /// which are more convenient and straight-forward to use than dealing
    /// with the `EntityComponentManager` directly.
    /// All the functions provided here are meant to be used with a sensor
    /// entity.
    ///
    /// For example, given a sensor's entity, find the value of its
    /// name component, one could use the entity-component manager (`ecm`)
    /// directly as follows:
    ///
    ///     std::string name = ecm.Component<components::Name>(entity)->Data();
    ///
    /// Using this class however, the same information can be obtained with
    /// a simpler function call:
    ///
    ///    Sensor sensor(entity);
    ///    std::string name = sensor.Name(ecm);
    ///
    class GZ_SIM_VISIBLE Sensor
    {
      /// \brief Constructor
      /// \param[in] _entity Sensor entity
      public: explicit Sensor(sim::Entity _entity = kNullEntity);

      /// \brief Get the entity which this Sensor is related to.
      /// \return Sensor entity.
      public: sim::Entity Entity() const;

      /// \brief Reset Entity to a new one
      /// \param[in] _newEntity New sensor entity.
      public: void ResetEntity(sim::Entity _newEntity);

      /// \brief Check whether this sensor correctly refers to an entity that
      /// has a components::Sensor.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if it's a valid sensor in the manager.
      public: bool Valid(const EntityComponentManager &_ecm) const;

      /// \brief Get the sensor's unscoped name.
      /// \param[in] _ecm Entity-component manager.
      /// \return Sensor's name or nullopt if the entity does not have a
      /// components::Name component
      public: std::optional<std::string> Name(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the pose of the sensor
      /// \param[in] _ecm Entity-component manager.
      /// \return Pose of the sensor or nullopt if the entity does not
      /// have a components::Pose component.
      public: std::optional<math::Pose3d> Pose(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the topic of the sensor
      /// \param[in] _ecm Entity-component manager.
      /// \return Topic of the sensor or nullopt if the entity does not
      /// have a components::SensorTopic component.
      public: std::optional<std::string> Topic(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the parent entity. This can be a link or a joint.
      /// \param[in] _ecm Entity-component manager.
      /// \return Parent entity or nullopt if the entity does not have a
      /// components::ParentEntity component.
      public: std::optional<sim::Entity> Parent(
          const EntityComponentManager &_ecm) const;

      /// \brief Private data pointer.
      GZ_UTILS_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif

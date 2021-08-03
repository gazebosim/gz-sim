/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef CUSTOMSENSORSYSTEM_HH_
#define CUSTOMSENSORSYSTEM_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/sensors/Sensor.hh>
#include <ignition/transport/Node.hh>

namespace custom
{
  /// \brief Example sensor that publishes a noisy double value.
  class CustomSensor : public ignition::sensors::Sensor
  {
    /// \brief Load the sensor with SDF parameters.
    /// \param[in] _sdf SDF Sensor parameters.
    /// \return True if loading was successful
    public: virtual bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Update the sensor and generate data
    /// \param[in] _now The current time
    /// \return True if the update was successfull
    public: virtual bool Update(
      const std::chrono::steady_clock::duration &_now) override;

    /// \brief Set current world pose.
    /// \param[in] _worldPose World pose.
    public: void SetWorldPose(const ignition::math::Pose3d &_worldPose);

    /// \brief Noise that will be applied to the sensor data
    private: ignition::sensors::NoisePtr noise{nullptr};

    /// \brief Node for communication
    private: ignition::transport::Node node;

    /// \brief Publishes sensor data
    private: ignition::transport::Node::Publisher pub;

    /// \brief Latest data
    private: ignition::math::Pose3d data;
  };

  /// \brief Example showing how to tie a custom sensor into simulation.
  class CustomSensorSystem:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
  {
    /// Documentation inherited
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) final;


    /// Documentation inherited
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
        const ignition::gazebo::EntityComponentManager &_ecm) final;

    /// \brief Remove custom sensors if their entities have been removed from
    /// simulation.
    /// \param[in] _ecm Immutable reference to ECM.
    private: void RemoveSensorEntities(
        const ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief A map of custom entities to their sensors
    private: std::unordered_map<ignition::gazebo::Entity,
        std::shared_ptr<CustomSensor>> entitySensorMap;
  };
}
#endif

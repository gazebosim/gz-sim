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

#ifndef IGNITION_GAZEBO_ARDUCOPTERPLUGIN_HH_
#define IGNITION_GAZEBO_ARDUCOPTERPLUGIN_HH_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{   
  // Forward declaration
  class ArduCopterPluginPrivate;

  /// \brief Interface ArduCopter from ardupilot stack
  /// modeled after SITL/SIM_*
  ///
  /// The plugin requires the following parameters:
  /// <rotor>       rotor description block
  ///    id         attribute rotor id
  ///    <vel_p_gain>       velocity pid p gain
  ///    <vel_i_gain>       velocity pid i gain
  ///    <vel_d_gain>       velocity pid d gain
  ///    <vel_i_max>        velocity pid max integral correction
  ///    <vel_i_min>        velocity pid min integral correction
  ///    <vel_cmd_max>      velocity pid max command torque
  ///    <vel_cmd_min>      velocity pid min command torque
  ///    <jointName>        rotor motor joint, torque applied here
  ///    <turningDirection> turning direction, 'cw' or 'ccw'
  ///    <rotorVelocitySlowdownSim> experimental, not needed
  /// <imuName>     scoped name for the imu sensor
  /// <connectionTimeoutMaxCount> timeout before giving up on
  ///                             controller synchronization

  /// \brief a Arducopter Plugin
  class ArduCopterPlugin : 
        public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
     /// \brief Constructor
     public: ArduCopterPlugin();

     /// \brief Destructor.
     public: ~ArduCopterPlugin();

     // Documentation inherited
     public: void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &_eventMgr) override;

     /// Documentation inherited
     public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

     /// \brief Private data pointer
     private: std::unique_ptr<ArduCopterPluginPrivate> dataPtr;
  };
  }
}
}
}
#endif
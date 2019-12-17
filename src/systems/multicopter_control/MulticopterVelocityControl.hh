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
#ifndef IGNITION_GAZEBO_SYSTEMS_MULTICOPTERVELOCITYCONTROL_HH_
#define IGNITION_GAZEBO_SYSTEMS_MULTICOPTERVELOCITYCONTROL_HH_

#include <Eigen/Geometry>
#include <memory>
#include <string>

#include <ignition/transport/Node.hh>

#include <ignition/gazebo/System.hh>
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include "Common.hh"
#include "LeeVelocityController.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \brief This system TBD.
  /// See examples/worlds/quadcopter_velocity_control.sdf for a demonstration.
  /// This system is inspired by the LeePositionController from RotorS
  /// https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_control/include/rotors_control/lee_position_controller.h
  class IGNITION_GAZEBO_VISIBLE MulticopterVelocityControl
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: MulticopterVelocityControl() = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Callback for twist messages
    /// The controller waits for the first twist message before publishing any
    /// rotor velocities.
    /// \param[in] _msg twist message
    private: void OnTwist(const msgs::Twist &_msg);

    /// \brief Callback for enable messages
    /// \param[in] _msg callback message. If false, the controller sends a zero
    /// rotor velocity command once and gets disabled. If the vehicle is in the
    /// air, disabling the controller will cause it to fall. If true, the
    /// controller becomes enabled and waits for a twist message.
    private: void OnEnable(const msgs::Boolean &_msg);

    /// \brief Publish provided rotor velocities
    /// \param[in] _vels Rotor velocities to be published
    private: void PublishRotorVelocities(const Eigen::VectorXd &_vels);

    /// \brief Model interface
    private: Model model{kNullEntity};

    /// \brief Link name
    private: std::string comLinkName;

    /// \brief Link Entity
    private: Entity comLinkEntity;

    /// \brief Topic namespace.
    private: std::string robotNamespace;

    /// \brief Topic for twist commands.
    private: std::string commandSubTopic{"cmd_vel"};

    /// \brief Topic for enable commands.
    private: std::string enableSubTopic{"enable"};

    /// \brief Topic for twist commands.
    private: std::string motorControlPubTopic{"motor_speed"};

    /// \brief Motor control publisher
    private: transport::Node::Publisher motorControlPub;

    /// \brief Ignition communication node.
    private: transport::Node node;

    /// \brief Holds the rotor velocities computed by the controller. This is
    /// here so we don't need to allocate memory every simulation step.
    private: Eigen::VectorXd rotorVelocities;

    /// \brief Velocity controller
    private: std::unique_ptr<multicopter_control::LeeVelocityController>
                 velocityController;

    /// \brief Noise parameteres read from SDF
    private: multicopter_control::NoiseParameters noiseParameters;

    /// \brief Current command velocity. This velocity is a reference velocity
    /// that the controller will try to maintain. A command of zeros must be
    /// given to stop the vehicle.
    private: std::optional<msgs::Twist> cmdVelMsg;

    /// \brief Mutex for cmdVelMsg
    private: std::mutex cmdVelMsgMutex;

    /// \brief Rotor velocities message. This is here so we don't allocate
    /// memory every time we publish a message.
    private: msgs::Actuators rotorVelocitiesMsg;

    /// \brief Whether the system has been initialized with valid parameters
    private: bool initialized{false};

    /// \brief Whether the the controller is active
    private: std::atomic<bool> controllerActive{true};
  };
  }
}
}
}

#endif

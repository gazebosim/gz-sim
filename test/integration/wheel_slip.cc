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

#include <gtest/gtest.h>
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/test_config.hh"
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/SignalStats.hh>
#include <ignition/transport/Node.hh>

using namespace ignition;
using namespace gazebo;

class WheelSlipTest : public ::testing::Test
{ 
  /// \brief Class to hold parameters for tire tests.
  public: class WheelSlipState
  { 
    /// \brief Constructor.
    public: WheelSlipState()
    {
    }
    
    /// \brief Destructor.
    public: ~WheelSlipState() = default;
    
    /// \brief Axel force in lateral direction to expect.
    public: double axelForceLateral = 0.0;
    
    /// \brief Axel force in lateral direction to expect.
    public: double axelForceLongitudinal = 0.0;
    
    /// \brief Description to print during test loop.
    public: std::string description;
    
    /// \brief Drum spin speed in rad/s.
    public: double drumSpeed = 0.0;
    
    /// \brief Steer angle to apply.
    public: ignition::math::Angle steer;
    
    /// \brief Suspension force to apply in N.
    public: double suspForce = 0.0;
    
    /// \brief Wheel slip compliance in lateral direction;
    public: double wheelSlipComplianceLateral = 0.01;
    
    /// \brief Wheel slip compliance in longitudinal direction;
    public: double wheelSlipComplianceLongitudinal = 0.01;
    
    /// \brief Wheel spin speed in rad/s.
    public: double wheelSpeed = 0.0;
    
    /// \brief P gain with wheel spin speed.
    public: double wheelSpeedGain = 0.0;
    
    /// \brief Wheel torque in Nm.
    public: double wheelTorque = 0.0;
  };
  
  /// \brief Class to hold parameters for each model in the TriballDrift test.
  public: class TriballDriftMeasurement
  { 
    /// \brief Reset stats and set reference pose.
    public: void Reset()
    { 
      //this->pose0 = this->model->WorldPose();
      this->statsPositionX.Reset();
      this->statsVelocityX.Reset();
    }
    
    /// \brief Take new measurements.
    public: void Update()
    {
      //this->statsPositionX.InsertData(
      //  this->pose0.Pos().X() - this->model->WorldPose().Pos().X());
      //this->statsVelocityX.InsertData(
      //  this->model->WorldLinearVel().X());
    }
    
    /// \brief Model pointer. 
    public: Model model;
    
    /// \brief Reference pose.
    public: ignition::math::Pose3d pose0;
    
    /// \brief Statistics on position error in X.
    public: ignition::math::SignalMaxAbsoluteValue statsPositionX;
    
    /// \brief Statistics on velocity error in X.
    public: ignition::math::SignalMaxAbsoluteValue statsVelocityX;
  };
  
  /// \brief Set joint commands for tire testrig.
  /// \param[in] _wheelSpeed Wheel spin speed in rad/s.
  /// \param[in] _drumSpeed Drum spin speed in rad/s.
  /// \param[in] _suspForce Suspension force to apply in N.
  /// \param[in] _steer Steer angle to apply.
  public: void SetCommands(const WheelSlipState &_state);
  
  /// \brief Node for Ignition Transport topics.
  protected: ignition::transport::Node ignNode;
  
  /// \brief Publisher of joint commands for the tire model.
  protected: ignition::transport::Node::Publisher tireJointCmdPub;
  
  /// \brief Publisher of joint commands for the drum model.
  protected: ignition::transport::Node::Publisher drumJointCmdPub;

  /// \brief Joint pointer for drum spin joint.
  protected: Entity drumJoint;

  /// \brief Joint pointer for spin joint.
  protected: Entity spinJoint;

  /// \brief Joint pointer for steering joint.
  protected: Entity steerJoint;
};

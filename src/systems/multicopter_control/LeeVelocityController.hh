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

#ifndef GZ_SIM_SYSTEMS_MULTICOPTER_CONTROL_LEEVELOCITYCONTROLLER_HH_
#define GZ_SIM_SYSTEMS_MULTICOPTER_CONTROL_LEEVELOCITYCONTROLLER_HH_

#include <Eigen/Geometry>
#include <memory>
#include "gz/sim/config.hh"

#include "Common.hh"
#include "LeeVelocityController.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
namespace multicopter_control
{
  /// \brief Data structure containing various parameters for the Lee velocity
  /// controller
  struct LeeVelocityControllerParameters
  {
    Eigen::Vector3d velocityGain;
    Eigen::Vector3d attitudeGain;
    Eigen::Vector3d angularRateGain;
    Eigen::Vector3d maxLinearAcceleration;
  };

  /// This controller is inspired by the LeePositionController from RotorS
  /// https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_control/include/rotors_control/lee_position_controller.h
  /// The controller can be used to command linear velocity and yaw angle
  /// velocity expressed in the body frame.
  class LeeVelocityController
  {
    /// \brief Factory function to create LeeVelocityController objects
    /// \param[in] _controllerParams Controller parameteres
    /// \param[in] _vehicleParams Vehicle parameteres
    /// \returns nullptr if the input parameters were valid, otherwise an
    /// pointer to an instance of LeeVelocityController
    public: static std::unique_ptr<LeeVelocityController> MakeController(
                const LeeVelocityControllerParameters &_controllerParams,
                const VehicleParameters &_vehicleParams);

    /// \brief Calculate rotor velocities given the current frame data and the
    /// commanded velocity
    /// \param[in] _frameData Frame data including pose and linear and angular
    /// velocities
    /// \param[in] _cmdVel Commanded velocity
    /// \param[out] _rotorVelocities Computed rotor velocities.
    public: void CalculateRotorVelocities(
                 const FrameData &_frameData,
                 const EigenTwist &_cmdVel,
                 Eigen::VectorXd &_rotorVelocities) const;

    /// \brief Private constructor. Use MakeController to create an instance of
    /// this class
    private: LeeVelocityController() = default;

    /// \brief Compute desired linear acceleration given the current frame data
    /// and commanded linear velocity
    private: Eigen::Vector3d ComputeDesiredAcceleration(
                 const FrameData &_frameData, const EigenTwist &_cmdVel) const;

    /// \brief Compute desired angular acceleration given the current frame
    /// data, comanded angular velocity and desired linear acceleration
    private: Eigen::Vector3d ComputeDesiredAngularAcc(
                 const FrameData &_frameData, const EigenTwist &_cmdVel,
                 const Eigen::Vector3d &_acceleration) const;

    /// \brief Initialize controller parameters
    /// \return True if successful.
    private: bool InitializeParameters();

    /// \brief Velocity controller parameters
    private: LeeVelocityControllerParameters controllerParameters;

    /// \brief Parameters of the whole vehicle
    private: VehicleParameters vehicleParameters;

    /// \brief Attitude gain normalized by the inverse of the inertia matrix
    private: Eigen::Vector3d normalizedAttitudeGain;

    /// \brief Angular gain normalized by the inverse of the inertia matrix
    private: Eigen::Vector3d normalizedAngularRateGain;

    /// \brief Holds the matrix that maps angular acceleration and thrust to
    /// rotor velocities
    private: Eigen::MatrixX4d angularAccToRotorVelocities;
  };
}  // namespace multicopter_control
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif

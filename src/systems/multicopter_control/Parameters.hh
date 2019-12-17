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

#ifndef IGNITION_GAZEBO_SYSTEMS_MULTICOPTERVELOCITYCONTROL_PARAMETERS_HH_
#define IGNITION_GAZEBO_SYSTEMS_MULTICOPTERVELOCITYCONTROL_PARAMETERS_HH_

#include <Eigen/Geometry>
#include <vector>

#include "ignition/gazebo/config.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
namespace multicopter_control
{
  /// \brief A struct that holds various properties of a rotor
  struct Rotor
  {
    // cppcheck-suppress unusedStructMember
    double angle;
    // cppcheck-suppress unusedStructMember
    double armLength;
    // cppcheck-suppress unusedStructMember
    double forceConstant;
    // cppcheck-suppress unusedStructMember
    double momentConstant;
    // cppcheck-suppress unusedStructMember
    int direction;
  };

  /// \brief A collection of Rotor objects
  using RotorConfiguration = std::vector<Rotor>;

  /// \brief A struct that holds properoties of the vehicle such as mass,
  /// inertia and rotor configuration. Gravity is also included even though its
  /// not a parameter unique to the vehicle
  struct VehicleParameters
  {
    // cppcheck-suppress unusedStructMember
    double mass;
    Eigen::Matrix3d inertia;
    Eigen::Vector3d gravity;
    RotorConfiguration rotorConfiguration;
  };

  /// \brief Noise parameters used when computing frame data. These are all
  /// assumed to be gaussian.
  struct NoiseParameters
  {
    Eigen::Vector3d linearVelocityMean;
    Eigen::Vector3d linearVelocityStdDev;
    Eigen::Vector3d angularVelocityMean;
    Eigen::Vector3d angularVelocityStdDev;
  };

}  // namespace multicopter_control
}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif

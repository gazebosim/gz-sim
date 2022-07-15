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

#ifndef GZ_SIM_SYSTEMS_MULTICOPTERVELOCITYCONTROL_PARAMETERS_HH_
#define GZ_SIM_SYSTEMS_MULTICOPTERVELOCITYCONTROL_PARAMETERS_HH_

#include <Eigen/Geometry>
#include <vector>

#include "gz/sim/config.hh"

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
  /// \brief A struct that holds various properties of a rotor
  struct Rotor
  {
    /// \brief The angle formed on the xy plane by the vector going from the
    /// the center of mass to the rotor. The angle is measured from the +x axis
    /// of the body frame.
    // cppcheck-suppress unusedStructMember
    double angle;
    /// \brief The length of the vector going from the center of mass to the
    /// rotor.
    // cppcheck-suppress unusedStructMember
    double armLength;
    /// \brief A constant that multiplies with the square of the rotor's
    /// velocity to compute its thrust.
    // cppcheck-suppress unusedStructMember
    double forceConstant;
    /// \brief A constant the multiplies with the rotor's thrust to compute its
    /// moment.
    // cppcheck-suppress unusedStructMember
    double momentConstant;
    /// \brief Direction of rotation of the rotor. +1 is counterclockwise and -1
    /// is clockwise.
    // cppcheck-suppress unusedStructMember
    int direction;
  };

  /// \brief A collection of Rotor objects
  using RotorConfiguration = std::vector<Rotor>;

  /// \brief A struct that holds properties of the vehicle such as mass,
  /// inertia and rotor configuration. Gravity is also included even though it's
  /// not a parameter unique to the vehicle
  struct VehicleParameters
  {
    /// \brief Total mass of the vehicle
    // cppcheck-suppress unusedStructMember
    double mass;
    /// \brief Moment of inertia matrix of the vehicle
    Eigen::Matrix3d inertia;
    /// \brief Gravity vector
    Eigen::Vector3d gravity;
    /// \brief A collection of Rotor objects that specifiy various properties of
    /// the rotors in the vehicle
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
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif

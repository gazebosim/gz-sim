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

#ifndef GZ_SIM_SYSTEMS_MULTICOPTERVELOCITYCONTROL_COMMON_HH_
#define GZ_SIM_SYSTEMS_MULTICOPTERVELOCITYCONTROL_COMMON_HH_

#include <Eigen/Geometry>
#include <optional>
#include <vector>

#include <sdf/sdf.hh>

#include "gz/sim/config.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"

#include "Parameters.hh"

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
  /// \brief Struct containing linear and angular velocities
  struct EigenTwist
  {
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
  };

  /// \brief Frame data of a link including its pose and linear velocity in
  /// world frame as well as its angular velocity in body frame
  struct FrameData
  {
    // Even though this struct contains Eigen objects, None of them are
    // fixed-size vectorizable, so there is no need to override the new operator
    Eigen::Isometry3d pose;
    Eigen::Vector3d linearVelocityWorld;
    Eigen::Vector3d angularVelocityBody;
  };

  /// \brief Loads rotor configuration from SDF
  /// \param[in] _ecm Immutable reference to the entity component manager
  /// \param[in] _sdf Pointer to the SDF element of the system
  /// \param[in] _model Model to which the system is attached
  /// \param[in] _comLink Link associated with the center of mass.
  RotorConfiguration loadRotorConfiguration(const EntityComponentManager &_ecm,
                                            const sdf::ElementPtr &_sdf,
                                            const Model &_model,
                                            const Entity &_comLink);

  /// \brief Create the matrix that maps rotor velocities to thrust and moments
  /// \param[in] _rotorConfiguration Rotor configurations
  /// \returns nullopt if the rotor configuration results in uncontrollable
  /// system. Otherwise, returns the computed matrix.
  std::optional<Eigen::Matrix4Xd> calculateAllocationMatrix(
      const RotorConfiguration &_rotorConfiguration);

  /// \brief Creates components necessary for obtaining the frame data of the
  /// given link
  /// \param[in] _ecm Mutable reference to the entity component manager
  /// \param[in] _link Link on which the components will be created.
  void createFrameDataComponents(EntityComponentManager &_ecm,
                                 const Entity &_link);

  /// \brief Retrieves the frame data of the given link and applies noise
  /// \param[in] _ecm Imutable reference to the entity component manager
  /// \param[in] _link Link on which the components will be created.
  /// \param[in] _noise Noise parameters
  std::optional<FrameData> getFrameData(const EntityComponentManager &_ecm,
                                        const Entity &_link,
                                        const NoiseParameters &_noise);

  /// \brief Creates a skew symmetric matrix (so(3)) from a vector. This is
  /// sometimes referred to as the hat map
  /// \param[in] _vector any vector in R3
  /// \returns a skew symmetric matrix in so(3)
  inline Eigen::Matrix3d skewMatrixFromVector(const Eigen::Vector3d &_vector)
  {
    Eigen::Matrix3d skewMatrix;
    skewMatrix << 0, -_vector.z(), _vector.y(), _vector.z(), 0, -_vector.x(),
        -_vector.y(), _vector.x(), 0;
    return skewMatrix;
  }

  /// \brief Creates a vector from a skew symmetric matrix(so3). This is
  /// sometimes referred to as the vee map or inverse hat map
  /// \param[in] _skewMatrix any matrix in so(3)
  /// \returns a vector in R3
  inline Eigen::Vector3d vectorFromSkewMatrix(
      const Eigen::Matrix3d &_skewMatrix)
  {
    return Eigen::Vector3d(_skewMatrix(2, 1), _skewMatrix(0, 2),
                           _skewMatrix(1, 0));
  }

}  // namespace multicopter_control
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif

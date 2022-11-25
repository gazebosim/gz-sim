/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_ENVIRONMENTAL_SYSTEM_TRANFORM_TYPE_HH_
#define GZ_ENVIRONMENTAL_SYSTEM_TRANFORM_TYPE_HH_

#include <string>
#include <optional>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

namespace gz {
namespace sim {
/// \brief Transform Type
enum TransformType {
  /// \brief Add the sensor's velocity and retrieve the value after
  /// transforming the vector to the local frame. (Example would be for a
  /// wind speed sensor or current sensor).
  ADD_VELOCITY_LOCAL,
  /// \brief Add the sensor's velocity and retrieve the value in a global
  /// frame. (For instance to get Airspeed in a global frame)
  ADD_VELOCITY_GLOBAL,
  /// \brief The vector doesn't change with speed but we want it in local
  /// frame. Example could be magnetic field lines in local frame.
  LOCAL,
  /// \brief The vector doesn't change with speed but we want it in local
  /// frame. Example could be magnetic field lines in global frame.
  GLOBAL
};

/// \brief Given a string return the type of transform
/// \param[in] _str - input string
/// \return std::nullopt if string invalid, else corresponding transform
std::optional<TransformType> getTransformType(const std::string &_str)
{
  if(_str == "ADD_VELOCITY_LOCAL")
    return TransformType::ADD_VELOCITY_LOCAL;
  if(_str == "ADD_VELOCITY_GLOBAL")
    return TransformType::ADD_VELOCITY_GLOBAL;
  if(_str == "LOCAL")
    return TransformType::LOCAL;
  if(_str == "GLOBAL")
    return TransformType::GLOBAL;
  return std::nullopt;
}

/// \brief Given a string return the type of transform
/// \param[in] _type - Transform type.
/// \param[in] _pose - Global pose of frame to be transformed into.
/// \param[in] _velocity - Velocity of current frame.
/// \param[in] _reading - vector to be transformed.
/// \return transformed vector.
math::Vector3d transformFrame(
  const TransformType _type, const math::Pose3d& _pose,
  const math::Vector3d& _velocity, const math::Vector3d& _reading)
{
  math::Vector3d result;
  math::Vector3d offset{0, 0, 0};

  if (_type == ADD_VELOCITY_LOCAL || _type == ADD_VELOCITY_GLOBAL)
  {
    offset = -_velocity;
  }

  switch (_type) {
      case ADD_VELOCITY_LOCAL:
      case LOCAL:
          result = _pose.Rot().Inverse() * (_reading + offset);
          break;
      default:
          result = (_reading + offset);
  }

  return result;
}
}
}
#endif

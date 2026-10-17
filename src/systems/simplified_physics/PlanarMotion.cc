/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "PlanarMotion.hh"

#include <cmath>

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
namespace
{
/// \brief Below this angle, the series are accurate to machine precision and
/// the closed forms divide two small numbers.
constexpr double kSmallAngle{1e-4};
}

//////////////////////////////////////////////////
PlanarDelta IntegrateArc(double _yaw, double _vx, double _vy, double _wz,
    double _dt)
{
  PlanarDelta delta;
  if (!std::isfinite(_dt) || !std::isfinite(_yaw) || !std::isfinite(_vx) ||
      !std::isfinite(_vy) || !std::isfinite(_wz) || _dt <= 0.0)
  {
    return delta;
  }

  const double theta = _wz * _dt;

  // sin(theta) / theta and (1 - cos(theta)) / theta.
  double sinc{1.0};
  double oneMinusCos{0.0};
  if (std::abs(theta) < kSmallAngle)
  {
    const double t2 = theta * theta;
    sinc = 1.0 - t2 / 6.0;
    oneMinusCos = theta * (0.5 - t2 / 24.0);
  }
  else
  {
    sinc = std::sin(theta) / theta;
    // 1 - cos(theta) = 2 sin^2(theta / 2) avoids cancellation near zero.
    const double halfSin = std::sin(0.5 * theta);
    oneMinusCos = 2.0 * halfSin * halfSin / theta;
  }

  const double localX = _dt * (_vx * sinc - _vy * oneMinusCos);
  const double localY = _dt * (_vy * sinc + _vx * oneMinusCos);

  const double c = std::cos(_yaw);
  const double s = std::sin(_yaw);
  delta.x = c * localX - s * localY;
  delta.y = s * localX + c * localY;
  delta.yaw = theta;
  return delta;
}

//////////////////////////////////////////////////
RigidFrame PlanarFrameAt(const math::Pose3d &_origin,
    const math::Vector3d &_linearVelocity, double _wz,
    const math::Pose3d &_offset)
{
  RigidFrame frame;
  frame.pose = _origin * _offset;

  const math::Vector3d angular(0.0, 0.0, _wz);
  const math::Vector3d arm = frame.pose.Pos() - _origin.Pos();
  frame.linearVelocity = _linearVelocity + angular.Cross(arm);
  frame.angularVelocity = angular;
  frame.linearAcceleration = angular.Cross(frame.linearVelocity);
  return frame;
}
}
}
}

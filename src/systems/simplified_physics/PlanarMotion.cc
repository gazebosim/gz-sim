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
PlanarTwist TwistFromDelta(double _yaw, const PlanarDelta &_delta, double _dt)
{
  PlanarTwist twist;
  if (!std::isfinite(_dt) || !std::isfinite(_yaw) ||
      !std::isfinite(_delta.x) || !std::isfinite(_delta.y) ||
      !std::isfinite(_delta.yaw) || _dt <= 0.0)
  {
    return twist;
  }

  const double theta = _delta.yaw;
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
    const double halfSin = std::sin(0.5 * theta);
    oneMinusCos = 2.0 * halfSin * halfSin / theta;
  }

  // The displacement in the heading frame, per second.
  const double c = std::cos(_yaw);
  const double s = std::sin(_yaw);
  const double localX = (c * _delta.x + s * _delta.y) / _dt;
  const double localY = (-s * _delta.x + c * _delta.y) / _dt;

  // sinc^2 + oneMinusCos^2 is 1 at theta = 0 and shrinks to 0 at a full turn
  // (theta = 2 pi), where a displacement cannot be inverted. A step turns a
  // few milliradians, so the guard only keeps a bad input finite.
  const double det = sinc * sinc + oneMinusCos * oneMinusCos;
  if (det < 1e-12)
    return twist;

  twist.vx = (sinc * localX + oneMinusCos * localY) / det;
  twist.vy = (-oneMinusCos * localX + sinc * localY) / det;
  twist.wz = theta / _dt;
  return twist;
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

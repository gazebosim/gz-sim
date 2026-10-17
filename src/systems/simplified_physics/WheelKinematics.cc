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
#include "WheelKinematics.hh"

#include <algorithm>
#include <cmath>
#include <utility>

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
namespace
{
/// \brief Added to the diagonal of the normal equations, so that a set of
/// wheels that does not fix the twist still has a solution (the one of
/// smallest norm).
constexpr double kRegularization{1e-12};

/// \brief Limits beyond this magnitude mean "no limit". SDF files use
/// +-1.79769e+308 for that.
constexpr double kHugeLimit{1e100};

using Row = std::array<double, 3>;
using Matrix = std::array<Row, 3>;

/// \brief Add the outer product of a row, scaled, to a symmetric matrix.
void AddOuter(Matrix &_m, const Row &_row, double _weight)
{
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
      _m[i][j] += _weight * _row[i] * _row[j];
  }
}

/// \brief Invert a 3x3 matrix with the adjugate.
/// \return False if the matrix is singular.
bool Invert(const Matrix &_m, Matrix &_inverse)
{
  const double c00 = _m[1][1] * _m[2][2] - _m[1][2] * _m[2][1];
  const double c01 = _m[1][2] * _m[2][0] - _m[1][0] * _m[2][2];
  const double c02 = _m[1][0] * _m[2][1] - _m[1][1] * _m[2][0];
  const double det = _m[0][0] * c00 + _m[0][1] * c01 + _m[0][2] * c02;
  if (!std::isfinite(det) || std::abs(det) < 1e-300)
    return false;

  const double s = 1.0 / det;
  _inverse[0][0] = c00 * s;
  _inverse[1][0] = c01 * s;
  _inverse[2][0] = c02 * s;
  _inverse[0][1] = (_m[0][2] * _m[2][1] - _m[0][1] * _m[2][2]) * s;
  _inverse[1][1] = (_m[0][0] * _m[2][2] - _m[0][2] * _m[2][0]) * s;
  _inverse[2][1] = (_m[0][1] * _m[2][0] - _m[0][0] * _m[2][1]) * s;
  _inverse[0][2] = (_m[0][1] * _m[1][2] - _m[0][2] * _m[1][1]) * s;
  _inverse[1][2] = (_m[0][2] * _m[1][0] - _m[0][0] * _m[1][2]) * s;
  _inverse[2][2] = (_m[0][0] * _m[1][1] - _m[0][1] * _m[1][0]) * s;
  return true;
}

/// \brief Row of the rolling constraint of a wheel, for the unknowns
/// (vx, vy, wz).
Row RollingRow(const WheelGeometry &_wheel)
{
  const auto &d = _wheel.rolling;
  const auto &p = _wheel.contact;
  return {d.X(), d.Y(), p.X() * d.Y() - p.Y() * d.X()};
}

/// \brief Row of the sideways constraint of a wheel.
Row SidewaysRow(const WheelGeometry &_wheel)
{
  const math::Vector2d n(-_wheel.rolling.Y(), _wheel.rolling.X());
  const auto &p = _wheel.contact;
  return {n.X(), n.Y(), p.X() * n.Y() - p.Y() * n.X()};
}
}

//////////////////////////////////////////////////
WheelDrive::WheelDrive(std::vector<WheelGeometry> _wheels)
  : wheels(std::move(_wheels))
{
  if (this->wheels.size() > kMaxWheels)
    this->wheels.resize(kMaxWheels);
  this->coefficients.assign(this->wheels.size(), {0.0, 0.0, 0.0});
}

//////////////////////////////////////////////////
std::size_t WheelDrive::Size() const
{
  return this->wheels.size();
}

//////////////////////////////////////////////////
const WheelGeometry &WheelDrive::Wheel(std::size_t _index) const
{
  return this->wheels.at(_index);
}

//////////////////////////////////////////////////
double WheelDrive::Track() const
{
  double track{0.0};
  for (std::size_t i = 0; i < this->wheels.size(); ++i)
  {
    // The unit vector across the rolling direction of wheel i.
    const math::Vector2d across(-this->wheels[i].rolling.Y(),
        this->wheels[i].rolling.X());
    for (std::size_t j = 0; j < this->wheels.size(); ++j)
    {
      if (j == i)
        continue;
      const auto offset = this->wheels[j].contact - this->wheels[i].contact;
      track = std::max(track, std::abs(across.Dot(offset)));
    }
  }
  return track;
}

//////////////////////////////////////////////////
void WheelDrive::Factor(std::uint64_t _driven)
{
  this->cachedSet = _driven;
  for (auto &c : this->coefficients)
    c = {0.0, 0.0, 0.0};

  Matrix normal{};
  for (std::size_t i = 0; i < this->wheels.size(); ++i)
  {
    if (!(_driven & (std::uint64_t{1} << i)))
      continue;
    AddOuter(normal, RollingRow(this->wheels[i]), 1.0);
    AddOuter(normal, SidewaysRow(this->wheels[i]),
        kSidewaysWeight * kSidewaysWeight);
  }
  for (std::size_t i = 0; i < 3; ++i)
    normal[i][i] += kRegularization;

  Matrix inverse{};
  if (!Invert(normal, inverse))
    return;

  for (std::size_t i = 0; i < this->wheels.size(); ++i)
  {
    if (!(_driven & (std::uint64_t{1} << i)))
      continue;
    const Row g = RollingRow(this->wheels[i]);
    for (std::size_t k = 0; k < 3; ++k)
    {
      this->coefficients[i][k] = inverse[k][0] * g[0] +
          inverse[k][1] * g[1] + inverse[k][2] * g[2];
    }
  }
}

//////////////////////////////////////////////////
PlanarTwist WheelDrive::TwistFromRates(const double *_rates,
    std::uint64_t _driven)
{
  PlanarTwist twist;
  if (_driven == 0 || this->wheels.empty())
    return twist;

  if (_driven != this->cachedSet)
    this->Factor(_driven);

  for (std::size_t i = 0; i < this->wheels.size(); ++i)
  {
    if (!(_driven & (std::uint64_t{1} << i)))
      continue;
    const double speed = _rates[i] * this->wheels[i].radius;
    twist.vx += this->coefficients[i][0] * speed;
    twist.vy += this->coefficients[i][1] * speed;
    twist.wz += this->coefficients[i][2] * speed;
  }
  return twist;
}

//////////////////////////////////////////////////
double WheelDrive::RateFromTwist(std::size_t _index,
    const PlanarTwist &_twist) const
{
  const auto &wheel = this->wheels[_index];
  // Velocity of the contact point: v + wz * (z x p).
  const double px = _twist.vx - _twist.wz * wheel.contact.Y();
  const double py = _twist.vy + _twist.wz * wheel.contact.X();
  return (wheel.rolling.X() * px + wheel.rolling.Y() * py) / wheel.radius;
}

//////////////////////////////////////////////////
double AdvanceJoint(double _position, double _rate, double _dt,
    double _lower, double _upper)
{
  double position = _position;
  if (std::isfinite(_rate) && std::isfinite(_dt) && _dt > 0.0)
    position += _rate * _dt;

  const bool hasLower = std::isfinite(_lower) && std::abs(_lower) < kHugeLimit;
  const bool hasUpper = std::isfinite(_upper) && std::abs(_upper) < kHugeLimit;
  if (hasLower && hasUpper && _lower > _upper)
    return position;
  if (hasLower)
    position = std::max(position, _lower);
  if (hasUpper)
    position = std::min(position, _upper);
  return position;
}
}
}
}

/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include "gz/math/Helpers.hh"
#include "gz/math/Angle.hh"

using namespace gz::math;

namespace {

// Use constexpr storage for the Color constants, to avoid the C++ static
// initialization order fiasco.
constexpr Angle gZero(0);
constexpr Angle gPi(GZ_PI);
constexpr Angle gHalfPi(GZ_PI_2);
constexpr Angle gTwoPi(GZ_PI * 2.0);

}  // namespace

const Angle &Angle::Zero = gZero;
const Angle &Angle::Pi = gPi;
const Angle &Angle::HalfPi = gHalfPi;
const Angle &Angle::TwoPi = gTwoPi;

//////////////////////////////////////////////////
void Angle::Radian(double _radian)
{
  this->value = _radian;
}

//////////////////////////////////////////////////
void Angle::SetRadian(double _radian)
{
  this->value = _radian;
}

//////////////////////////////////////////////////
void Angle::Degree(double _degree)
{
  this->value = _degree * GZ_PI / 180.0;
}

//////////////////////////////////////////////////
void Angle::SetDegree(double _degree)
{
  this->value = _degree * GZ_PI / 180.0;
}

//////////////////////////////////////////////////
double Angle::Radian() const
{
  return this->value;
}

//////////////////////////////////////////////////
double Angle::Degree() const
{
  return this->value * 180.0 / GZ_PI;
}

//////////////////////////////////////////////////
void Angle::Normalize()
{
  this->value = atan2(sin(this->value), cos(this->value));
}

//////////////////////////////////////////////////
Angle Angle::Normalized() const
{
  return atan2(sin(this->value), cos(this->value));
}

//////////////////////////////////////////////////
Angle Angle::operator-(const Angle &_angle) const
{
  return Angle(this->value - _angle.value);
}

//////////////////////////////////////////////////
Angle Angle::operator+(const Angle &_angle) const
{
  return Angle(this->value + _angle.value);
}

//////////////////////////////////////////////////
Angle Angle::operator*(const Angle &_angle) const
{
  return Angle(this->value * _angle.value);
}

//////////////////////////////////////////////////
Angle Angle::operator/(const Angle &_angle) const
{
  return Angle(this->value / _angle.value);
}

//////////////////////////////////////////////////
Angle Angle::operator-=(const Angle &_angle)
{
  this->value -= _angle.value;
  return *this;
}

//////////////////////////////////////////////////
Angle Angle::operator+=(const Angle &_angle)
{
  this->value += _angle.value;
  return *this;
}

//////////////////////////////////////////////////
Angle Angle::operator*=(const Angle &_angle)
{
  this->value *= _angle.value;
  return *this;
}

//////////////////////////////////////////////////
Angle Angle::operator/=(const Angle &_angle)
{
  this->value /= _angle.value;
  return *this;
}

//////////////////////////////////////////////////
bool Angle::operator==(const Angle &_angle) const
{
  return equal(this->value, _angle.value, 0.001);
}

//////////////////////////////////////////////////
bool Angle::operator!=(const Angle &_angle) const
{
  return !(*this == _angle);
}

//////////////////////////////////////////////////
bool Angle::operator<(const Angle &_angle) const
{
  return this->value < _angle.value;
}

//////////////////////////////////////////////////
bool Angle::operator<=(const Angle &_angle) const
{
  return this->value < _angle.value || equal(this->value, _angle.value);
}

//////////////////////////////////////////////////
bool Angle::operator>(const Angle &_angle) const
{
  return this->value > _angle.value;
}

//////////////////////////////////////////////////
bool Angle::operator>=(const Angle &_angle) const
{
  return this->value > _angle.value || equal(this->value, _angle.value);
}

//////////////////////////////////////////////////
double Angle::operator()() const
{
  return this->value;
}

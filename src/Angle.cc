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
#include "ignition/math/Helpers.hh"
#include "ignition/math/Angle.hh"

using namespace ignition::math;

const Angle Angle::Zero = Angle(0);
const Angle Angle::Pi = Angle(IGN_PI);
const Angle Angle::HalfPi = Angle(IGN_PI_2);
const Angle Angle::TwoPi = Angle(IGN_PI * 2.0);

//////////////////////////////////////////////////
Angle::Angle()
{
  this->value = 0;
}

//////////////////////////////////////////////////
Angle::Angle(const double _radian)
{
  this->value = _radian;
}

//////////////////////////////////////////////////
Angle::Angle(const Angle &_angle)
{
  this->value = _angle.value;
}

//////////////////////////////////////////////////
Angle::~Angle()
{
}

//////////////////////////////////////////////////
void Angle::Radian(double _radian)
{
  this->value = _radian;
}

//////////////////////////////////////////////////
void Angle::Degree(double _degree)
{
  this->value = _degree * IGN_PI / 180.0;
}

//////////////////////////////////////////////////
double Angle::Radian() const
{
  return this->value;
}

//////////////////////////////////////////////////
double Angle::Degree() const
{
  return this->value * 180.0 / IGN_PI;
}

//////////////////////////////////////////////////
void Angle::Normalize()
{
  this->value = atan2(sin(this->value), cos(this->value));
}

//////////////////////////////////////////////////
Angle Angle::operator-(const Angle &angle) const
{
  return Angle(this->value - angle.value);
}

//////////////////////////////////////////////////
Angle Angle::operator+(const Angle &angle) const
{
  return Angle(this->value + angle.value);
}

//////////////////////////////////////////////////
Angle Angle::operator*(const Angle &angle) const
{
  return Angle(this->value * angle.value);
}

//////////////////////////////////////////////////
Angle Angle::operator/(const Angle &angle) const
{
  return Angle(this->value / angle.value);
}

//////////////////////////////////////////////////
Angle Angle::operator-=(const Angle &angle)
{
  this->value -= angle.value;
  return *this;
}

//////////////////////////////////////////////////
Angle Angle::operator+=(const Angle &angle)
{
  this->value += angle.value;
  return *this;
}

//////////////////////////////////////////////////
Angle Angle::operator*=(const Angle &angle)
{
  this->value *= angle.value;
  return *this;
}

//////////////////////////////////////////////////
Angle Angle::operator/=(const Angle &angle)
{
  this->value /= angle.value;
  return *this;
}

//////////////////////////////////////////////////
bool Angle::operator==(const Angle &angle) const
{
  return equal(this->value, angle.value, 0.001);
}

//////////////////////////////////////////////////
bool Angle::operator!=(const Angle &angle) const
{
  return !(*this == angle);
}

//////////////////////////////////////////////////
bool Angle::operator<(const Angle &angle) const
{
  return this->value < angle.value;
}

//////////////////////////////////////////////////
bool Angle::operator<=(const Angle &angle) const
{
  return this->value < angle.value || equal(this->value, angle.value);
}

//////////////////////////////////////////////////
bool Angle::operator>(const Angle &angle) const
{
  return this->value > angle.value;
}

//////////////////////////////////////////////////
bool Angle::operator>=(const Angle &angle) const
{
  return this->value > angle.value || equal(this->value, angle.value);
}

//////////////////////////////////////////////////
double Angle::operator()() const
{
  return this->value;
}

/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "ignition/math/Vector3.hh"

using namespace ignition;
using namespace math;

const Vector3 Vector3::Zero = math::Vector3(0, 0, 0);
const Vector3 Vector3::One = math::Vector3(1, 1, 1);
const Vector3 Vector3::UnitX = math::Vector3(1, 0, 0);
const Vector3 Vector3::UnitY = math::Vector3(0, 1, 0);
const Vector3 Vector3::UnitZ = math::Vector3(0, 0, 1);

//////////////////////////////////////////////////
Vector3::Vector3()
    : x(0.0), y(0.0), z(0.0)
{
}

//////////////////////////////////////////////////
Vector3::Vector3(const double &_x, const double &_y, const double &_z)
    : x(_x), y(_y), z(_z)
{
}

//////////////////////////////////////////////////
Vector3::Vector3(const Vector3 &_pt)
    : x(_pt.x), y(_pt.y), z(_pt.z)
{
}

//////////////////////////////////////////////////
Vector3::~Vector3()
{
}

//////////////////////////////////////////////////
double Vector3::Distance(const Vector3 &_pt) const
{
  return sqrt((this->x-_pt.x)*(this->x-_pt.x) +
              (this->y-_pt.y)*(this->y-_pt.y) +
              (this->z-_pt.z)*(this->z-_pt.z));
}

//////////////////////////////////////////////////
double Vector3::Distance(double _x, double _y, double _z) const
{
  return this->Distance(Vector3(_x, _y, _z));
}

//////////////////////////////////////////////////
double Vector3::GetSum() const
{
  return this->x + this->y + this->z;
}

//////////////////////////////////////////////////
double Vector3::GetLength() const
{
  return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

//////////////////////////////////////////////////
double Vector3::GetSquaredLength() const
{
  return this->x * this->x + this->y * this->y + this->z * this->z;
}

//////////////////////////////////////////////////
Vector3 Vector3::Normalize()
{
  double d = sqrt(this->x * this->x + this->y * this->y + this->z * this->z);

  if (!math::equal(d, 0.0))
  {
    this->x /= d;
    this->y /= d;
    this->z /= d;
  }

  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::Round()
{
  this->x = nearbyint(this->x);
  this->y = nearbyint(this->y);
  this->z = nearbyint(this->z);
  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::GetRounded() const
{
  Vector3 result = *this;
  result.Round();
  return result;
}

//////////////////////////////////////////////////
Vector3 Vector3::Cross(const Vector3 &_pt) const
{
  Vector3 c(0, 0, 0);

  c.x = this->y * _pt.z - this->z * _pt.y;
  c.y = this->z * _pt.x - this->x * _pt.z;
  c.z = this->x * _pt.y - this->y * _pt.x;

  return c;
}

//////////////////////////////////////////////////
double Vector3::Dot(const Vector3 &_pt) const
{
  return this->x * _pt.x + this->y * _pt.y + this->z * _pt.z;
}

//////////////////////////////////////////////////
Vector3 Vector3::GetAbs() const
{
  return Vector3(fabs(this->x), fabs(this->y), fabs(this->z));
}

//////////////////////////////////////////////////
Vector3 Vector3::GetPerpendicular() const
{
  static const double sqrZero = 1e-06 * 1e-06;

  Vector3 perp = this->Cross(Vector3(1, 0, 0));

  // Check the length of the vector
  if (perp.GetSquaredLength() < sqrZero)
  {
    perp = this->Cross(Vector3(0, 1, 0));
  }

  return perp;
}

//////////////////////////////////////////////////
Vector3 Vector3::GetNormal(const Vector3 &_v1, const Vector3 &_v2,
                           const Vector3 &_v3)
{
  Vector3 a = _v2 - _v1;
  Vector3 b = _v3 - _v1;
  Vector3 n = a.Cross(b);
  return n;
}

//////////////////////////////////////////////////
double Vector3::GetDistToLine(const Vector3 &_pt1, const Vector3 &_pt2)
{
  double d = ((*this) - _pt1).Cross((*this) - _pt2).GetLength();
  d = d / (_pt2 - _pt1).GetLength();
  return d;
}

//////////////////////////////////////////////////
void Vector3::SetToMax(const Vector3 & _v)
{
  if (_v.x > this->x)
    this->x = _v.x;
  if (_v.y > this->y)
    this->y = _v.y;
  if (_v.z > this->z)
    this->z = _v.z;
}

//////////////////////////////////////////////////
void Vector3::SetToMin(const Vector3 & _v)
{
  if (_v.x < this->x)
    this->x = _v.x;
  if (_v.y < this->y)
    this->y = _v.y;
  if (_v.z < this->z)
    this->z = _v.z;
}

//////////////////////////////////////////////////
double Vector3::GetMax() const
{
  return std::max(std::max(this->x, this->y), this->z);
}

//////////////////////////////////////////////////
double Vector3::GetMin() const
{
  return std::min(std::min(this->x, this->y), this->z);
}

//////////////////////////////////////////////////
Vector3 &Vector3::operator =(const Vector3 &_pt)
{
  this->x = _pt.x;
  this->y = _pt.y;
  this->z = _pt.z;

  return *this;
}

//////////////////////////////////////////////////
Vector3 &Vector3::operator =(double value)
{
  this->x = value;
  this->y = value;
  this->z = value;

  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::operator+(const Vector3 &pt) const
{
  return Vector3(this->x + pt.x, this->y + pt.y, this->z + pt.z);
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator+=(const Vector3 &pt)
{
  this->x += pt.x;
  this->y += pt.y;
  this->z += pt.z;

  return *this;
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator-=(const Vector3 &pt)
{
  this->x -= pt.x;
  this->y -= pt.y;
  this->z -= pt.z;

  return *this;
}

//////////////////////////////////////////////////
const Vector3 Vector3::operator/(const Vector3 &pt) const
{
  return Vector3(this->x / pt.x, this->y / pt.y, this->z / pt.z);
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator/=(const Vector3 &pt)
{
  this->x /= pt.x;
  this->y /= pt.y;
  this->z /= pt.z;

  return *this;
}

//////////////////////////////////////////////////
const Vector3 Vector3::operator/(double v) const
{
  return Vector3(this->x / v, this->y / v, this->z / v);
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator/=(double v)
{
  this->x /= v;
  this->y /= v;
  this->z /= v;

  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::operator*(const Vector3 &pt) const
{
  return Vector3(this->x * pt.x, this->y * pt.y, this->z * pt.z);
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator*=(const Vector3 &pt)
{
  this->x *= pt.x;
  this->y *= pt.y;
  this->z *= pt.z;

  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::operator*(double v) const
{
  return Vector3(this->x * v, this->y * v, this->z * v);
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator*=(double v)
{
  this->x *= v;
  this->y *= v;
  this->z *= v;

  return *this;
}

//////////////////////////////////////////////////
bool Vector3::operator ==(const Vector3 &_pt) const
{
  return equal(this->x, _pt.x, 0.001) &&
         equal(this->y, _pt.y, 0.001) &&
         equal(this->z, _pt.z, 0.001);
}

//////////////////////////////////////////////////
bool Vector3::operator!=(const Vector3 &_pt) const
{
  return !(*this == _pt);
}

//////////////////////////////////////////////////
bool Vector3::IsFinite() const
{
  return finite(this->x) && finite(this->y) && finite(this->z);
}

//////////////////////////////////////////////////
double Vector3::operator[](unsigned int index) const
{
  switch (index)
  {
    case 0:
      return this->x;
    case 1:
      return this->y;
    case 2:
      return this->z;
    default:
      return 0;
  }
}

//////////////////////////////////////////////////
/// Round all values to _decimalPlaces
void Vector3::Round(int _precision)
{
  this->x = precision(this->x, _precision);
  this->y = precision(this->y, _precision);
  this->z = precision(this->z, _precision);
}

//////////////////////////////////////////////////
/// Returns true if the two vectors are exacatly equal
bool Vector3::Equal(const Vector3 &_v) const
{
  return math::equal(this->x, _v.x) &&
         math::equal(this->y, _v.y) &&
         math::equal(this->z, _v.z);
}

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
/* Desc: Vector 4
 * Author: Andrew Howard and Nate Koenig
 * Date: 4 Apr 2007
 */

#include <math.h>

#include "ignition/math/Helpers.hh"
#include "ignition/math/Vector4.hh"

using namespace ignition;
using namespace math;


//////////////////////////////////////////////////
Vector4::Vector4()
    : x(0), y(0), z(0), w(0)
{
}

//////////////////////////////////////////////////
Vector4::Vector4(const double &_x, const double &_y, const double &_z,
                 const double &_w)
    : x(_x), y(_y), z(_z), w(_w)
{
}

//////////////////////////////////////////////////
Vector4::Vector4(const Vector4 &_pt)
    : x(_pt.x), y(_pt.y), z(_pt.z), w(_pt.w)
{
}

//////////////////////////////////////////////////
Vector4::~Vector4()
{
}

//////////////////////////////////////////////////
double Vector4::Distance(const Vector4 &_pt) const
{
  return sqrt((this->x-_pt.x)*(this->x-_pt.x) +
              (this->y-_pt.y)*(this->y-_pt.y) +
              (this->z-_pt.z)*(this->z-_pt.z) +
              (this->w-_pt.w)*(this->w-_pt.w));
}

//////////////////////////////////////////////////
double Vector4::GetLength() const
{
  return sqrt(this->x * this->x + this->y * this->y +
              this->z * this->z + this->w * this->w);
}

//////////////////////////////////////////////////
double Vector4::GetSquaredLength() const
{
  return this->x * this->x + this->y * this->y + this->z * this->z +
         this->w * this->w;
}

//////////////////////////////////////////////////
void Vector4::Normalize()
{
  double d = this->GetLength();

  this->x /= d;
  this->y /= d;
  this->z /= d;
  this->w /= d;
}

//////////////////////////////////////////////////
void Vector4::Set(double _x, double _y, double _z, double _w)
{
  this->x = _x;
  this->y = _y;
  this->z = _z;
  this->w = _w;
}


//////////////////////////////////////////////////
Vector4 &Vector4::operator =(const Vector4 &pt)
{
  this->x = pt.x;
  this->y = pt.y;
  this->z = pt.z;
  this->w = pt.w;

  return *this;
}

//////////////////////////////////////////////////
Vector4 &Vector4::operator =(double value)
{
  this->x = value;
  this->y = value;
  this->z = value;
  this->w = value;

  return *this;
}



//////////////////////////////////////////////////
Vector4 Vector4::operator+(const Vector4 &pt) const
{
  return Vector4(this->x + pt.x, this->y + pt.y, this->z + pt.z, this->w+pt.w);
}

const Vector4 &Vector4::operator+=(const Vector4 &pt)
{
  this->x += pt.x;
  this->y += pt.y;
  this->z += pt.z;
  this->w += pt.w;

  return *this;
}

//////////////////////////////////////////////////
Vector4 Vector4::operator-(const Vector4 &pt) const
{
  return Vector4(this->x - pt.x, this->y - pt.y, this->z - pt.z, this->w-pt.w);
}

const Vector4 &Vector4::operator-=(const Vector4 &pt)
{
  this->x -= pt.x;
  this->y -= pt.y;
  this->z -= pt.z;
  this->w -= pt.w;

  return *this;
}


//////////////////////////////////////////////////

const Vector4 Vector4::operator/(const Vector4 &pt) const
{
  return Vector4(this->x / pt.x, this->y / pt.y, this->z / pt.z, this->w/pt.w);
}

const Vector4 &Vector4::operator/=(const Vector4 &pt)
{
  this->x /= pt.x;
  this->y /= pt.y;
  this->z /= pt.z;
  this->w /= pt.w;

  return *this;
}

const Vector4 Vector4::operator/(double v) const
{
  return Vector4(this->x / v, this->y / v, this->z / v, this->w / v);
}

const Vector4 &Vector4::operator/=(double v)
{
  this->x /= v;
  this->y /= v;
  this->z /= v;
  this->w /= v;

  return *this;
}



//////////////////////////////////////////////////
const Vector4 Vector4::operator*(const Vector4 &pt) const
{
  return Vector4(this->x * pt.x, this->y * pt.y, this->z * pt.z, this->w*pt.w);
}

const Vector4 Vector4::operator*(const Matrix4 &_m) const
{
  return Vector4(
      this->x*_m[0][0] + this->y*_m[1][0] + this->z*_m[2][0] + this->w*_m[3][0],
      this->x*_m[0][1] + this->y*_m[1][1] + this->z*_m[2][1] + this->w*_m[3][1],
      this->x*_m[0][2] + this->y*_m[1][2] + this->z*_m[2][2] + this->w*_m[3][2],
      this->x*_m[0][3] + this->y*_m[1][3] + this->z*_m[2][3] + this->w*_m[3][3]
);
}

const Vector4 &Vector4::operator*=(const Vector4 &pt)
{
  this->x *= pt.x;
  this->y *= pt.y;
  this->z *= pt.z;
  this->w *= pt.w;

  return *this;
}

const Vector4 Vector4::operator*(double v) const
{
  return Vector4(this->x * v, this->y * v, this->z * v, this->w*v);
}

const Vector4 &Vector4::operator*=(double v)
{
  this->x *= v;
  this->y *= v;
  this->z *= v;
  this->w *= v;

  return *this;
}

//////////////////////////////////////////////////
bool Vector4::operator ==(const Vector4 &pt) const
{
  return equal(this->x, pt.x) && equal(this->y, pt.y) &&
         equal(this->z, pt.z) && equal(this->w, pt.w);
}

//////////////////////////////////////////////////
bool Vector4::operator!=(const Vector4 &pt) const
{
  return !(*this == pt);
}

//////////////////////////////////////////////////
bool Vector4::IsFinite() const
{
  return finite(this->x) && finite(this->y) && finite(this->z) &&
         finite(this->w);
}

//////////////////////////////////////////////////
double Vector4::operator[](unsigned int index) const
{
  switch (index)
  {
    case 0:
      return this->x;
    case 1:
      return this->y;
    case 2:
      return this->z;
    case 3:
      return this->w;
    default:
      return 0;
  }
}



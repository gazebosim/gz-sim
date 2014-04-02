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
/* Desc: Vector 2
 * Author: Nate Koenig
 * Date: 21 July 2007
 */

#include <math.h>
#include "ignition/math/Vector2i.hh"

using namespace ignition;
using namespace math;

//////////////////////////////////////////////////
Vector2i::Vector2i()
  : x(0), y(0)
{
}

//////////////////////////////////////////////////
Vector2i::Vector2i(const int &_x, const int &_y)
  : x(_x), y(_y)
{
}

//////////////////////////////////////////////////
Vector2i::Vector2i(const Vector2i &_pt)
  : x(_pt.x), y(_pt.y)
{
}

//////////////////////////////////////////////////
Vector2i::~Vector2i()
{
}

//////////////////////////////////////////////////
int Vector2i::Distance(const Vector2i &_pt) const
{
  return sqrt((this->x-_pt.x)*(this->x-_pt.x) +
              (this->y-_pt.y)*(this->y-_pt.y));
}

//////////////////////////////////////////////////
void Vector2i::Normalize()
{
  int d = sqrt(this->x * this->x + this->y * this->y);

  this->x /= d;
  this->y /= d;
}

//////////////////////////////////////////////////
void Vector2i::Set(int _x, int _y)
{
  this->x = _x;
  this->y = _y;
}

//////////////////////////////////////////////////
Vector2i &Vector2i::operator =(const Vector2i &_pt)
{
  this->x = _pt.x;
  this->y = _pt.y;

  return *this;
}

//////////////////////////////////////////////////
const Vector2i &Vector2i::operator =(int _value)
{
  this->x = _value;
  this->y = _value;

  return *this;
}

//////////////////////////////////////////////////
Vector2i Vector2i::operator+(const Vector2i &_pt) const
{
  return Vector2i(this->x + _pt.x, this->y + _pt.y);
}

//////////////////////////////////////////////////
const Vector2i &Vector2i::operator+=(const Vector2i &_pt)
{
  this->x += _pt.x;
  this->y += _pt.y;

  return *this;
}

//////////////////////////////////////////////////
Vector2i Vector2i::operator-(const Vector2i &_pt) const
{
  return Vector2i(this->x - _pt.x, this->y - _pt.y);
}

//////////////////////////////////////////////////
const Vector2i &Vector2i::operator-=(const Vector2i &_pt)
{
  this->x -= _pt.x;
  this->y -= _pt.y;

  return *this;
}

//////////////////////////////////////////////////
const Vector2i Vector2i::operator/(const Vector2i &_pt) const
{
  return Vector2i(this->x / _pt.x, this->y / _pt.y);
}

//////////////////////////////////////////////////
const Vector2i &Vector2i::operator/=(const Vector2i &_pt)
{
  this->x /= _pt.x;
  this->y /= _pt.y;

  return *this;
}

//////////////////////////////////////////////////
const Vector2i Vector2i::operator/(int _v) const
{
  return Vector2i(this->x / _v, this->y / _v);
}

//////////////////////////////////////////////////
const Vector2i &Vector2i::operator/=(int _v)
{
  this->x /= _v;
  this->y /= _v;

  return *this;
}

//////////////////////////////////////////////////
const Vector2i Vector2i::operator*(const Vector2i &_pt) const
{
  return Vector2i(this->x * _pt.x, this->y * _pt.y);
}

//////////////////////////////////////////////////
const Vector2i &Vector2i::operator*=(const Vector2i &_pt)
{
  this->x *= _pt.x;
  this->y *= _pt.y;

  return *this;
}

//////////////////////////////////////////////////
const Vector2i Vector2i::operator*(int _v) const
{
  return Vector2i(this->x * _v, this->y * _v);
}

//////////////////////////////////////////////////
const Vector2i &Vector2i::operator*=(int _v)
{
  this->x *= _v;
  this->y *= _v;

  return *this;
}

//////////////////////////////////////////////////
bool Vector2i::operator ==(const Vector2i &_pt) const
{
  return this->x == _pt.x && this->y == _pt.y;
}

//////////////////////////////////////////////////
bool Vector2i::IsFinite() const
{
  return finite(this->x) && finite(this->y);
}

//////////////////////////////////////////////////
int Vector2i::operator[](unsigned int index) const
{
  switch (index)
  {
    case 0:
      return this->x;
    case 1:
      return this->y;
    default:
      return 0;
  }
}



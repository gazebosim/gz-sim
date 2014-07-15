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
#include <cmath>
#include <ignition/math/Box.hh>

using namespace ignition;
using namespace math;

//////////////////////////////////////////////////
Box::Box()
: min(0, 0, 0), max(0, 0, 0), extent(EXTENT_NULL)
{
}

//////////////////////////////////////////////////
Box::Box(double _minX, double _minY, double _minZ,
         double _maxX, double _maxY, double _maxZ)
: min(_minX, _minY, _minZ),
  max(_maxX, _maxY, _maxZ),
  extent(EXTENT_FINITE)
{
}

//////////////////////////////////////////////////
Box::Box(const Vector3d &_min, const Vector3d &_max)
  : min(_min), max(_max), extent(EXTENT_FINITE)
{
}

//////////////////////////////////////////////////
Box::Box(const Box &_b)
  : min(_b.min), max(_b.max), extent(_b.extent)
{
}

//////////////////////////////////////////////////
Box::~Box()
{
}

//////////////////////////////////////////////////
double Box::XLength() const
{
  return std::abs(max.X() - min.X());
}

//////////////////////////////////////////////////
double Box::YLength() const
{
  return std::abs(max.Y() - min.Y());
}

//////////////////////////////////////////////////
double Box::ZLength() const
{
  return std::abs(max.Z() - min.Z());
}

//////////////////////////////////////////////////
math::Vector3d Box::Size() const
{
  return math::Vector3d(this->XLength(),
                        this->YLength(),
                        this->ZLength());
}

//////////////////////////////////////////////////
math::Vector3d Box::Center() const
{
  Vector3d size = this->Size();
  size /= 2.0;
  return this->min + size;
}


//////////////////////////////////////////////////
void Box::Merge(const Box &_box)
{
  if (this->extent == EXTENT_NULL)
  {
    this->min = _box.min;
    this->max = _box.max;
    this->extent = _box.extent;
  }
  else
  {
    this->min.Min(_box.min);
    this->max.Max(_box.max);
  }
}

//////////////////////////////////////////////////
Box &Box::operator =(const Box &_b)
{
  this->max = _b.max;
  this->min = _b.min;
  this->extent = _b.extent;

  return *this;
}

//////////////////////////////////////////////////
Box Box::operator+(const Box &_b) const
{
  Vector3d mn, mx;

  if (this->extent != EXTENT_NULL)
  {
    mn = this->min;
    mx = this->max;

    mn.Min(_b.min);
    mx.Max(_b.max);
  }
  else
  {
    mn = _b.min;
    mx = _b.max;
  }

  return Box(mn, mx);
}

//////////////////////////////////////////////////
const Box &Box::operator+=(const Box &_b)
{
  if (this->extent != EXTENT_NULL)
  {
    this->min.Min(_b.min);
    this->max.Max(_b.max);
  }
  else
  {
    this->min = _b.min;
    this->max = _b.max;
    this->extent = _b.extent;
  }
  return *this;
}

//////////////////////////////////////////////////
bool Box::operator==(const Box &_b) const
{
  return this->min == _b.min && this->max == _b.max;
}

//////////////////////////////////////////////////
bool Box::operator!=(const Box &_b) const
{
  return !(*this == _b);
}

//////////////////////////////////////////////////
Box Box::operator-(const Vector3d &_v)
{
  return Box(this->min - _v, this->max - _v);
}

//////////////////////////////////////////////////
bool Box::Intersects(const Box &_box) const
{
  // Check the six separating planes.
  if (this->Max().X() < _box.Min().X())
    return false;
  if (this->Max().Y() < _box.Min().Y())
    return false;
  if (this->Max().Z() < _box.Min().Z())
    return false;

  if (this->Min().X() > _box.Max().X())
    return false;
  if (this->Min().Y() > _box.Max().Y())
    return false;
  if (this->Min().Z() > _box.Max().Z())
    return false;

  // Otherwise the two boxes must intersect.
  return true;
}

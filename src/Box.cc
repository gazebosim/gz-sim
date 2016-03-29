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

#include "ignition/math/BoxPrivate.hh"

using namespace ignition;
using namespace math;

//////////////////////////////////////////////////
Box::Box()
: dataPtr(new BoxPrivate)
{
}

//////////////////////////////////////////////////
Box::Box(double _vec1X, double _vec1Y, double _vec1Z,
         double _vec2X, double _vec2Y, double _vec2Z)
: dataPtr(new BoxPrivate)
{
  this->dataPtr->extent = BoxPrivate::EXTENT_FINITE;
  this->dataPtr->min.Set(_vec1X, _vec1Y, _vec1Z);
  this->dataPtr->max.Set(_vec2X, _vec2Y, _vec2Z);

  this->dataPtr->min.Min(math::Vector3d(_vec2X, _vec2Y, _vec2Z));
  this->dataPtr->max.Max(math::Vector3d(_vec1X, _vec1Y, _vec1Z));
}

//////////////////////////////////////////////////
Box::Box(const Vector3d &_vec1, const Vector3d &_vec2)
: dataPtr(new BoxPrivate)
{
  this->dataPtr->extent = BoxPrivate::EXTENT_FINITE;
  this->dataPtr->min = _vec1;
  this->dataPtr->min.Min(_vec2);

  this->dataPtr->max = _vec2;
  this->dataPtr->max.Max(_vec1);
}

//////////////////////////////////////////////////
Box::Box(const Box &_b)
: dataPtr(new BoxPrivate)
{
  this->dataPtr->min = _b.dataPtr->min;
  this->dataPtr->max = _b.dataPtr->max;
  this->dataPtr->extent = _b.dataPtr->extent;
}

//////////////////////////////////////////////////
Box::~Box()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
double Box::XLength() const
{
  return std::abs(this->dataPtr->max.X() - this->dataPtr->min.X());
}

//////////////////////////////////////////////////
double Box::YLength() const
{
  return std::abs(this->dataPtr->max.Y() - this->dataPtr->min.Y());
}

//////////////////////////////////////////////////
double Box::ZLength() const
{
  return std::abs(this->dataPtr->max.Z() - this->dataPtr->min.Z());
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
  return this->dataPtr->min + (this->dataPtr->max - this->dataPtr->min) * 0.5;
}


//////////////////////////////////////////////////
void Box::Merge(const Box &_box)
{
  if (this->dataPtr->extent == BoxPrivate::EXTENT_NULL)
  {
    this->dataPtr->min = _box.dataPtr->min;
    this->dataPtr->max = _box.dataPtr->max;
    this->dataPtr->extent = _box.dataPtr->extent;
  }
  else
  {
    this->dataPtr->min.Min(_box.dataPtr->min);
    this->dataPtr->max.Max(_box.dataPtr->max);
  }
}

//////////////////////////////////////////////////
Box &Box::operator =(const Box &_b)
{
  this->dataPtr->max = _b.dataPtr->max;
  this->dataPtr->min = _b.dataPtr->min;
  this->dataPtr->extent = _b.dataPtr->extent;

  return *this;
}

//////////////////////////////////////////////////
Box Box::operator+(const Box &_b) const
{
  Vector3d mn, mx;

  if (this->dataPtr->extent != BoxPrivate::EXTENT_NULL)
  {
    mn = this->dataPtr->min;
    mx = this->dataPtr->max;

    mn.Min(_b.dataPtr->min);
    mx.Max(_b.dataPtr->max);
  }
  else
  {
    mn = _b.dataPtr->min;
    mx = _b.dataPtr->max;
  }

  return Box(mn, mx);
}

//////////////////////////////////////////////////
const Box &Box::operator+=(const Box &_b)
{
  if (this->dataPtr->extent != BoxPrivate::EXTENT_NULL)
  {
    this->dataPtr->min.Min(_b.dataPtr->min);
    this->dataPtr->max.Max(_b.dataPtr->max);
  }
  else
  {
    this->dataPtr->min = _b.dataPtr->min;
    this->dataPtr->max = _b.dataPtr->max;
    this->dataPtr->extent = _b.dataPtr->extent;
  }
  return *this;
}

//////////////////////////////////////////////////
bool Box::operator==(const Box &_b) const
{
  return this->dataPtr->min == _b.dataPtr->min &&
         this->dataPtr->max == _b.dataPtr->max;
}

//////////////////////////////////////////////////
bool Box::operator!=(const Box &_b) const
{
  return !(*this == _b);
}

//////////////////////////////////////////////////
Box Box::operator-(const Vector3d &_v)
{
  return Box(this->dataPtr->min - _v, this->dataPtr->max - _v);
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

//////////////////////////////////////////////////
const Vector3d &Box::Min() const
{
  return this->dataPtr->min;
}

//////////////////////////////////////////////////
const Vector3d &Box::Max() const
{
  return this->dataPtr->max;
}

//////////////////////////////////////////////////
Vector3d &Box::Min()
{
  return this->dataPtr->min;
}

//////////////////////////////////////////////////
Vector3d &Box::Max()
{
  return this->dataPtr->max;
}

//////////////////////////////////////////////////
bool Box::Contains(const Vector3d &_p) const
{
  return _p.X() >= this->dataPtr->min.X() && _p.X() <= this->dataPtr->max.X() &&
         _p.Y() >= this->dataPtr->min.Y() && _p.Y() <= this->dataPtr->max.Y() &&
         _p.Z() >= this->dataPtr->min.Z() && _p.Z() <= this->dataPtr->max.Z();
}

//////////////////////////////////////////////////
bool Box::ClipLine(const int _d, const Line3d &_line,
                   double &_low, double &_high) const
{
  // dimLow and dimHigh are the results we're calculating for this
  // current dimension.
  double dimLow, dimHigh;

  // Find the point of intersection in this dimension only as a fraction of
  // the total vector http://youtu.be/USjbg5QXk3g?t=3m12s
  dimLow = (this->dataPtr->min[_d] - _line[0][_d]) /
    (_line[1][_d] - _line[0][_d]);

  dimHigh = (this->dataPtr->max[_d] - _line[0][_d]) /
    (_line[1][_d] - _line[0][_d]);

  // Make sure low is less than high
  if (dimHigh < dimLow)
    std::swap(dimHigh, dimLow);

  // If this dimension's high is less than the low we got then we definitely
  // missed. http://youtu.be/USjbg5QXk3g?t=7m16s
  if (dimHigh < _low)
    return false;

  // Likewise if the low is less than the high.
  if (dimLow > _high)
    return false;

  // Add the clip from this dimension to the previous results
  // http://youtu.be/USjbg5QXk3g?t=5m32s
  if (std::isfinite(dimLow))
    _low = std::max(dimLow, _low);

  if (std::isfinite(dimHigh))
    _high = std::min(dimHigh, _high);

  return true;
}

/////////////////////////////////////////////////
bool Box::IntersectCheck(const Vector3d &_origin, const Vector3d &_dir,
    const double _min, const double _max) const
{
  return std::get<0>(this->Intersect(_origin, _dir, _min, _max));
}

/////////////////////////////////////////////////
std::tuple<bool, double> Box::IntersectDist(const Vector3d &_origin,
    const Vector3d &_dir, const double _min, const double _max) const
{
  return std::make_tuple(
      std::get<0>(this->Intersect(_origin, _dir, _min, _max)),
      std::get<1>(this->Intersect(_origin, _dir, _min, _max)));
}

/////////////////////////////////////////////////
std::tuple<bool, double, Vector3d>  Box::Intersect(
    const Vector3d &_origin, const Vector3d &_dir,
    const double _min, const double _max) const
{
  Vector3d dir = _dir;
  dir.Normalize();
  return this->Intersect(Line3d(_origin + dir * _min, _origin + dir * _max));
}

/////////////////////////////////////////////////
// Find the intersection of a line from v0 to v1 and an
// axis-aligned bounding box http://www.youtube.com/watch?v=USjbg5QXk3g
std::tuple<bool, double, Vector3d> Box::Intersect(const Line3d &_line) const
{
  // low and high are the results from all clipping so far.
  // We'll write our results back out to those parameters.
  double low = 0;
  double high = 1;

  if (!this->ClipLine(0, _line, low, high))
    return std::make_tuple(false, 0, Vector3d::Zero);

  if (!this->ClipLine(1, _line, low, high))
    return std::make_tuple(false, 0, Vector3d::Zero);

  if (!this->ClipLine(2, _line, low, high))
    return std::make_tuple(false, 0, Vector3d::Zero);

  // The formula for I: http://youtu.be/USjbg5QXk3g?t=6m24s
  Vector3d intersection = _line[0] + ((_line[1] - _line[0]) * low);

  return std::make_tuple(true, _line[0].Distance(intersection), intersection);
}

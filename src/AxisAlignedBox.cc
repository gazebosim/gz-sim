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
#include <cmath>
#include <ignition/math/AxisAlignedBox.hh>

using namespace ignition;
using namespace math;

// Private data for AxisAlignedBox class
class ignition::math::AxisAlignedBoxPrivate
{
  /// \brief Minimum corner of the box
  public: Vector3d min = Vector3d(MAX_D, MAX_D, MAX_D);

  /// \brief Maximum corner of the box
  public: Vector3d max = Vector3d(LOW_D, LOW_D, LOW_D);
};

//////////////////////////////////////////////////
AxisAlignedBox::AxisAlignedBox()
: dataPtr(new AxisAlignedBoxPrivate)
{
}

//////////////////////////////////////////////////
AxisAlignedBox::AxisAlignedBox(double _vec1X, double _vec1Y, double _vec1Z,
         double _vec2X, double _vec2Y, double _vec2Z)
: dataPtr(new AxisAlignedBoxPrivate)
{
  this->dataPtr->min.Set(_vec1X, _vec1Y, _vec1Z);
  this->dataPtr->max.Set(_vec2X, _vec2Y, _vec2Z);

  this->dataPtr->min.Min(math::Vector3d(_vec2X, _vec2Y, _vec2Z));
  this->dataPtr->max.Max(math::Vector3d(_vec1X, _vec1Y, _vec1Z));
}

//////////////////////////////////////////////////
AxisAlignedBox::AxisAlignedBox(const Vector3d &_vec1, const Vector3d &_vec2)
: dataPtr(new AxisAlignedBoxPrivate)
{
  this->dataPtr->min = _vec1;
  this->dataPtr->min.Min(_vec2);

  this->dataPtr->max = _vec2;
  this->dataPtr->max.Max(_vec1);
}

//////////////////////////////////////////////////
AxisAlignedBox::AxisAlignedBox(const AxisAlignedBox &_b)
: dataPtr(new AxisAlignedBoxPrivate)
{
  this->dataPtr->min = _b.dataPtr->min;
  this->dataPtr->max = _b.dataPtr->max;
}

//////////////////////////////////////////////////
AxisAlignedBox::~AxisAlignedBox()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
double AxisAlignedBox::XLength() const
{
  return std::max(0.0, this->dataPtr->max.X() - this->dataPtr->min.X());
}

//////////////////////////////////////////////////
double AxisAlignedBox::YLength() const
{
  return std::max(0.0, this->dataPtr->max.Y() - this->dataPtr->min.Y());
}

//////////////////////////////////////////////////
double AxisAlignedBox::ZLength() const
{
  return std::max(0.0, this->dataPtr->max.Z() - this->dataPtr->min.Z());
}

//////////////////////////////////////////////////
math::Vector3d AxisAlignedBox::Size() const
{
  return math::Vector3d(this->XLength(),
                        this->YLength(),
                        this->ZLength());
}

//////////////////////////////////////////////////
math::Vector3d AxisAlignedBox::Center() const
{
  return 0.5 * this->dataPtr->min + 0.5 * this->dataPtr->max;
}


//////////////////////////////////////////////////
void AxisAlignedBox::Merge(const AxisAlignedBox &_box)
{
  this->dataPtr->min.Min(_box.dataPtr->min);
  this->dataPtr->max.Max(_box.dataPtr->max);
}

//////////////////////////////////////////////////
AxisAlignedBox &AxisAlignedBox::operator =(const AxisAlignedBox &_b)
{
  this->dataPtr->max = _b.dataPtr->max;
  this->dataPtr->min = _b.dataPtr->min;

  return *this;
}

//////////////////////////////////////////////////
AxisAlignedBox AxisAlignedBox::operator+(const AxisAlignedBox &_b) const
{
  AxisAlignedBox result(*this);
  result += _b;
  return result;
}

//////////////////////////////////////////////////
const AxisAlignedBox &AxisAlignedBox::operator+=(const AxisAlignedBox &_b)
{
  this->dataPtr->min.Min(_b.dataPtr->min);
  this->dataPtr->max.Max(_b.dataPtr->max);
  return *this;
}

//////////////////////////////////////////////////
bool AxisAlignedBox::operator==(const AxisAlignedBox &_b) const
{
  return this->dataPtr->min == _b.dataPtr->min &&
         this->dataPtr->max == _b.dataPtr->max;
}

//////////////////////////////////////////////////
bool AxisAlignedBox::operator!=(const AxisAlignedBox &_b) const
{
  return !(*this == _b);
}

//////////////////////////////////////////////////
AxisAlignedBox AxisAlignedBox::operator-(const Vector3d &_v)
{
  return AxisAlignedBox(this->dataPtr->min - _v, this->dataPtr->max - _v);
}

//////////////////////////////////////////////////
bool AxisAlignedBox::Intersects(const AxisAlignedBox &_box) const
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
const Vector3d &AxisAlignedBox::Min() const
{
  return this->dataPtr->min;
}

//////////////////////////////////////////////////
const Vector3d &AxisAlignedBox::Max() const
{
  return this->dataPtr->max;
}

//////////////////////////////////////////////////
Vector3d &AxisAlignedBox::Min()
{
  return this->dataPtr->min;
}

//////////////////////////////////////////////////
Vector3d &AxisAlignedBox::Max()
{
  return this->dataPtr->max;
}

//////////////////////////////////////////////////
bool AxisAlignedBox::Contains(const Vector3d &_p) const
{
  return _p.X() >= this->dataPtr->min.X() && _p.X() <= this->dataPtr->max.X() &&
         _p.Y() >= this->dataPtr->min.Y() && _p.Y() <= this->dataPtr->max.Y() &&
         _p.Z() >= this->dataPtr->min.Z() && _p.Z() <= this->dataPtr->max.Z();
}

//////////////////////////////////////////////////
bool AxisAlignedBox::ClipLine(const int _d, const Line3d &_line,
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
bool AxisAlignedBox::IntersectCheck(
    const Vector3d &_origin, const Vector3d &_dir,
    const double _min, const double _max) const
{
  return std::get<0>(this->Intersect(_origin, _dir, _min, _max));
}

/////////////////////////////////////////////////
std::tuple<bool, double> AxisAlignedBox::IntersectDist(const Vector3d &_origin,
    const Vector3d &_dir, const double _min, const double _max) const
{
  return std::make_tuple(
      std::get<0>(this->Intersect(_origin, _dir, _min, _max)),
      std::get<1>(this->Intersect(_origin, _dir, _min, _max)));
}

/////////////////////////////////////////////////
std::tuple<bool, double, Vector3d>  AxisAlignedBox::Intersect(
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
std::tuple<bool, double, Vector3d> AxisAlignedBox::Intersect(
    const Line3d &_line) const
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

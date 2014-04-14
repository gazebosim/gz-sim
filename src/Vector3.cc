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

const IGN_VECTOR3 IGN_VECTOR3::Zero = math::IGN_VECTOR3(0, 0, 0);
const IGN_VECTOR3 IGN_VECTOR3::One = math::IGN_VECTOR3(1, 1, 1);
const IGN_VECTOR3 IGN_VECTOR3::UnitX = math::IGN_VECTOR3(1, 0, 0);
const IGN_VECTOR3 IGN_VECTOR3::UnitY = math::IGN_VECTOR3(0, 1, 0);
const IGN_VECTOR3 IGN_VECTOR3::UnitZ = math::IGN_VECTOR3(0, 0, 1);

//////////////////////////////////////////////////
IGN_VECTOR3::IGN_VECTOR3()
{
  this->data[0] = 0;
  this->data[1] = 0;
  this->data[2] = 0;
}

//////////////////////////////////////////////////
IGN_VECTOR3::IGN_VECTOR3(const IGN_NUMERIC &_x, const IGN_NUMERIC &_y,
                         const IGN_NUMERIC &_z)
{
  this->data[0] = _x;
  this->data[1] = _y;
  this->data[2] = _z;
}

//////////////////////////////////////////////////
IGN_VECTOR3::IGN_VECTOR3(const IGN_VECTOR3 &_pt)
{
  this->data[0] = _pt[0];
  this->data[1] = _pt[1];
  this->data[2] = _pt[2];
}

//////////////////////////////////////////////////
IGN_VECTOR3::~IGN_VECTOR3()
{
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR3::Distance(const IGN_VECTOR3 &_pt) const
{
  return sqrt((this->data[0]-_pt[0])*(this->data[0]-_pt[0]) +
              (this->data[1]-_pt[1])*(this->data[1]-_pt[1]) +
              (this->data[2]-_pt[2])*(this->data[2]-_pt[2]));
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR3::Distance(IGN_NUMERIC _x, IGN_NUMERIC _y,
                                  IGN_NUMERIC _z) const
{
  return this->Distance(IGN_VECTOR3(_x, _y, _z));
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR3::GetSum() const
{
  return this->data[0] + this->data[1] + this->data[2];
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR3::GetLength() const
{
  return sqrt(this->data[0] * this->data[0] +
              this->data[1] * this->data[1] +
              this->data[2] * this->data[2]);
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR3::GetSquaredLength() const
{
  return this->data[0] * this->data[0] +
         this->data[1] * this->data[1] +
         this->data[2] * this->data[2];
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_VECTOR3::Normalize()
{
  IGN_NUMERIC d = sqrt(this->data[0] * this->data[0] +
                       this->data[1] * this->data[1] +
                       this->data[2] * this->data[2]);

  if (!math::equal(d, static_cast<IGN_NUMERIC>(0.0)))
  {
    this->data[0] /= d;
    this->data[1] /= d;
    this->data[2] /= d;
  }

  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_VECTOR3::Round()
{
  this->data[0] = nearbyint(this->data[0]);
  this->data[1] = nearbyint(this->data[1]);
  this->data[2] = nearbyint(this->data[2]);
  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_VECTOR3::GetRounded() const
{
  IGN_VECTOR3 result = *this;
  result.Round();
  return result;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_VECTOR3::Cross(const IGN_VECTOR3 &_pt) const
{
  return IGN_VECTOR3(this->data[1] * _pt[2] - this->data[2] * _pt[1],
                     this->data[2] * _pt[0] - this->data[0] * _pt[2],
                     this->data[0] * _pt[1] - this->data[1] * _pt[0]);
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR3::Dot(const IGN_VECTOR3 &_pt) const
{
  return this->data[0] * _pt[0] +
         this->data[1] * _pt[1] +
         this->data[2] * _pt[2];
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_VECTOR3::GetAbs() const
{
  return IGN_VECTOR3(std::abs(this->data[0]),
                     std::abs(this->data[1]),
                     std::abs(this->data[2]));
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_VECTOR3::GetPerpendicular() const
{
  static const IGN_NUMERIC sqrZero = 1e-06 * 1e-06;

  IGN_VECTOR3 perp = this->Cross(IGN_VECTOR3(1, 0, 0));

  // Check the length of the vector
  if (perp.GetSquaredLength() < sqrZero)
  {
    perp = this->Cross(IGN_VECTOR3(0, 1, 0));
  }

  return perp;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_VECTOR3::GetNormal(const IGN_VECTOR3 &_v1,
    const IGN_VECTOR3 &_v2, const IGN_VECTOR3 &_v3)
{
  IGN_VECTOR3 a = _v2 - _v1;
  IGN_VECTOR3 b = _v3 - _v1;
  IGN_VECTOR3 n = a.Cross(b);
  return n;
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR3::GetDistToLine(const IGN_VECTOR3 &_pt1,
    const IGN_VECTOR3 &_pt2)
{
  IGN_NUMERIC d = ((*this) - _pt1).Cross((*this) - _pt2).GetLength();
  d = d / (_pt2 - _pt1).GetLength();
  return d;
}

//////////////////////////////////////////////////
void IGN_VECTOR3::SetToMax(const IGN_VECTOR3 & _v)
{
  if (_v[0] > this->data[0])
    this->data[0] = _v[0];
  if (_v[1] > this->data[1])
    this->data[1] = _v[1];
  if (_v[2] > this->data[2])
    this->data[2] = _v[2];
}

//////////////////////////////////////////////////
void IGN_VECTOR3::SetToMin(const IGN_VECTOR3 & _v)
{
  if (_v[0] < this->data[0])
    this->data[0] = _v[0];
  if (_v[1] < this->data[1])
    this->data[1] = _v[1];
  if (_v[2] < this->data[2])
    this->data[2] = _v[2];
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR3::GetMax() const
{
  return std::max(std::max(this->data[0], this->data[1]), this->data[2]);
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR3::GetMin() const
{
  return std::min(std::min(this->data[0], this->data[1]), this->data[2]);
}

//////////////////////////////////////////////////
IGN_VECTOR3 &IGN_VECTOR3::operator =(const IGN_VECTOR3 &_pt)
{
  this->data[0] = _pt[0];
  this->data[1] = _pt[1];
  this->data[2] = _pt[2];

  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR3 &IGN_VECTOR3::operator =(IGN_NUMERIC value)
{
  this->data[0] = value;
  this->data[1] = value;
  this->data[2] = value;

  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_VECTOR3::operator+(const IGN_VECTOR3 &pt) const
{
  return IGN_VECTOR3(this->data[0] + pt[0],
                     this->data[1] + pt[1],
                     this->data[2] + pt[2]);
}

//////////////////////////////////////////////////
const IGN_VECTOR3 &IGN_VECTOR3::operator+=(const IGN_VECTOR3 &pt)
{
  this->data[0] += pt[0];
  this->data[1] += pt[1];
  this->data[2] += pt[2];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR3 &IGN_VECTOR3::operator-=(const IGN_VECTOR3 &pt)
{
  this->data[0] -= pt[0];
  this->data[1] -= pt[1];
  this->data[2] -= pt[2];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR3 IGN_VECTOR3::operator/(const IGN_VECTOR3 &pt) const
{
  return IGN_VECTOR3(this->data[0] / pt[0],
                     this->data[1] / pt[1],
                     this->data[2] / pt[2]);
}

//////////////////////////////////////////////////
const IGN_VECTOR3 &IGN_VECTOR3::operator/=(const IGN_VECTOR3 &pt)
{
  this->data[0] /= pt[0];
  this->data[1] /= pt[1];
  this->data[2] /= pt[2];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR3 IGN_VECTOR3::operator/(IGN_NUMERIC v) const
{
  return IGN_VECTOR3(this->data[0] / v, this->data[1] / v, this->data[2] / v);
}

//////////////////////////////////////////////////
const IGN_VECTOR3 &IGN_VECTOR3::operator/=(IGN_NUMERIC v)
{
  this->data[0] /= v;
  this->data[1] /= v;
  this->data[2] /= v;

  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_VECTOR3::operator*(const IGN_VECTOR3 &pt) const
{
  return IGN_VECTOR3(this->data[0] * pt[0],
                     this->data[1] * pt[1],
                     this->data[2] * pt[2]);
}

//////////////////////////////////////////////////
const IGN_VECTOR3 &IGN_VECTOR3::operator*=(const IGN_VECTOR3 &pt)
{
  this->data[0] *= pt[0];
  this->data[1] *= pt[1];
  this->data[2] *= pt[2];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR3 &IGN_VECTOR3::operator*=(IGN_NUMERIC v)
{
  this->data[0] *= v;
  this->data[1] *= v;
  this->data[2] *= v;

  return *this;
}

//////////////////////////////////////////////////
bool IGN_VECTOR3::operator ==(const IGN_VECTOR3 &_pt) const
{
  return equal(this->data[0], _pt[0], static_cast<IGN_NUMERIC>(0.001)) &&
         equal(this->data[1], _pt[1], static_cast<IGN_NUMERIC>(0.001)) &&
         equal(this->data[2], _pt[2], static_cast<IGN_NUMERIC>(0.001));
}

//////////////////////////////////////////////////
bool IGN_VECTOR3::operator!=(const IGN_VECTOR3 &_pt) const
{
  return !(*this == _pt);
}

//////////////////////////////////////////////////
bool IGN_VECTOR3::IsFinite() const
{
  return std::isfinite(this->data[0]) &&
         std::isfinite(this->data[1]) &&
         std::isfinite(this->data[2]);
}

//////////////////////////////////////////////////
/// Round all values to _decimalPlaces
void IGN_VECTOR3::Round(int _precision)
{
  this->data[0] = precision(this->data[0], _precision);
  this->data[1] = precision(this->data[1], _precision);
  this->data[2] = precision(this->data[2], _precision);
}

//////////////////////////////////////////////////
/// Returns true if the two vectors are exacatly equal
bool IGN_VECTOR3::Equal(const IGN_VECTOR3 &_v) const
{
  return math::equal(this->data[0], _v[0]) &&
         math::equal(this->data[1], _v[1]) &&
         math::equal(this->data[2], _v[2]);
}

//////////////////////////////////////////////////
std::ostream &ignition::math::operator<<(std::ostream &_out,
    const ignition::math::IGN_VECTOR3 &_pt)
{
  _out << precision(_pt[0], 6) << " " << precision(_pt[1], 6) << " "
    << precision(_pt[2], 6);
  return _out;
}

//////////////////////////////////////////////////
std::istream &ignition::math::operator>>(std::istream &_in,
    ignition::math::IGN_VECTOR3 &_pt)
{
  // Skip white spaces
  _in.setf(std::ios_base::skipws);
  IGN_NUMERIC x, y, z;
  _in >> x >> y >> z;
  _pt.Set(x, y, z);
  return _in;
}

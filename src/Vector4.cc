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

//////////////////////////////////////////////////
IGN_VECTOR4::IGN_VECTOR4()
{
  this->data[0] = this->data[1] = this->data[2] = this->data[3] = 0;
}

//////////////////////////////////////////////////
IGN_VECTOR4::IGN_VECTOR4(const IGN_NUMERIC &_x, const IGN_NUMERIC &_y,
    const IGN_NUMERIC &_z, const IGN_NUMERIC &_w)
{
  this->data[0] = _x;
  this->data[1] = _y;
  this->data[2] = _z;
  this->data[3] = _w;
}

//////////////////////////////////////////////////
IGN_VECTOR4::IGN_VECTOR4(const IGN_VECTOR4 &_pt)
{
  this->data[0] = _pt[0];
  this->data[1] = _pt[1];
  this->data[2] = _pt[2];
  this->data[3] = _pt[3];
}

//////////////////////////////////////////////////
IGN_VECTOR4::~IGN_VECTOR4()
{
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR4::Distance(const IGN_VECTOR4 &_pt) const
{
  return sqrt((this->data[0]-_pt[0])*(this->data[0]-_pt[0]) +
              (this->data[1]-_pt[1])*(this->data[1]-_pt[1]) +
              (this->data[2]-_pt[2])*(this->data[2]-_pt[2]) +
              (this->data[3]-_pt[3])*(this->data[3]-_pt[3]));
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR4::GetLength() const
{
  return sqrt(this->data[0] * this->data[0] + this->data[1] * this->data[1] +
              this->data[2] * this->data[2] + this->data[3] * this->data[3]);
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_VECTOR4::GetSquaredLength() const
{
  return this->data[0] * this->data[0] + this->data[1] * this->data[1] +
    this->data[2] * this->data[2] + this->data[3] * this->data[3];
}

//////////////////////////////////////////////////
void IGN_VECTOR4::Normalize()
{
  IGN_NUMERIC d = this->GetLength();

  this->data[0] /= d;
  this->data[1] /= d;
  this->data[2] /= d;
  this->data[3] /= d;
}

//////////////////////////////////////////////////
void IGN_VECTOR4::Set(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z,
    IGN_NUMERIC _w)
{
  this->data[0] = _x;
  this->data[1] = _y;
  this->data[2] = _z;
  this->data[3] = _w;
}


//////////////////////////////////////////////////
IGN_VECTOR4 &IGN_VECTOR4::operator =(const IGN_VECTOR4 &pt)
{
  this->data[0] = pt[0];
  this->data[1] = pt[1];
  this->data[2] = pt[2];
  this->data[3] = pt[3];

  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR4 &IGN_VECTOR4::operator =(IGN_NUMERIC value)
{
  this->data[0] = value;
  this->data[1] = value;
  this->data[2] = value;
  this->data[3] = value;

  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR4 IGN_VECTOR4::operator+(const IGN_VECTOR4 &pt) const
{
  return IGN_VECTOR4(this->data[0] + pt[0],
      this->data[1] + pt[1],
      this->data[2] + pt[2],
      this->data[3] + pt[3]);
}

//////////////////////////////////////////////////
const IGN_VECTOR4 &IGN_VECTOR4::operator+=(const IGN_VECTOR4 &pt)
{
  this->data[0] += pt[0];
  this->data[1] += pt[1];
  this->data[2] += pt[2];
  this->data[3] += pt[3];

  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR4 IGN_VECTOR4::operator-(const IGN_VECTOR4 &pt) const
{
  return IGN_VECTOR4(this->data[0] - pt[0],
      this->data[1] - pt[1],
      this->data[2] - pt[2],
      this->data[3] - pt[3]);
}

//////////////////////////////////////////////////
const IGN_VECTOR4 &IGN_VECTOR4::operator-=(const IGN_VECTOR4 &pt)
{
  this->data[0] -= pt[0];
  this->data[1] -= pt[1];
  this->data[2] -= pt[2];
  this->data[3] -= pt[3];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR4 IGN_VECTOR4::operator/(const IGN_VECTOR4 &pt) const
{
  return IGN_VECTOR4(this->data[0] / pt[0],
      this->data[1] / pt[1],
      this->data[2] / pt[2],
      this->data[3]/pt[3]);
}

//////////////////////////////////////////////////
const IGN_VECTOR4 &IGN_VECTOR4::operator/=(const IGN_VECTOR4 &pt)
{
  this->data[0] /= pt[0];
  this->data[1] /= pt[1];
  this->data[2] /= pt[2];
  this->data[3] /= pt[3];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR4 IGN_VECTOR4::operator/(IGN_NUMERIC v) const
{
  return IGN_VECTOR4(this->data[0] / v, this->data[1] / v,
      this->data[2] / v, this->data[3] / v);
}

//////////////////////////////////////////////////
const IGN_VECTOR4 &IGN_VECTOR4::operator/=(IGN_NUMERIC v)
{
  this->data[0] /= v;
  this->data[1] /= v;
  this->data[2] /= v;
  this->data[3] /= v;

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR4 IGN_VECTOR4::operator*(const IGN_VECTOR4 &pt) const
{
  return IGN_VECTOR4(this->data[0] * pt[0],
      this->data[1] * pt[1],
      this->data[2] * pt[2],
      this->data[3] * pt[3]);
}

//////////////////////////////////////////////////
const IGN_VECTOR4 IGN_VECTOR4::operator*(const IGN_MATRIX4 &_m) const
{
  return IGN_VECTOR4(
      this->data[0]*_m(0, 0) + this->data[1]*_m(1, 0) +
      this->data[2]*_m(2, 0) + this->data[3]*_m(3, 0),
      this->data[0]*_m(0, 1) + this->data[1]*_m(1, 1) +
      this->data[2]*_m(2, 1) + this->data[3]*_m(3, 1),
      this->data[0]*_m(0, 2) + this->data[1]*_m(1, 2) +
      this->data[2]*_m(2, 2) + this->data[3]*_m(3, 2),
      this->data[0]*_m(0, 3) + this->data[1]*_m(1, 3) +
      this->data[2]*_m(2, 3) + this->data[3]*_m(3, 3));
}

//////////////////////////////////////////////////
const IGN_VECTOR4 &IGN_VECTOR4::operator*=(const IGN_VECTOR4 &pt)
{
  this->data[0] *= pt[0];
  this->data[1] *= pt[1];
  this->data[2] *= pt[2];
  this->data[3] *= pt[3];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR4 IGN_VECTOR4::operator*(IGN_NUMERIC v) const
{
  return IGN_VECTOR4(this->data[0] * v, this->data[1] * v,
      this->data[2] * v, this->data[3]*v);
}

//////////////////////////////////////////////////
const IGN_VECTOR4 &IGN_VECTOR4::operator*=(IGN_NUMERIC v)
{
  this->data[0] *= v;
  this->data[1] *= v;
  this->data[2] *= v;
  this->data[3] *= v;

  return *this;
}

//////////////////////////////////////////////////
bool IGN_VECTOR4::operator ==(const IGN_VECTOR4 &pt) const
{
  return equal(this->data[0], pt[0]) && equal(this->data[1], pt[1]) &&
         equal(this->data[2], pt[2]) && equal(this->data[3], pt[3]);
}

//////////////////////////////////////////////////
bool IGN_VECTOR4::operator!=(const IGN_VECTOR4 &pt) const
{
  return !(*this == pt);
}

//////////////////////////////////////////////////
bool IGN_VECTOR4::IsFinite() const
{
  // std::isfinite works with floating point values, need to explicit cast to
  // avoid ambiguity in vc++.
  return std::isfinite(static_cast<double>(this->data[0])) &&
         std::isfinite(static_cast<double>(this->data[1])) &&
         std::isfinite(static_cast<double>(this->data[2])) &&
         std::isfinite(static_cast<double>(this->data[3]));
}

//////////////////////////////////////////////////
std::ostream &ignition::math::operator<<(std::ostream &_out,
    const ignition::math::IGN_VECTOR4 &_pt)
{
  _out << _pt[0] << " " << _pt[1] << " " << _pt[2] << " " << _pt[3];
  return _out;
}

//////////////////////////////////////////////////
std::istream &ignition::math::operator>>(std::istream &_in,
    ignition::math::IGN_VECTOR4 &_pt)
{
  IGN_NUMERIC x, y, z, w;

  // Skip white spaces
  _in.setf(std::ios_base::skipws);
  _in >> x >> y >> z >> w;
  _pt.Set(x, y, z, w);
  return _in;
}

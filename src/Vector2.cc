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
IGN_VECTOR2::IGN_VECTOR2()
{
  this->data[0] = 0;
  this->data[1] = 0;
}

//////////////////////////////////////////////////
IGN_VECTOR2::IGN_VECTOR2(const IGN_NUMERIC &_x, const IGN_NUMERIC &_y)
{
  this->data[0] = _x;
  this->data[1] = _y;
}

//////////////////////////////////////////////////
IGN_VECTOR2::IGN_VECTOR2(const IGN_VECTOR2 &_pt)
{
  this->data[0] = _pt[0];
  this->data[1] = _pt[1];
}

//////////////////////////////////////////////////
IGN_VECTOR2::~IGN_VECTOR2()
{
}

//////////////////////////////////////////////////
double IGN_VECTOR2::Distance(const IGN_VECTOR2 &_pt) const
{
  return sqrt((this->data[0]-_pt[0])*(this->data[0]-_pt[0]) +
              (this->data[1]-_pt[1])*(this->data[1]-_pt[1]));
}

//////////////////////////////////////////////////
void IGN_VECTOR2::Normalize()
{
  double d = sqrt(this->data[0] * this->data[0] +
                  this->data[1] * this->data[1]);

  this->data[0] /= d;
  this->data[1] /= d;
}

//////////////////////////////////////////////////
void IGN_VECTOR2::Set(IGN_NUMERIC _x, IGN_NUMERIC _y)
{
  this->data[0] = _x;
  this->data[1] = _y;
}

//////////////////////////////////////////////////
IGN_VECTOR2 &IGN_VECTOR2::operator =(const IGN_VECTOR2 &_pt)
{
  this->data[0] = _pt[0];
  this->data[1] = _pt[1];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR2 &IGN_VECTOR2::operator =(IGN_NUMERIC value)
{
  this->data[0] = value;
  this->data[1] = value;

  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR2 IGN_VECTOR2::operator+(const IGN_VECTOR2 &pt) const
{
  return IGN_VECTOR2(this->data[0] + pt[0], this->data[1] + pt[1]);
}

//////////////////////////////////////////////////
const IGN_VECTOR2 &IGN_VECTOR2::operator+=(const IGN_VECTOR2 &pt)
{
  this->data[0] += pt[0];
  this->data[1] += pt[1];

  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR2 IGN_VECTOR2::operator-(const IGN_VECTOR2 &pt) const
{
  return IGN_VECTOR2(this->data[0] - pt[0], this->data[1] - pt[1]);
}

//////////////////////////////////////////////////
const IGN_VECTOR2 &IGN_VECTOR2::operator-=(const IGN_VECTOR2 &pt)
{
  this->data[0] -= pt[0];
  this->data[1] -= pt[1];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR2 IGN_VECTOR2::operator/(const IGN_VECTOR2 &pt) const
{
  return IGN_VECTOR2(this->data[0] / pt[0], this->data[1] / pt[1]);
}

//////////////////////////////////////////////////
const IGN_VECTOR2 &IGN_VECTOR2::operator/=(const IGN_VECTOR2 &pt)
{
  this->data[0] /= pt[0];
  this->data[1] /= pt[1];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR2 IGN_VECTOR2::operator/(IGN_NUMERIC v) const
{
  return IGN_VECTOR2(this->data[0] / v, this->data[1] / v);
}

//////////////////////////////////////////////////
const IGN_VECTOR2 &IGN_VECTOR2::operator/=(IGN_NUMERIC v)
{
  this->data[0] /= v;
  this->data[1] /= v;

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR2 IGN_VECTOR2::operator*(const IGN_VECTOR2 &pt) const
{
  return IGN_VECTOR2(this->data[0] * pt[0], this->data[1] * pt[1]);
}

//////////////////////////////////////////////////
const IGN_VECTOR2 &IGN_VECTOR2::operator*=(const IGN_VECTOR2 &pt)
{
  this->data[0] *= pt[0];
  this->data[1] *= pt[1];

  return *this;
}

//////////////////////////////////////////////////
const IGN_VECTOR2 IGN_VECTOR2::operator*(IGN_NUMERIC v) const
{
  return IGN_VECTOR2(this->data[0] * v, this->data[1] * v);
}

//////////////////////////////////////////////////
const IGN_VECTOR2 &IGN_VECTOR2::operator*=(IGN_NUMERIC v)
{
  this->data[0] *= v;
  this->data[1] *= v;

  return *this;
}

//////////////////////////////////////////////////
bool IGN_VECTOR2::operator ==(const IGN_VECTOR2 &pt) const
{
  return equal(this->data[0], pt[0]) && equal(this->data[1], pt[1]);
}

//////////////////////////////////////////////////
bool IGN_VECTOR2::IsFinite() const
{
  return finite(this->data[0]) && finite(this->data[1]);
}

//////////////////////////////////////////////////
std::ostream &ignition::math::operator<<(std::ostream &_out,
    const ignition::math::IGN_VECTOR2 &_pt)
{
  _out << _pt[0] << " " << _pt[1];
  return _out;
}

//////////////////////////////////////////////////
std::istream &ignition::math::operator>>(std::istream &_in,
    ignition::math::IGN_VECTOR2 &_pt)
{
  IGN_NUMERIC x, y;
  // Skip white spaces
  _in.setf(std::ios_base::skipws);
  _in >> x >> y;
  _pt.Set(x, y);
  return _in;
}

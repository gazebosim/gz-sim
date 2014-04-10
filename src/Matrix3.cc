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
const IGN_MATRIX3 IGN_MATRIX3::Identity(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);

const IGN_MATRIX3 IGN_MATRIX3::Zero(
    0, 0, 0,
    0, 0, 0,
    0, 0, 0);

//////////////////////////////////////////////////
IGN_MATRIX3::IGN_MATRIX3()
{
  std::memset(this->data, 0, sizeof(this->data[0][0])*9);
}

//////////////////////////////////////////////////
IGN_MATRIX3::IGN_MATRIX3(const IGN_MATRIX3 &_m)
{
  std::memcpy(this->data, _m.data, sizeof(this->data[0][0])*9);
}

//////////////////////////////////////////////////
IGN_MATRIX3::IGN_MATRIX3(IGN_NUMERIC _v00, IGN_NUMERIC _v01, IGN_NUMERIC _v02,
                 IGN_NUMERIC _v10, IGN_NUMERIC _v11, IGN_NUMERIC _v12,
                 IGN_NUMERIC _v20, IGN_NUMERIC _v21, IGN_NUMERIC _v22)
{
  this->data[0][0] = _v00;
  this->data[0][1] = _v01;
  this->data[0][2] = _v02;
  this->data[1][0] = _v10;
  this->data[1][1] = _v11;
  this->data[1][2] = _v12;
  this->data[2][0] = _v20;
  this->data[2][1] = _v21;
  this->data[2][2] = _v22;
}

//////////////////////////////////////////////////
IGN_MATRIX3::~IGN_MATRIX3()
{
}

//////////////////////////////////////////////////
void IGN_MATRIX3::Set(IGN_NUMERIC _v00, IGN_NUMERIC _v01, IGN_NUMERIC _v02,
                 IGN_NUMERIC _v10, IGN_NUMERIC _v11, IGN_NUMERIC _v12,
                 IGN_NUMERIC _v20, IGN_NUMERIC _v21, IGN_NUMERIC _v22)
{
  this->data[0][0] = _v00;
  this->data[0][1] = _v01;
  this->data[0][2] = _v02;
  this->data[1][0] = _v10;
  this->data[1][1] = _v11;
  this->data[1][2] = _v12;
  this->data[2][0] = _v20;
  this->data[2][1] = _v21;
  this->data[2][2] = _v22;
}

//////////////////////////////////////////////////
void IGN_MATRIX3::SetFromAxes(const IGN_VECTOR3 &_xAxis,
    const IGN_VECTOR3 &_yAxis,
    const IGN_VECTOR3 &_zAxis)
{
  this->SetCol(0, _xAxis);
  this->SetCol(1, _yAxis);
  this->SetCol(2, _zAxis);
}

//////////////////////////////////////////////////
void IGN_MATRIX3::SetFromAxis(const IGN_VECTOR3 &_axis, IGN_NUMERIC _angle)
{
  IGN_NUMERIC c = cos(_angle);
  IGN_NUMERIC s = sin(_angle);
  IGN_NUMERIC C = 1-c;

  this->data[0][0] = _axis.x()*_axis.x()*C + c;
  this->data[0][1] = _axis.x()*_axis.y()*C - _axis.z()*s;
  this->data[0][2] = _axis.x()*_axis.z()*C + _axis.y()*s;

  this->data[1][0] = _axis.y()*_axis.x()*C + _axis.z()*s;
  this->data[1][1] = _axis.y()*_axis.y()*C + c;
  this->data[1][2] = _axis.y()*_axis.z()*C - _axis.x()*s;

  this->data[2][0] = _axis.z()*_axis.x()*C - _axis.y()*s;
  this->data[2][1] = _axis.z()*_axis.y()*C + _axis.x()*s;
  this->data[2][2] = _axis.z()*_axis.z()*C + c;
}

//////////////////////////////////////////////////
void IGN_MATRIX3::SetCol(unsigned int _i, const IGN_VECTOR3 &_v)
{
  if (_i >= 3)
    throw IndexException();

  this->data[0][_i] = _v.x();
  this->data[1][_i] = _v.y();
  this->data[2][_i] = _v.z();
}

//////////////////////////////////////////////////
bool IGN_MATRIX3::operator==(const IGN_MATRIX3 &_m) const
{
  return math::equal(this->data[0][0], _m(0, 0)) &&
         math::equal(this->data[0][1], _m(0, 1)) &&
         math::equal(this->data[0][2], _m(0, 2)) &&

         math::equal(this->data[1][0], _m(1, 0)) &&
         math::equal(this->data[1][1], _m(1, 1)) &&
         math::equal(this->data[1][2], _m(1, 2)) &&

         math::equal(this->data[2][0], _m(2, 0)) &&
         math::equal(this->data[2][1], _m(2, 1)) &&
         math::equal(this->data[2][2], _m(2, 2));
}

//////////////////////////////////////////////////
IGN_MATRIX3 IGN_MATRIX3::operator-(const IGN_MATRIX3 &_m) const
{
  return IGN_MATRIX3(
      this->data[0][0] - _m(0, 0),
      this->data[0][1] - _m(0, 1),
      this->data[0][2] - _m(0, 2),
      this->data[1][0] - _m(1, 0),
      this->data[1][1] - _m(1, 1),
      this->data[1][2] - _m(1, 2),
      this->data[2][0] - _m(2, 0),
      this->data[2][1] - _m(2, 1),
      this->data[2][2] - _m(2, 2));
}

//////////////////////////////////////////////////
IGN_MATRIX3 IGN_MATRIX3::operator+(const IGN_MATRIX3 &_m) const
{
  return IGN_MATRIX3(
      this->data[0][0]+_m(0, 0),
      this->data[0][1]+_m(0, 1),
      this->data[0][2]+_m(0, 2),
      this->data[1][0]+_m(1, 0),
      this->data[1][1]+_m(1, 1),
      this->data[1][2]+_m(1, 2),
      this->data[2][0]+_m(2, 0),
      this->data[2][1]+_m(2, 1),
      this->data[2][2]+_m(2, 2));
}

//////////////////////////////////////////////////
IGN_MATRIX3 IGN_MATRIX3::operator*(const IGN_NUMERIC &_s) const
{
  return IGN_MATRIX3(
      _s * this->data[0][0], _s * this->data[0][1], _s * this->data[0][2],
      _s * this->data[1][0], _s * this->data[1][1], _s * this->data[1][2],
      _s * this->data[2][0], _s * this->data[2][1], _s * this->data[2][2]);
}

//////////////////////////////////////////////////
IGN_MATRIX3 IGN_MATRIX3::operator*(const IGN_MATRIX3 &_m) const
{
  return IGN_MATRIX3(
      // first row
      this->data[0][0]*_m(0, 0)+
      this->data[0][1]*_m(1, 0)+
      this->data[0][2]*_m(2, 0),

      this->data[0][0]*_m(0, 1)+
      this->data[0][1]*_m(1, 1)+
      this->data[0][2]*_m(2, 1),

      this->data[0][0]*_m(0, 2)+
      this->data[0][1]*_m(1, 2)+
      this->data[0][2]*_m(2, 2),

      // second row
      this->data[1][0]*_m(0, 0)+
      this->data[1][1]*_m(1, 0)+
      this->data[1][2]*_m(2, 0),

      this->data[1][0]*_m(0, 1)+
      this->data[1][1]*_m(1, 1)+
      this->data[1][2]*_m(2, 1),

      this->data[1][0]*_m(0, 2)+
      this->data[1][1]*_m(1, 2)+
      this->data[1][2]*_m(2, 2),

      // third row
      this->data[2][0]*_m(0, 0)+
      this->data[2][1]*_m(1, 0)+
      this->data[2][2]*_m(2, 0),

      this->data[2][0]*_m(0, 1)+
      this->data[2][1]*_m(1, 1)+
      this->data[2][2]*_m(2, 1),

      this->data[2][0]*_m(0, 2)+
      this->data[2][1]*_m(1, 2)+
      this->data[2][2]*_m(2, 2));
}

//////////////////////////////////////////////////
std::ostream &ignition::math::operator<<(std::ostream &_out,
    const ignition::math::IGN_MATRIX3 &_m)
{
  _out << precision(_m(0, 0), 6) << " "
       << precision(_m(0, 1), 6) << " "
       << precision(_m(0, 2), 6) << " "
       << precision(_m(1, 0), 6) << " "
       << precision(_m(1, 1), 6) << " "
       << precision(_m(1, 2), 6) << " "
       << precision(_m(2, 0), 6) << " "
       << precision(_m(2, 1), 6) << " "
       << precision(_m(2, 2), 6);

  return _out;
}

//////////////////////////////////////////////////
std::istream &ignition::math::operator>>(std::istream &_in,
    ignition::math::IGN_MATRIX3 &_m)
{
  // Skip white spaces
  _in.setf(std::ios_base::skipws);
  IGN_NUMERIC d[9];
  _in >> d[0] >> d[1] >> d[2]
      >> d[3] >> d[4] >> d[5]
      >> d[6] >> d[7] >> d[8];

  _m.Set(d[0], d[1], d[2],
         d[3], d[4], d[5],
         d[6], d[7], d[8]);
  return _in;
}

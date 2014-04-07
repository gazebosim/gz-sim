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
#include <string.h>
#include <math.h>

#include "ignition/math/Helpers.hh"
#include "ignition/math/Matrix3.hh"

using namespace ignition;
using namespace math;

//////////////////////////////////////////////////
Matrix3::Matrix3()
{
  memset(this->m, 0, sizeof(this->m[0][0])*9);
}

//////////////////////////////////////////////////
Matrix3::Matrix3(const Matrix3 &_m)
{
  memcpy(this->m, _m.m, sizeof(this->m[0][0])*9);
}

//////////////////////////////////////////////////
Matrix3::Matrix3(double _v00, double _v01, double _v02,
                 double _v10, double _v11, double _v12,
                 double _v20, double _v21, double _v22)
{
  this->m[0][0] = _v00;
  this->m[0][1] = _v01;
  this->m[0][2] = _v02;
  this->m[1][0] = _v10;
  this->m[1][1] = _v11;
  this->m[1][2] = _v12;
  this->m[2][0] = _v20;
  this->m[2][1] = _v21;
  this->m[2][2] = _v22;
}


//////////////////////////////////////////////////
Matrix3::~Matrix3()
{
}

//////////////////////////////////////////////////
void Matrix3::SetFromAxes(const Vector3d &xAxis, const Vector3d &yAxis,
                          const Vector3d &zAxis)
{
  this->SetCol(0, xAxis);
  this->SetCol(1, yAxis);
  this->SetCol(2, zAxis);
}

//////////////////////////////////////////////////
void Matrix3::SetFromAxis(const Vector3d &_axis, double _angle)
{
  double c = cos(_angle);
  double s = sin(_angle);
  double C = 1-c;

  this->m[0][0] = _axis.x()*_axis.x()*C + c;
  this->m[0][1] = _axis.x()*_axis.y()*C - _axis.z()*s;
  this->m[0][2] = _axis.x()*_axis.z()*C + _axis.y()*s;

  this->m[1][0] = _axis.y()*_axis.x()*C + _axis.z()*s;
  this->m[1][1] = _axis.y()*_axis.y()*C + c;
  this->m[1][2] = _axis.y()*_axis.z()*C - _axis.x()*s;

  this->m[2][0] = _axis.z()*_axis.x()*C - _axis.y()*s;
  this->m[2][1] = _axis.z()*_axis.y()*C + _axis.x()*s;
  this->m[2][2] = _axis.z()*_axis.z()*C + c;
}

//////////////////////////////////////////////////
void Matrix3::SetCol(unsigned int _i, const Vector3d &_v)
{
  if (_i >= 3)
    throw(std::string("Invalid column number"));

  m[0][_i] = _v.x();
  m[1][_i] = _v.y();
  m[2][_i] = _v.z();
}

//////////////////////////////////////////////////
bool Matrix3::operator==(const Matrix3 &_m) const
{
  return math::equal(this->m[0][0], _m[0][0]) &&
         math::equal(this->m[0][1], _m[0][1]) &&
         math::equal(this->m[0][2], _m[0][2]) &&

         math::equal(this->m[1][0], _m[1][0]) &&
         math::equal(this->m[1][1], _m[1][1]) &&
         math::equal(this->m[1][2], _m[1][2]) &&

         math::equal(this->m[2][0], _m[2][0]) &&
         math::equal(this->m[2][1], _m[2][1]) &&
         math::equal(this->m[2][2], _m[2][2]);
}

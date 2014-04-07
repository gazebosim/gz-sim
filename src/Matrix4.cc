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

#include "ignition/math/Helpers.hh"
#include "ignition/math/Matrix4.hh"
#include "ignition/math/Quaternion.hh"
#include "ignition/math/Pose.hh"

using namespace ignition;
using namespace math;

const Matrix4 Matrix4::IDENTITY(
       1.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 1.0);


const Matrix4 Matrix4::ZERO(
       0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0);


//////////////////////////////////////////////////
Matrix4::Matrix4()
{
  memset(this->m, 0, sizeof(this->m[0][0])*16);
}

//////////////////////////////////////////////////
Matrix4::Matrix4(const Matrix4 &_m)
{
  memcpy(this->m, _m.m, sizeof(this->m[0][0])*16);
}

//////////////////////////////////////////////////
Matrix4::Matrix4(double _v00, double _v01, double _v02, double _v03,
                 double _v10, double _v11, double _v12, double _v13,
                 double _v20, double _v21, double _v22, double _v23,
                 double _v30, double _v31, double _v32, double _v33)
{
  this->Set(_v00, _v01, _v02, _v03,
            _v10, _v11, _v12, _v13,
            _v20, _v21, _v22, _v23,
            _v30, _v31, _v32, _v33);
}

//////////////////////////////////////////////////
Matrix4::~Matrix4()
{
}

//////////////////////////////////////////////////
void Matrix4::Set(double _v00, double _v01, double _v02, double _v03,
                  double _v10, double _v11, double _v12, double _v13,
                  double _v20, double _v21, double _v22, double _v23,
                  double _v30, double _v31, double _v32, double _v33)
{
  this->m[0][0] = _v00;
  this->m[0][1] = _v01;
  this->m[0][2] = _v02;
  this->m[0][3] = _v03;

  this->m[1][0] = _v10;
  this->m[1][1] = _v11;
  this->m[1][2] = _v12;
  this->m[1][3] = _v13;

  this->m[2][0] = _v20;
  this->m[2][1] = _v21;
  this->m[2][2] = _v22;
  this->m[2][3] = _v23;

  this->m[3][0] = _v30;
  this->m[3][1] = _v31;
  this->m[3][2] = _v32;
  this->m[3][3] = _v33;
}

//////////////////////////////////////////////////
void Matrix4::SetTranslate(const Vector3d &_t)
{
  this->m[0][3] = _t.x();
  this->m[1][3] = _t.y();
  this->m[2][3] = _t.z();
}

//////////////////////////////////////////////////
void Matrix4::SetTranslate(double _x, double _y, double _z)
{
  this->m[0][3] = _x;
  this->m[1][3] = _y;
  this->m[2][3] = _z;
}

//////////////////////////////////////////////////
Vector3d Matrix4::GetTranslation() const
{
  return Vector3d(this->m[0][3], this->m[1][3], this->m[2][3]);
}

//////////////////////////////////////////////////
Vector3d Matrix4::GetScale() const
{
  return Vector3d(this->m[0][0], this->m[1][1], this->m[2][2]);
}

//////////////////////////////////////////////////
Quaternion Matrix4::GetRotation() const
{
  Quaternion q;
  /// algorithm from Ogre::Quaternion source, which in turn is based on
  /// Ken Shoemake's article "Quaternion Calculus and Fast Animation".
  double trace = this->m[0][0] + this->m[1][1] + this->m[2][2];
  double root;
  if (trace > 0)
  {
    root = sqrt(trace + 1.0);
    q.w = root / 2.0;
    root = 1.0 / (2.0 * root);
    q.x = (this->m[2][1] - this->m[1][2]) * root;
    q.y = (this->m[0][2] - this->m[2][0]) * root;
    q.z = (this->m[1][0] - this->m[0][1]) * root;
  }
  else
  {
    static unsigned int s_iNext[3] = {1, 2, 0};
    unsigned int i = 0;
    if (this->m[1][1] > this->m[0][0])
      i = 1;
    if (this->m[2][2] > this->m[i][i])
      i = 2;
    unsigned int j = s_iNext[i];
    unsigned int k = s_iNext[j];

    root = sqrt(this->m[i][i] - this->m[j][j] - this->m[k][k] + 1.0);
    double* xyzQ[3] = { &q.x, &q.y, &q.z};
    *xyzQ[i] = root / 2.0;
    root = 1.0 / (2.0 * root);
    q.w = (this->m[k][j] - this->m[j][k]) * root;
    *xyzQ[j] = (this->m[j][i] + this->m[i][j]) * root;
    *xyzQ[k] = (this->m[k][i] + this->m[i][k]) * root;
  }

  return q;
}

//////////////////////////////////////////////////
Vector3d Matrix4::GetEulerRotation(bool _firstSolution) const
{
  Vector3d euler;
  Vector3d euler2;

  double m31 = this->m[2][0];
  double m11 = this->m[0][0];
  double m12 = this->m[0][1];
  double m13 = this->m[0][2];
  double m32 = this->m[2][1];
  double m33 = this->m[2][2];
  double m21 = this->m[1][0];

  if (fabs(m31) >= 1.0)
  {
    euler.z(0.0);
    euler2.z(0.0);

    if (m31 < 0.0)
    {
      euler.y(M_PI / 2.0);
      euler2.y(M_PI / 2.0);
      euler.x(atan2(m12, m13));
      euler2.x(atan2(m12, m13));
    }
    else
    {
      euler.y(-M_PI / 2.0);
      euler2.y(-M_PI / 2.0);
      euler.x(atan2(-m12, -m13));
      euler2.x(atan2(-m12, -m13));
    }
  }
  else
  {
    euler.y(-asin(m31));
    euler2.y(M_PI - euler.y());

    euler.x(atan2(m32 / cos(euler.y()), m33 / cos(euler.y())));
    euler2.x(atan2(m32 / cos(euler2.y()), m33 / cos(euler2.y())));

    euler.z(atan2(m21 / cos(euler.y()), m11 / cos(euler.y())));
    euler2.z(atan2(m21 / cos(euler2.y()), m11 / cos(euler2.y())));
  }

  if (_firstSolution)
    return euler;
  else
    return euler2;
}

//////////////////////////////////////////////////
void Matrix4::SetScale(const Vector3d &_s)
{
  this->m[0][0] = _s.x();
  this->m[1][1] = _s.y();
  this->m[2][2] = _s.z();
  this->m[3][3] = 1.0;
}

//////////////////////////////////////////////////
void Matrix4::SetScale(double _x, double _y, double _z)
{
  this->m[0][0] = _x;
  this->m[1][1] = _y;
  this->m[2][2] = _z;
  this->m[3][3] = 1.0;
}

//////////////////////////////////////////////////
Matrix4 &Matrix4::operator =(const Matrix4 &_mat)
{
  memcpy(this->m, _mat.m, sizeof(this->m[0][0])*16);
  return *this;
}

//////////////////////////////////////////////////
const Matrix4 &Matrix4::operator =(const Matrix3 &mat)
{
  this->m[0][0] = mat.m[0][0];
  this->m[0][1] = mat.m[0][1];
  this->m[0][2] = mat.m[0][2];

  this->m[1][0] = mat.m[1][0];
  this->m[1][1] = mat.m[1][1];
  this->m[1][2] = mat.m[1][2];

  this->m[2][0] = mat.m[2][0];
  this->m[2][1] = mat.m[2][1];
  this->m[2][2] = mat.m[2][2];

  return *this;
}


//////////////////////////////////////////////////
Matrix4 Matrix4::operator*(const Matrix3 &m2) const
{
  Matrix4 r;
  r = *this;

  r.m[0][0] = m[0][0]*m2.m[0][0] + m[0][1]*m2.m[1][0] + m[0][2] * m2.m[2][0];
  r.m[0][1] = m[0][0]*m2.m[0][1] + m[0][1]*m2.m[1][1] + m[0][2] * m2.m[2][1];
  r.m[0][2] = m[0][0]*m2.m[0][2] + m[0][1]*m2.m[1][2] + m[0][2] * m2.m[2][2];

  r.m[1][0] = m[1][0]*m2.m[0][0] + m[1][1]*m2.m[1][0] + m[1][2] * m2.m[2][0];
  r.m[1][1] = m[1][0]*m2.m[0][1] + m[1][1]*m2.m[1][1] + m[1][2] * m2.m[2][1];
  r.m[1][2] = m[1][0]*m2.m[0][2] + m[1][1]*m2.m[1][2] + m[1][2] * m2.m[2][2];

  r.m[2][0] = m[2][0]*m2.m[0][0] + m[2][1]*m2.m[1][0] + m[2][2] * m2.m[2][0];
  r.m[2][1] = m[2][0]*m2.m[0][1] + m[2][1]*m2.m[1][1] + m[2][2] * m2.m[2][1];
  r.m[2][2] = m[2][0]*m2.m[0][2] + m[2][1]*m2.m[1][2] + m[2][2] * m2.m[2][2];

  return r;
}

//////////////////////////////////////////////////
Matrix4 Matrix4::operator*(const Matrix4 &m2) const
{
  Matrix4 r;

  r.m[0][0] = this->m[0][0] * m2.m[0][0] +
              this->m[0][1] * m2.m[1][0] +
              this->m[0][2] * m2.m[2][0] +
              this->m[0][3] * m2.m[3][0];

  r.m[0][1] = this->m[0][0] * m2.m[0][1] +
              this->m[0][1] * m2.m[1][1] +
              this->m[0][2] * m2.m[2][1] +
              this->m[0][3] * m2.m[3][1];

  r.m[0][2] = this->m[0][0] * m2.m[0][2] +
              this->m[0][1] * m2.m[1][2] +
              this->m[0][2] * m2.m[2][2] +
              this->m[0][3] * m2.m[3][2];

  r.m[0][3] = this->m[0][0] * m2.m[0][3] +
              this->m[0][1] * m2.m[1][3] +
              this->m[0][2] * m2.m[2][3] +
              this->m[0][3] * m2.m[3][3];

  r.m[1][0] = this->m[1][0] * m2.m[0][0] +
              this->m[1][1] * m2.m[1][0] +
              this->m[1][2] * m2.m[2][0] +
              this->m[1][3] * m2.m[3][0];

  r.m[1][1] = this->m[1][0] * m2.m[0][1] +
              this->m[1][1] * m2.m[1][1] +
              this->m[1][2] * m2.m[2][1] +
              this->m[1][3] * m2.m[3][1];

  r.m[1][2] = this->m[1][0] * m2.m[0][2] +
              this->m[1][1] * m2.m[1][2] +
              this->m[1][2] * m2.m[2][2] +
              this->m[1][3] * m2.m[3][2];

  r.m[1][3] = this->m[1][0] * m2.m[0][3] +
              this->m[1][1] * m2.m[1][3] +
              this->m[1][2] * m2.m[2][3] +
              this->m[1][3] * m2.m[3][3];

  r.m[2][0] = this->m[2][0] * m2.m[0][0] +
              this->m[2][1] * m2.m[1][0] +
              this->m[2][2] * m2.m[2][0] +
              this->m[2][3] * m2.m[3][0];

  r.m[2][1] = this->m[2][0] * m2.m[0][1] +
              this->m[2][1] * m2.m[1][1] +
              this->m[2][2] * m2.m[2][1] +
              this->m[2][3] * m2.m[3][1];

  r.m[2][2] = this->m[2][0] * m2.m[0][2] +
              this->m[2][1] * m2.m[1][2] +
              this->m[2][2] * m2.m[2][2] +
              this->m[2][3] * m2.m[3][2];

  r.m[2][3] = this->m[2][0] * m2.m[0][3] +
              this->m[2][1] * m2.m[1][3] +
              this->m[2][2] * m2.m[2][3] +
              this->m[2][3] * m2.m[3][3];

  r.m[3][0] = this->m[3][0] * m2.m[0][0] +
              this->m[3][1] * m2.m[1][0] +
              this->m[3][2] * m2.m[2][0] +
              this->m[3][3] * m2.m[3][0];

  r.m[3][1] = this->m[3][0] * m2.m[0][1] +
              this->m[3][1] * m2.m[1][1] +
              this->m[3][2] * m2.m[2][1] +
              this->m[3][3] * m2.m[3][1];

  r.m[3][2] = this->m[3][0] * m2.m[0][2] +
              this->m[3][1] * m2.m[1][2] +
              this->m[3][2] * m2.m[2][2] +
              this->m[3][3] * m2.m[3][2];

  r.m[3][3] = this->m[3][0] * m2.m[0][3] +
              this->m[3][1] * m2.m[1][3] +
              this->m[3][2] * m2.m[2][3] +
              this->m[3][3] * m2.m[3][3];

  return r;
}

//////////////////////////////////////////////////
Vector3d Matrix4::operator*(const Vector3d &_vec) const
{
  return Vector3d(this->m[0][0]*_vec.x() + this->m[0][1]*_vec.y() +
                  this->m[0][2]*_vec.z() + this->m[0][3],
                  this->m[1][0]*_vec.x() + this->m[1][1]*_vec.y() +
                  this->m[1][2]*_vec.z() + this->m[1][3],
                  this->m[2][0]*_vec.x() + this->m[2][1]*_vec.y() +
                  this->m[2][2]*_vec.z() + this->m[2][3]);
}

//////////////////////////////////////////////////
bool Matrix4::IsAffine() const
{
  return equal(this->m[3][0], 0.0) && equal(this->m[3][1], 0.0) &&
         equal(this->m[3][2], 0.0) && equal(this->m[3][3], 1.0);
}

//////////////////////////////////////////////////
Vector3d Matrix4::TransformAffine(const Vector3d &_v) const
{
  if (!this->IsAffine())
  {
    throw(std::string("Not and affine matrix"));
  }

  return Vector3d(this->m[0][0]*_v.x() + this->m[0][1]*_v.y() +
                  this->m[0][2]*_v.z() + this->m[0][3],
                  this->m[1][0]*_v.x() + this->m[1][1]*_v.y() +
                  this->m[1][2]*_v.z() + this->m[1][3],
                  this->m[2][0]*_v.x() + this->m[2][1]*_v.y() +
                  this->m[2][2]*_v.z() + this->m[2][3]);
}

//////////////////////////////////////////////////
bool Matrix4::operator==(const Matrix4 &_m) const
{
  return math::equal(this->m[0][0], _m[0][0]) &&
         math::equal(this->m[0][1], _m[0][1]) &&
         math::equal(this->m[0][2], _m[0][2]) &&
         math::equal(this->m[0][3], _m[0][3]) &&

         math::equal(this->m[1][0], _m[1][0]) &&
         math::equal(this->m[1][1], _m[1][1]) &&
         math::equal(this->m[1][2], _m[1][2]) &&
         math::equal(this->m[1][3], _m[1][3]) &&

         math::equal(this->m[2][0], _m[2][0]) &&
         math::equal(this->m[2][1], _m[2][1]) &&
         math::equal(this->m[2][2], _m[2][2]) &&
         math::equal(this->m[2][3], _m[2][3]) &&

         math::equal(this->m[3][0], _m[3][0]) &&
         math::equal(this->m[3][1], _m[3][1]) &&
         math::equal(this->m[3][2], _m[3][2]) &&
         math::equal(this->m[3][3], _m[3][3]);
}

//////////////////////////////////////////////////
Matrix4 Matrix4::Inverse() const
{
  double v0 = this->m[2][0] * this->m[3][1] - this->m[2][1] * this->m[3][0];
  double v1 = this->m[2][0] * this->m[3][2] - this->m[2][2] * this->m[3][0];
  double v2 = this->m[2][0] * this->m[3][3] - this->m[2][3] * this->m[3][0];
  double v3 = this->m[2][1] * this->m[3][2] - this->m[2][2] * this->m[3][1];
  double v4 = this->m[2][1] * this->m[3][3] - this->m[2][3] * this->m[3][1];
  double v5 = this->m[2][2] * this->m[3][3] - this->m[2][3] * this->m[3][2];

  double t00 = + (v5 * this->m[1][1] - v4 * this->m[1][2] + v3 * this->m[1][3]);
  double t10 = - (v5 * this->m[1][0] - v2 * this->m[1][2] + v1 * this->m[1][3]);
  double t20 = + (v4 * this->m[1][0] - v2 * this->m[1][1] + v0 * this->m[1][3]);
  double t30 = - (v3 * this->m[1][0] - v1 * this->m[1][1] + v0 * this->m[1][2]);

  double invDet = 1 / (t00 * this->m[0][0] + t10 * this->m[0][1] +
                       t20 * this->m[0][2] + t30 * this->m[0][3]);

  double d00 = t00 * invDet;
  double d10 = t10 * invDet;
  double d20 = t20 * invDet;
  double d30 = t30 * invDet;

  double d01 = - (v5 * this->m[0][1] - v4 * this->m[0][2] + v3 * this->m[0][3])
               * invDet;
  double d11 = + (v5 * this->m[0][0] - v2 * this->m[0][2] + v1 * this->m[0][3])
               * invDet;
  double d21 = - (v4 * this->m[0][0] - v2 * this->m[0][1] + v0 * this->m[0][3])
               * invDet;
  double d31 = + (v3 * this->m[0][0] - v1 * this->m[0][1] + v0 * this->m[0][2])
               * invDet;

  v0 = this->m[1][0] * this->m[3][1] - this->m[1][1] * this->m[3][0];
  v1 = this->m[1][0] * this->m[3][2] - this->m[1][2] * this->m[3][0];
  v2 = this->m[1][0] * this->m[3][3] - this->m[1][3] * this->m[3][0];
  v3 = this->m[1][1] * this->m[3][2] - this->m[1][2] * this->m[3][1];
  v4 = this->m[1][1] * this->m[3][3] - this->m[1][3] * this->m[3][1];
  v5 = this->m[1][2] * this->m[3][3] - this->m[1][3] * this->m[3][2];

  double d02 = + (v5 * this->m[0][1] - v4 * this->m[0][2] + v3 * this->m[0][3])
               * invDet;
  double d12 = - (v5 * this->m[0][0] - v2 * this->m[0][2] + v1 * this->m[0][3])
               * invDet;
  double d22 = + (v4 * this->m[0][0] - v2 * this->m[0][1] + v0 * this->m[0][3])
               * invDet;
  double d32 = - (v3 * this->m[0][0] - v1 * this->m[0][1] + v0 * this->m[0][2])
               * invDet;

  v0 = this->m[2][1] * this->m[1][0] - this->m[2][0] * this->m[1][1];
  v1 = this->m[2][2] * this->m[1][0] - this->m[2][0] * this->m[1][2];
  v2 = this->m[2][3] * this->m[1][0] - this->m[2][0] * this->m[1][3];
  v3 = this->m[2][2] * this->m[1][1] - this->m[2][1] * this->m[1][2];
  v4 = this->m[2][3] * this->m[1][1] - this->m[2][1] * this->m[1][3];
  v5 = this->m[2][3] * this->m[1][2] - this->m[2][2] * this->m[1][3];

  double d03 = - (v5 * this->m[0][1] - v4 * this->m[0][2] + v3 * this->m[0][3])
               * invDet;
  double d13 = + (v5 * this->m[0][0] - v2 * this->m[0][2] + v1 * this->m[0][3])
               * invDet;
  double d23 = - (v4 * this->m[0][0] - v2 * this->m[0][1] + v0 * this->m[0][3])
               * invDet;
  double d33 = + (v3 * this->m[0][0] - v1 * this->m[0][1] + v0 * this->m[0][2])
               * invDet;

  return Matrix4(d00, d01, d02, d03,
                 d10, d11, d12, d13,
                 d20, d21, d22, d23,
                 d30, d31, d32, d33);
}

//////////////////////////////////////////////////
math::Pose Matrix4::GetAsPose() const
{
  return math::Pose(this->GetTranslation(), this->GetRotation());
}

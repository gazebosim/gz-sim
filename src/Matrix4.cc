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
const IGN_MATRIX4 IGN_MATRIX4::Identity(
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1);


const IGN_MATRIX4 IGN_MATRIX4::Zero(
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0);

//////////////////////////////////////////////////
IGN_MATRIX4::IGN_MATRIX4()
{
  memset(this->data, 0, sizeof(this->data[0][0])*16);
}

//////////////////////////////////////////////////
IGN_MATRIX4::IGN_MATRIX4(const IGN_MATRIX4 &_m)
{
  memcpy(this->data, _m.data, sizeof(this->data[0][0])*16);
}

//////////////////////////////////////////////////
IGN_MATRIX4::IGN_MATRIX4(
  IGN_NUMERIC _v00, IGN_NUMERIC _v01, IGN_NUMERIC _v02, IGN_NUMERIC _v03,
  IGN_NUMERIC _v10, IGN_NUMERIC _v11, IGN_NUMERIC _v12, IGN_NUMERIC _v13,
  IGN_NUMERIC _v20, IGN_NUMERIC _v21, IGN_NUMERIC _v22, IGN_NUMERIC _v23,
  IGN_NUMERIC _v30, IGN_NUMERIC _v31, IGN_NUMERIC _v32, IGN_NUMERIC _v33)
{
  this->Set(_v00, _v01, _v02, _v03,
            _v10, _v11, _v12, _v13,
            _v20, _v21, _v22, _v23,
            _v30, _v31, _v32, _v33);
}

//////////////////////////////////////////////////
IGN_MATRIX4::~IGN_MATRIX4()
{
}

//////////////////////////////////////////////////
void IGN_MATRIX4::Set(
  IGN_NUMERIC _v00, IGN_NUMERIC _v01, IGN_NUMERIC _v02, IGN_NUMERIC _v03,
  IGN_NUMERIC _v10, IGN_NUMERIC _v11, IGN_NUMERIC _v12, IGN_NUMERIC _v13,
  IGN_NUMERIC _v20, IGN_NUMERIC _v21, IGN_NUMERIC _v22, IGN_NUMERIC _v23,
  IGN_NUMERIC _v30, IGN_NUMERIC _v31, IGN_NUMERIC _v32, IGN_NUMERIC _v33)
{
  this->data[0][0] = _v00;
  this->data[0][1] = _v01;
  this->data[0][2] = _v02;
  this->data[0][3] = _v03;

  this->data[1][0] = _v10;
  this->data[1][1] = _v11;
  this->data[1][2] = _v12;
  this->data[1][3] = _v13;

  this->data[2][0] = _v20;
  this->data[2][1] = _v21;
  this->data[2][2] = _v22;
  this->data[2][3] = _v23;

  this->data[3][0] = _v30;
  this->data[3][1] = _v31;
  this->data[3][2] = _v32;
  this->data[3][3] = _v33;
}

//////////////////////////////////////////////////
void IGN_MATRIX4::SetTranslate(const IGN_VECTOR3 &_t)
{
  this->data[0][3] = _t.x();
  this->data[1][3] = _t.y();
  this->data[2][3] = _t.z();
}

//////////////////////////////////////////////////
void IGN_MATRIX4::SetTranslate(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z)
{
  this->data[0][3] = _x;
  this->data[1][3] = _y;
  this->data[2][3] = _z;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_MATRIX4::GetTranslation() const
{
  return IGN_VECTOR3(this->data[0][3], this->data[1][3], this->data[2][3]);
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_MATRIX4::GetScale() const
{
  return IGN_VECTOR3(this->data[0][0], this->data[1][1], this->data[2][2]);
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_MATRIX4::GetRotation() const
{
  IGN_QUATERNION q;
  /// algorithm from Ogre::IGN_QUATERNION source, which in turn is based on
  /// Ken Shoemake's article "IGN_QUATERNION Calculus and Fast Animation".
  IGN_NUMERIC trace = this->data[0][0] + this->data[1][1] + this->data[2][2];
  IGN_NUMERIC root;
  if (trace > 0)
  {
    root = sqrt(trace + 1.0);
    q.w(root / 2.0);
    root = 1.0 / (2.0 * root);
    q.x((this->data[2][1] - this->data[1][2]) * root);
    q.y((this->data[0][2] - this->data[2][0]) * root);
    q.z((this->data[1][0] - this->data[0][1]) * root);
  }
  else
  {
    static unsigned int s_iNext[3] = {1, 2, 0};
    unsigned int i = 0;
    if (this->data[1][1] > this->data[0][0])
      i = 1;
    if (this->data[2][2] > this->data[i][i])
      i = 2;
    unsigned int j = s_iNext[i];
    unsigned int k = s_iNext[j];

    root = sqrt(this->data[i][i] - this->data[j][j] - this->data[k][k] + 1.0);

    IGN_NUMERIC a, b, c; 
    a = root / 2.0;
    root = 1.0 / (2.0 * root);
    b = (this->data[j][i] + this->data[i][j]) * root;
    c = (this->data[k][i] + this->data[i][k]) * root;

    switch (i)
    {
      case 0: q.x(a); break;
      case 1: q.y(a); break;
      case 2: q.z(a); break;
      default: break;
    };
    switch (j)
    {
      case 0: q.x(b); break;
      case 1: q.y(b); break;
      case 2: q.z(b); break;
      default: break;
    };
    switch (k)
    {
      case 0: q.x(c); break;
      case 1: q.y(c); break;
      case 2: q.z(c); break;
      default: break;
    };

    q.w((this->data[k][j] - this->data[j][k]) * root);
  }

  return q;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_MATRIX4::GetEulerRotation(bool _firstSolution) const
{
  IGN_VECTOR3 euler;
  IGN_VECTOR3 euler2;

  IGN_NUMERIC m31 = this->data[2][0];
  IGN_NUMERIC m11 = this->data[0][0];
  IGN_NUMERIC m12 = this->data[0][1];
  IGN_NUMERIC m13 = this->data[0][2];
  IGN_NUMERIC m32 = this->data[2][1];
  IGN_NUMERIC m33 = this->data[2][2];
  IGN_NUMERIC m21 = this->data[1][0];

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
void IGN_MATRIX4::SetScale(const IGN_VECTOR3 &_s)
{
  this->data[0][0] = _s.x();
  this->data[1][1] = _s.y();
  this->data[2][2] = _s.z();
  this->data[3][3] = 1.0;
}

//////////////////////////////////////////////////
void IGN_MATRIX4::SetScale(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z)
{
  this->data[0][0] = _x;
  this->data[1][1] = _y;
  this->data[2][2] = _z;
  this->data[3][3] = 1.0;
}

//////////////////////////////////////////////////
IGN_MATRIX4 &IGN_MATRIX4::operator =(const IGN_MATRIX4 &_mat)
{
  memcpy(this->data, _mat.data, sizeof(this->data[0][0])*16);
  return *this;
}

//////////////////////////////////////////////////
const IGN_MATRIX4 &IGN_MATRIX4::operator=(const IGN_MATRIX3 &_mat)
{
  this->data[0][0] = _mat(0, 0);
  this->data[0][1] = _mat(0, 1);
  this->data[0][2] = _mat(0, 2);

  this->data[1][0] = _mat(1, 0);
  this->data[1][1] = _mat(1, 1);
  this->data[1][2] = _mat(1, 2);

  this->data[2][0] = _mat(2, 0);
  this->data[2][1] = _mat(2, 1);
  this->data[2][2] = _mat(2, 2);

  return *this;
}


//////////////////////////////////////////////////
IGN_MATRIX4 IGN_MATRIX4::operator*(const IGN_MATRIX3 &_m2) const
{
  return IGN_MATRIX4(
      this->data[0][0] * _m2(0, 0) + this->data[0][1] * _m2(1, 0) +
      this->data[0][2] * _m2(2, 0),
      this->data[0][0] * _m2(0, 1) + this->data[0][1] * _m2(1, 1) +
      this->data[0][2] * _m2(2, 1),
      this->data[0][0] * _m2(0, 2) + this->data[0][1] * _m2(1, 2) +
      this->data[0][2] * _m2(2, 2),
      this->data[0][3],

      this->data[1][0] * _m2(0, 0) + this->data[1][1] * _m2(1, 0) +
      this->data[1][2] * _m2(2, 0),
      this->data[1][0] * _m2(0, 1) + this->data[1][1] * _m2(1, 1) +
      this->data[1][2] * _m2(2, 1),
      this->data[1][0] * _m2(0, 2) + this->data[1][1] * _m2(1, 2) +
      this->data[1][2] * _m2(2, 2),
      this->data[1][3],

      this->data[2][0] * _m2(0, 0) + this->data[2][1] * _m2(1, 0) +
      this->data[2][2] * _m2(2, 0),
      this->data[2][0] * _m2(0, 1) + this->data[2][1] * _m2(1, 1) +
      this->data[2][2] * _m2(2, 1),
      this->data[2][0] * _m2(0, 2) + this->data[2][1] * _m2(1, 2) +
      this->data[2][2] * _m2(2, 2),
      this->data[2][3],

      this->data[3][0],
      this->data[3][1],
      this->data[3][2],
      this->data[3][3]);
}

//////////////////////////////////////////////////
IGN_MATRIX4 IGN_MATRIX4::operator*(const IGN_MATRIX4 &_m2) const
{
  return IGN_MATRIX4(
    this->data[0][0] * _m2(0, 0) +
    this->data[0][1] * _m2(1, 0) +
    this->data[0][2] * _m2(2, 0) +
    this->data[0][3] * _m2(3, 0),
  
    this->data[0][0] * _m2(0, 1) +
    this->data[0][1] * _m2(1, 1) +
    this->data[0][2] * _m2(2, 1) +
    this->data[0][3] * _m2(3, 1),
  
    this->data[0][0] * _m2(0, 2) +
    this->data[0][1] * _m2(1, 2) +
    this->data[0][2] * _m2(2, 2) +
    this->data[0][3] * _m2(3, 2),
  
    this->data[0][0] * _m2(0, 3) +
    this->data[0][1] * _m2(1, 3) +
    this->data[0][2] * _m2(2, 3) +
    this->data[0][3] * _m2(3, 3),
  
    this->data[1][0] * _m2(0, 0) +
    this->data[1][1] * _m2(1, 0) +
    this->data[1][2] * _m2(2, 0) +
    this->data[1][3] * _m2(3, 0),
  
    this->data[1][0] * _m2(0, 1) +
    this->data[1][1] * _m2(1, 1) +
    this->data[1][2] * _m2(2, 1) +
    this->data[1][3] * _m2(3, 1),
  
    this->data[1][0] * _m2(0, 2) +
    this->data[1][1] * _m2(1, 2) +
    this->data[1][2] * _m2(2, 2) +
    this->data[1][3] * _m2(3, 2),
  
    this->data[1][0] * _m2(0, 3) +
    this->data[1][1] * _m2(1, 3) +
    this->data[1][2] * _m2(2, 3) +
    this->data[1][3] * _m2(3, 3),
  
    this->data[2][0] * _m2(0, 0) +
    this->data[2][1] * _m2(1, 0) +
    this->data[2][2] * _m2(2, 0) +
    this->data[2][3] * _m2(3, 0),
  
    this->data[2][0] * _m2(0, 1) +
    this->data[2][1] * _m2(1, 1) +
    this->data[2][2] * _m2(2, 1) +
    this->data[2][3] * _m2(3, 1),
  
    this->data[2][0] * _m2(0, 2) +
    this->data[2][1] * _m2(1, 2) +
    this->data[2][2] * _m2(2, 2) +
    this->data[2][3] * _m2(3, 2),
  
    this->data[2][0] * _m2(0, 3) +
    this->data[2][1] * _m2(1, 3) +
    this->data[2][2] * _m2(2, 3) +
    this->data[2][3] * _m2(3, 3),
  
    this->data[3][0] * _m2(0, 0) +
    this->data[3][1] * _m2(1, 0) +
    this->data[3][2] * _m2(2, 0) +
    this->data[3][3] * _m2(3, 0),
  
    this->data[3][0] * _m2(0, 1) +
    this->data[3][1] * _m2(1, 1) +
    this->data[3][2] * _m2(2, 1) +
    this->data[3][3] * _m2(3, 1),
  
    this->data[3][0] * _m2(0, 2) +
    this->data[3][1] * _m2(1, 2) +
    this->data[3][2] * _m2(2, 2) +
    this->data[3][3] * _m2(3, 2),
  
    this->data[3][0] * _m2(0, 3) +
    this->data[3][1] * _m2(1, 3) +
    this->data[3][2] * _m2(2, 3) +
    this->data[3][3] * _m2(3, 3));
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_MATRIX4::operator*(const IGN_VECTOR3 &_vec) const
{
  return IGN_VECTOR3(this->data[0][0]*_vec.x() + this->data[0][1]*_vec.y() +
                     this->data[0][2]*_vec.z() + this->data[0][3],
                     this->data[1][0]*_vec.x() + this->data[1][1]*_vec.y() +
                     this->data[1][2]*_vec.z() + this->data[1][3],
                     this->data[2][0]*_vec.x() + this->data[2][1]*_vec.y() +
                     this->data[2][2]*_vec.z() + this->data[2][3]);
}

//////////////////////////////////////////////////
bool IGN_MATRIX4::IsAffine() const
{
  return equal(this->data[3][0], static_cast<IGN_NUMERIC>(0)) &&
    equal(this->data[3][1], static_cast<IGN_NUMERIC>(0)) &&
    equal(this->data[3][2], static_cast<IGN_NUMERIC>(0)) &&
    equal(this->data[3][3], static_cast<IGN_NUMERIC>(1));
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_MATRIX4::TransformAffine(const IGN_VECTOR3 &_v) const
{
  if (!this->IsAffine())
    throw AffineException();

  return IGN_VECTOR3(this->data[0][0]*_v.x() + this->data[0][1]*_v.y() +
                     this->data[0][2]*_v.z() + this->data[0][3],
                     this->data[1][0]*_v.x() + this->data[1][1]*_v.y() +
                     this->data[1][2]*_v.z() + this->data[1][3],
                     this->data[2][0]*_v.x() + this->data[2][1]*_v.y() +
                     this->data[2][2]*_v.z() + this->data[2][3]);
}

//////////////////////////////////////////////////
bool IGN_MATRIX4::operator==(const IGN_MATRIX4 &_m) const
{
  return math::equal(this->data[0][0], _m(0, 0)) &&
         math::equal(this->data[0][1], _m(0, 1)) &&
         math::equal(this->data[0][2], _m(0, 2)) &&
         math::equal(this->data[0][3], _m(0, 3)) &&

         math::equal(this->data[1][0], _m(1, 0)) &&
         math::equal(this->data[1][1], _m(1, 1)) &&
         math::equal(this->data[1][2], _m(1, 2)) &&
         math::equal(this->data[1][3], _m(1, 3)) &&

         math::equal(this->data[2][0], _m(2, 0)) &&
         math::equal(this->data[2][1], _m(2, 1)) &&
         math::equal(this->data[2][2], _m(2, 2)) &&
         math::equal(this->data[2][3], _m(2, 3)) &&

         math::equal(this->data[3][0], _m(3, 0)) &&
         math::equal(this->data[3][1], _m(3, 1)) &&
         math::equal(this->data[3][2], _m(3, 2)) &&
         math::equal(this->data[3][3], _m(3, 3));
}

//////////////////////////////////////////////////
IGN_MATRIX4 IGN_MATRIX4::Inverse() const
{
  IGN_NUMERIC v0, v1, v2, v3, v4, v5, t00, t10, t20, t30;
  IGN_MATRIX4 r;

  v0 = this->data[2][0]*this->data[3][1] - this->data[2][1]*this->data[3][0];
  v1 = this->data[2][0]*this->data[3][2] - this->data[2][2]*this->data[3][0];
  v2 = this->data[2][0]*this->data[3][3] - this->data[2][3]*this->data[3][0];
  v3 = this->data[2][1]*this->data[3][2] - this->data[2][2]*this->data[3][1];
  v4 = this->data[2][1]*this->data[3][3] - this->data[2][3]*this->data[3][1];
  v5 = this->data[2][2]*this->data[3][3] - this->data[2][3]*this->data[3][2];

  t00 = +(v5*this->data[1][1] - v4*this->data[1][2] + v3*this->data[1][3]);
  t10 = -(v5*this->data[1][0] - v2*this->data[1][2] + v1*this->data[1][3]);
  t20 = +(v4*this->data[1][0] - v2*this->data[1][1] + v0*this->data[1][3]);
  t30 = -(v3*this->data[1][0] - v1*this->data[1][1] + v0*this->data[1][2]);

  IGN_NUMERIC invDet = 1 / (t00 * this->data[0][0] + t10 * this->data[0][1] +
      t20 * this->data[0][2] + t30 * this->data[0][3]);

  r(0, 0) = t00 * invDet;
  r(1, 0) = t10 * invDet;
  r(2, 0) = t20 * invDet;
  r(3, 0) = t30 * invDet;

  r(0, 1) = -(v5*this->data[0][1] - v4*this->data[0][2] + v3*this->data[0][3])
    * invDet;
  r(1, 1) = +(v5*this->data[0][0] - v2*this->data[0][2] + v1*this->data[0][3])
    * invDet;
  r(2, 1) = -(v4*this->data[0][0] - v2*this->data[0][1] + v0*this->data[0][3])
    * invDet;
  r(3, 1) = +(v3*this->data[0][0] - v1*this->data[0][1] + v0*this->data[0][2])
    * invDet;

  v0 = this->data[1][0]*this->data[3][1] - this->data[1][1]*this->data[3][0];
  v1 = this->data[1][0]*this->data[3][2] - this->data[1][2]*this->data[3][0];
  v2 = this->data[1][0]*this->data[3][3] - this->data[1][3]*this->data[3][0];
  v3 = this->data[1][1]*this->data[3][2] - this->data[1][2]*this->data[3][1];
  v4 = this->data[1][1]*this->data[3][3] - this->data[1][3]*this->data[3][1];
  v5 = this->data[1][2]*this->data[3][3] - this->data[1][3]*this->data[3][2];

  r(0, 2) = +(v5*this->data[0][1] - v4*this->data[0][2] + v3*this->data[0][3])
    * invDet;
  r(1, 2) = -(v5*this->data[0][0] - v2*this->data[0][2] + v1*this->data[0][3])
    * invDet;
  r(2, 2) = +(v4*this->data[0][0] - v2*this->data[0][1] + v0*this->data[0][3])
    * invDet;
  r(3, 2) = -(v3*this->data[0][0] - v1*this->data[0][1] + v0*this->data[0][2])
    * invDet;

  v0 = this->data[2][1]*this->data[1][0] - this->data[2][0]*this->data[1][1];
  v1 = this->data[2][2]*this->data[1][0] - this->data[2][0]*this->data[1][2];
  v2 = this->data[2][3]*this->data[1][0] - this->data[2][0]*this->data[1][3];
  v3 = this->data[2][2]*this->data[1][1] - this->data[2][1]*this->data[1][2];
  v4 = this->data[2][3]*this->data[1][1] - this->data[2][1]*this->data[1][3];
  v5 = this->data[2][3]*this->data[1][2] - this->data[2][2]*this->data[1][3];

  r(0, 3) = -(v5*this->data[0][1] - v4*this->data[0][2] + v3*this->data[0][3])
    * invDet;
  r(1, 3) = +(v5*this->data[0][0] - v2*this->data[0][2] + v1*this->data[0][3])
    * invDet;
  r(2, 3) = -(v4*this->data[0][0] - v2*this->data[0][1] + v0*this->data[0][3])
    * invDet;
  r(3, 3) = +(v3*this->data[0][0] - v1*this->data[0][1] + v0*this->data[0][2])
    * invDet;

  return r;
}

//////////////////////////////////////////////////
IGN_POSE3 IGN_MATRIX4::GetAsPose() const
{
  return IGN_POSE3(this->GetTranslation(), this->GetRotation());
}

//////////////////////////////////////////////////
std::ostream &ignition::math::operator<<(std::ostream &_out,
    const ignition::math::IGN_MATRIX4 &_m)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      _out << (fabs(_m(i, j)) < 1e-6 ? 0 : _m(i, j)) << " ";
    }
    _out << "\n";
  }

  return _out;
}


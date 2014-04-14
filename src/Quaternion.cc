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
IGN_QUATERNION::IGN_QUATERNION()
    : qw(1), qx(0), qy(0), qz(0)
{
  // quaternion not normalized, because that breaks
  // Pose::CoordPositionAdd(...)
}

//////////////////////////////////////////////////
IGN_QUATERNION::IGN_QUATERNION(const IGN_NUMERIC &_w, const IGN_NUMERIC &_x,
                       const IGN_NUMERIC &_y, const IGN_NUMERIC &_z)
    : qw(_w), qx(_x), qy(_y), qz(_z)
{
}

//////////////////////////////////////////////////
IGN_QUATERNION::IGN_QUATERNION(const IGN_NUMERIC &_roll,
    const IGN_NUMERIC &_pitch, const IGN_NUMERIC &_yaw)
{
  this->SetFromEuler(IGN_VECTOR3(_roll, _pitch, _yaw));
}

//////////////////////////////////////////////////
IGN_QUATERNION::IGN_QUATERNION(const IGN_VECTOR3 &_rpy)
{
  this->SetFromEuler(_rpy);
}

//////////////////////////////////////////////////
IGN_QUATERNION::IGN_QUATERNION(const IGN_VECTOR3 &_axis,
    const IGN_NUMERIC &_angle)
{
  this->SetFromAxis(_axis, _angle);
}

//////////////////////////////////////////////////
IGN_QUATERNION::IGN_QUATERNION(const IGN_QUATERNION &_qt)
{
  this->qw = _qt.qw;
  this->qx = _qt.qx;
  this->qy = _qt.qy;
  this->qz = _qt.qz;
}

//////////////////////////////////////////////////
IGN_QUATERNION::~IGN_QUATERNION()
{
}

//////////////////////////////////////////////////
IGN_QUATERNION &IGN_QUATERNION::operator =(const IGN_QUATERNION &qt)
{
  this->qw = qt.qw;
  this->qx = qt.qx;
  this->qy = qt.qy;
  this->qz = qt.qz;

  return *this;
}

//////////////////////////////////////////////////
void IGN_QUATERNION::SetToIdentity()
{
  this->qw = static_cast<IGN_NUMERIC>(1);
  this->qx = static_cast<IGN_NUMERIC>(0);
  this->qy = static_cast<IGN_NUMERIC>(0);
  this->qz = static_cast<IGN_NUMERIC>(0);
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::GetLog() const
{
  // If q = cos(A)+sin(A)*(x*i+y*j+z*k) where (x, y, z) is unit length, then
  // log(q) = A*(x*i+y*j+z*k).  If sin(A) is near zero, use log(q) =
  // sin(A)*(x*i+y*j+z*k) since sin(A)/A has limit 1.

  IGN_QUATERNION result;
  result.qw = 0.0;

  if (std::abs(this->qw) < 1.0)
  {
    IGN_NUMERIC fAngle = acos(this->qw);
    IGN_NUMERIC fSin = sin(fAngle);
    if (std::abs(fSin) >= 1e-3)
    {
      IGN_NUMERIC fCoeff = fAngle/fSin;
      result.qx = fCoeff*this->qx;
      result.qy = fCoeff*this->qy;
      result.qz = fCoeff*this->qz;
      return result;
    }
  }

  result.qx = this->qx;
  result.qy = this->qy;
  result.qz = this->qz;

  return result;
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::GetExp() const
{
  // If q = A*(x*i+y*j+z*k) where (x, y, z) is unit length, then
  // exp(q) = cos(A)+sin(A)*(x*i+y*j+z*k).  If sin(A) is near zero,
  // use exp(q) = cos(A)+A*(x*i+y*j+z*k) since A/sin(A) has limit 1.

  IGN_NUMERIC fAngle = sqrt(this->qx*this->qx+
      this->qy*this->qy+this->qz*this->qz);
  IGN_NUMERIC fSin = sin(fAngle);

  IGN_QUATERNION result;
  result.qw = cos(fAngle);

  if (std::abs(fSin) >= 1e-3)
  {
    IGN_NUMERIC fCoeff = fSin/fAngle;
    result.qx = fCoeff*this->qx;
    result.qy = fCoeff*this->qy;
    result.qz = fCoeff*this->qz;
  }
  else
  {
    result.qx = this->qx;
    result.qy = this->qy;
    result.qz = this->qz;
  }

  return result;
}

//////////////////////////////////////////////////
void IGN_QUATERNION::Invert()
{
  this->Normalize();
  // this->qw = this->qw;
  this->qx = -this->qx;
  this->qy = -this->qy;
  this->qz = -this->qz;
}

//////////////////////////////////////////////////
void IGN_QUATERNION::Normalize()
{
  IGN_NUMERIC s = 0;

  s = sqrt(this->qw * this->qw + this->qx * this->qx + this->qy * this->qy +
           this->qz * this->qz);

  if (math::equal(s, static_cast<IGN_NUMERIC>(0)))
  {
    this->qw = 1.0;
    this->qx = 0.0;
    this->qy = 0.0;
    this->qz = 0.0;
  }
  else
  {
    this->qw /= s;
    this->qx /= s;
    this->qy /= s;
    this->qz /= s;
  }
}

//////////////////////////////////////////////////
void IGN_QUATERNION::SetFromAxis(IGN_NUMERIC _ax, IGN_NUMERIC _ay,
    IGN_NUMERIC _az, IGN_NUMERIC _aa)
{
  IGN_NUMERIC l;

  l = _ax * _ax + _ay * _ay + _az * _az;

  if (math::equal(l, static_cast<IGN_NUMERIC>(0)))
  {
    this->qw = 1;
    this->qx = 0;
    this->qy = 0;
    this->qz = 0;
  }
  else
  {
    _aa *= 0.5;
    l = sin(_aa) / sqrt(l);
    this->qw = cos(_aa);
    this->qx = _ax * l;
    this->qy = _ay * l;
    this->qz = _az * l;
  }

  this->Normalize();
}

//////////////////////////////////////////////////
void IGN_QUATERNION::SetFromAxis(const IGN_VECTOR3 &_axis, IGN_NUMERIC _a)
{
  this->SetFromAxis(_axis.x(), _axis.y(), _axis.z(), _a);
}

//////////////////////////////////////////////////
void IGN_QUATERNION::Set(IGN_NUMERIC _w, IGN_NUMERIC _x, IGN_NUMERIC _y,
    IGN_NUMERIC _z)
{
  this->qw = _w;
  this->qx = _x;
  this->qy = _y;
  this->qz = _z;
}

//////////////////////////////////////////////////
void IGN_QUATERNION::SetFromEuler(const IGN_VECTOR3 &_vec)
{
  this->SetFromEuler(_vec.x(), _vec.y(), _vec.z());
}

//////////////////////////////////////////////////
void IGN_QUATERNION::SetFromEuler(IGN_NUMERIC _roll, IGN_NUMERIC _pitch,
    IGN_NUMERIC _yaw)
{
  IGN_NUMERIC phi, the, psi;

  phi = _roll / 2.0;
  the = _pitch / 2.0;
  psi = _yaw / 2.0;

  this->qw = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
  this->qx = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
  this->qy = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
  this->qz = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);

  this->Normalize();
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_QUATERNION::GetAsEuler() const
{
  IGN_VECTOR3 vec;

  IGN_QUATERNION copy = *this;
  IGN_NUMERIC squ;
  IGN_NUMERIC sqx;
  IGN_NUMERIC sqy;
  IGN_NUMERIC sqz;

  copy.Normalize();

  squ = copy.qw * copy.qw;
  sqx = copy.qx * copy.qx;
  sqy = copy.qy * copy.qy;
  sqz = copy.qz * copy.qz;

  // Roll
  vec.x(atan2(2 * (copy.qy*copy.qz + copy.qw*copy.qx), squ - sqx - sqy + sqz));

  // Pitch
  IGN_NUMERIC sarg = -2 * (copy.qx*copy.qz - copy.qw * copy.qy);
  vec.y(sarg <= -1.0 ? -0.5*M_PI : (sarg >= 1.0 ? 0.5*M_PI : asin(sarg)));

  // Yaw
  vec.z(atan2(2 * (copy.qx*copy.qy + copy.qw*copy.qz), squ + sqx - sqy - sqz));

  return vec;
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::EulerToQuaternion(const IGN_VECTOR3 &_vec)
{
  IGN_QUATERNION result;
  result.SetFromEuler(_vec);
  return result;
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::EulerToQuaternion(IGN_NUMERIC _x,
    IGN_NUMERIC _y, IGN_NUMERIC _z)
{
  return EulerToQuaternion(IGN_VECTOR3(_x, _y, _z));
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_QUATERNION::GetRoll()
{
  return this->GetAsEuler().x();
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_QUATERNION::GetPitch()
{
  return this->GetAsEuler().y();
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_QUATERNION::GetYaw()
{
  return this->GetAsEuler().z();
}

//////////////////////////////////////////////////
void IGN_QUATERNION::GetAsAxis(IGN_VECTOR3 &_axis, IGN_NUMERIC &_angle) const
{
  IGN_NUMERIC len = this->qx*this->qx + this->qy*this->qy + this->qz*this->qz;
  if (math::equal(len, static_cast<IGN_NUMERIC>(0)))
  {
    _angle = 0.0;
    _axis.Set(1, 0, 0);
  }
  else
  {
    _angle = 2.0 * acos(this->qw);
    IGN_NUMERIC invLen =  1.0 / sqrt(len);
    _axis.Set(this->qx*invLen, this->qy*invLen, this->qz*invLen);
  }
}

//////////////////////////////////////////////////
void IGN_QUATERNION::Scale(IGN_NUMERIC _scale)
{
  IGN_QUATERNION b;
  IGN_VECTOR3 axis;
  IGN_NUMERIC angle;

  // Convert to axis-and-angle
  this->GetAsAxis(axis, angle);
  angle *= _scale;

  this->SetFromAxis(axis.x(), axis.y(), axis.z(), angle);
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::operator+(const IGN_QUATERNION &qt) const
{
  IGN_QUATERNION result(this->qw + qt.qw, this->qx + qt.qx,
                        this->qy + qt.qy, this->qz + qt.qz);
  return result;
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::operator+=(const IGN_QUATERNION &qt)
{
  *this = *this + qt;

  return *this;
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::operator-=(const IGN_QUATERNION &qt)
{
  *this = *this - qt;
  return *this;
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::operator-(const IGN_QUATERNION &qt) const
{
  IGN_QUATERNION result(this->qw - qt.qw, this->qx - qt.qx,
                 this->qy - qt.qy, this->qz - qt.qz);
  return result;
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::operator*=(const IGN_QUATERNION &qt)
{
  *this = *this * qt;
  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_QUATERNION::operator*(const IGN_VECTOR3 &v) const
{
  IGN_VECTOR3 uv, uuv;
  IGN_VECTOR3 qvec(this->qx, this->qy, this->qz);
  uv = qvec.Cross(v);
  uuv = qvec.Cross(uv);
  uv *= (2.0f * this->qw);
  uuv *= 2.0f;

  return v + uv + uuv;
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::operator*(const IGN_NUMERIC &_f) const
{
  return IGN_QUATERNION(this->qw*_f, this->qx*_f, this->qy*_f, this->qz*_f);
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_QUATERNION::RotateVectorReverse(IGN_VECTOR3 _vec) const
{
  IGN_QUATERNION tmp(0.0, _vec.x(), _vec.y(), _vec.z());

  tmp =  this->GetInverse() * (tmp * (*this));

  return IGN_VECTOR3(tmp.qx, tmp.qy, tmp.qz);
}


//////////////////////////////////////////////////
bool IGN_QUATERNION::IsFinite() const
{
  return std::isfinite(this->qw) && std::isfinite(this->qx) && std::isfinite(this->qy) &&
         std::isfinite(this->qz);
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_QUATERNION::GetXAxis() const
{
  IGN_NUMERIC fTy  = 2.0f*this->qy;
  IGN_NUMERIC fTz  = 2.0f*this->qz;

  IGN_NUMERIC fTwy = fTy*this->qw;
  IGN_NUMERIC fTwz = fTz*this->qw;
  IGN_NUMERIC fTxy = fTy*this->qx;
  IGN_NUMERIC fTxz = fTz*this->qx;
  IGN_NUMERIC fTyy = fTy*this->qy;
  IGN_NUMERIC fTzz = fTz*this->qz;

  return IGN_VECTOR3(1.0f-(fTyy+fTzz), fTxy+fTwz, fTxz-fTwy);
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_QUATERNION::GetYAxis() const
{
  IGN_NUMERIC fTx  = 2.0f*this->qx;
  IGN_NUMERIC fTy  = 2.0f*this->qy;
  IGN_NUMERIC fTz  = 2.0f*this->qz;
  IGN_NUMERIC fTwx = fTx*this->qw;
  IGN_NUMERIC fTwz = fTz*this->qw;
  IGN_NUMERIC fTxx = fTx*this->qx;
  IGN_NUMERIC fTxy = fTy*this->qx;
  IGN_NUMERIC fTyz = fTz*this->qy;
  IGN_NUMERIC fTzz = fTz*this->qz;

  return IGN_VECTOR3(fTxy-fTwz, 1.0f-(fTxx+fTzz), fTyz+fTwx);
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_QUATERNION::GetZAxis() const
{
  IGN_NUMERIC fTx  = 2.0f*this->qx;
  IGN_NUMERIC fTy  = 2.0f*this->qy;
  IGN_NUMERIC fTz  = 2.0f*this->qz;
  IGN_NUMERIC fTwx = fTx*this->qw;
  IGN_NUMERIC fTwy = fTy*this->qw;
  IGN_NUMERIC fTxx = fTx*this->qx;
  IGN_NUMERIC fTxz = fTz*this->qx;
  IGN_NUMERIC fTyy = fTy*this->qy;
  IGN_NUMERIC fTyz = fTz*this->qy;

  return IGN_VECTOR3(fTxz+fTwy, fTyz-fTwx, 1.0f-(fTxx+fTyy));
}

//////////////////////////////////////////////////
bool IGN_QUATERNION::operator ==(const IGN_QUATERNION &_qt) const
{
  return equal(this->qx, _qt.qx, static_cast<IGN_NUMERIC>(0.001)) &&
         equal(this->qy, _qt.qy, static_cast<IGN_NUMERIC>(0.001)) &&
         equal(this->qz, _qt.qz, static_cast<IGN_NUMERIC>(0.001)) &&
         equal(this->qw, _qt.qw, static_cast<IGN_NUMERIC>(0.001));
}

//////////////////////////////////////////////////
bool IGN_QUATERNION::operator!=(const IGN_QUATERNION &_qt) const
{
  return !equal(this->qx, _qt.qx, static_cast<IGN_NUMERIC>(0.001)) ||
         !equal(this->qy, _qt.qy, static_cast<IGN_NUMERIC>(0.001)) ||
         !equal(this->qz, _qt.qz, static_cast<IGN_NUMERIC>(0.001)) ||
         !equal(this->qw, _qt.qw, static_cast<IGN_NUMERIC>(0.001));
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::operator-() const
{
  return IGN_QUATERNION(-this->qw, -this->qx, -this->qy, -this->qz);
}

//////////////////////////////////////////////////
IGN_MATRIX3 IGN_QUATERNION::GetAsMatrix3() const
{
  IGN_QUATERNION qt = *this;
  qt.Normalize();
  return IGN_MATRIX3(1 - 2*qt.qy*qt.qy - 2 *qt.qz*qt.qz,
                 2 * qt.qx*qt.qy - 2*qt.qz*qt.qw,
                 2 * qt.qx * qt.qz + 2 * qt.qy * qt.qw,
                 2 * qt.qx * qt.qy + 2 * qt.qz * qt.qw,
                 1 - 2*qt.qx*qt.qx - 2 * qt.qz*qt.qz,
                 2 * qt.qy * qt.qz - 2 * qt.qx * qt.qw,
                 2 * qt.qx * qt.qz - 2 * qt.qy * qt.qw,
                 2 * qt.qy * qt.qz + 2 * qt.qx * qt.qw,
                 1 - 2 * qt.qx*qt.qx - 2 * qt.qy*qt.qy);
}

//////////////////////////////////////////////////
IGN_MATRIX4 IGN_QUATERNION::GetAsMatrix4() const
{
  IGN_MATRIX4 result(IGN_MATRIX4::Identity);
  result = this->GetAsMatrix3();
  return result;
}

//////////////////////////////////////////////////
void IGN_QUATERNION::Round(int _precision)
{
  this->qx = precision(this->qx, _precision);
  this->qy = precision(this->qy, _precision);
  this->qz = precision(this->qz, _precision);
  this->qw = precision(this->qw, _precision);
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_QUATERNION::Dot(const IGN_QUATERNION &_q) const
{
  return this->qw*_q.qw + this->qx * _q.qx + this->qy*_q.qy + this->qz*_q.qz;
}


//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::Squad(IGN_NUMERIC _fT,
    const IGN_QUATERNION &_rkP, const IGN_QUATERNION &_rkA,
    const IGN_QUATERNION &_rkB, const IGN_QUATERNION &_rkQ, bool _shortestPath)
{
  IGN_NUMERIC fSlerpT = 2.0f*_fT*(1.0f-_fT);
  IGN_QUATERNION kSlerpP = Slerp(_fT, _rkP, _rkQ, _shortestPath);
  IGN_QUATERNION kSlerpQ = Slerp(_fT, _rkA, _rkB);
  return Slerp(fSlerpT, kSlerpP, kSlerpQ);
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_QUATERNION::Slerp(IGN_NUMERIC _fT,
    const IGN_QUATERNION &_rkP, const IGN_QUATERNION &_rkQ, bool _shortestPath)
{
  IGN_NUMERIC fCos = _rkP.Dot(_rkQ);
  IGN_QUATERNION rkT;

  // Do we need to invert rotation?
  if (fCos < 0.0f && _shortestPath)
  {
    fCos = -fCos;
    rkT = -_rkQ;
  }
  else
  {
    rkT = _rkQ;
  }

  if (std::abs(fCos) < 1 - 1e-03)
  {
    // Standard case (slerp)
    IGN_NUMERIC fSin = sqrt(1 - (fCos*fCos));
    IGN_NUMERIC fAngle = atan2(fSin, fCos);
    // FIXME: should check if (std::abs(fSin) >= 1e-3)
    IGN_NUMERIC fInvSin = 1.0f / fSin;
    IGN_NUMERIC fCoeff0 = sin((1.0f - _fT) * fAngle) * fInvSin;
    IGN_NUMERIC fCoeff1 = sin(_fT * fAngle) * fInvSin;
    return _rkP * fCoeff0 + rkT * fCoeff1;
  }
  else
  {
    // There are two situations:
    // 1. "rkP" and "rkQ" are very close (fCos ~= +1), so we can do a linear
    //    interpolation safely.
    // 2. "rkP" and "rkQ" are almost inverse of each other (fCos ~= -1), there
    //    are an infinite number of possibilities interpolation. but we haven't
    //    have method to fix this case, so just use linear interpolation here.
    IGN_QUATERNION t = _rkP * (1.0f - _fT) + rkT * _fT;
    // taking the complement requires renormalisation
    t.Normalize();
    return t;
  }
}

/////////////////////////////////////////////////
std::ostream &ignition::math::operator<<(std::ostream &_out,
    const ignition::math::IGN_QUATERNION &_q)
{
  IGN_VECTOR3 v(_q.GetAsEuler());
  _out << precision(v.x(), 6) << " " << precision(v.y(), 6) << " "
       << precision(v.z(), 6);
  return _out;
}


/////////////////////////////////////////////////
std::istream &ignition::math::operator>>(std::istream &_in,
    ignition::math::IGN_QUATERNION &_q)
{
  Angle roll, pitch, yaw;

  // Skip white spaces
  _in.setf(std::ios_base::skipws);
  _in >> roll >> pitch >> yaw;

  _q.SetFromEuler(IGN_VECTOR3(*roll, *pitch, *yaw));

  return _in;
}

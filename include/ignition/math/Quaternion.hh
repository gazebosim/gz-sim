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

#ifndef _IGNITION_QUATERNION_HH_
#define _IGNITION_QUATERNION_HH_

#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
  namespace math
  {
    /// \class Quaternion Quaternion.hh ignition/math.hh
    /// \brief A quaternion class
    template<typename T>
    class IGNITION_VISIBLE Quaternion
    {
      /// \brief Default Constructor
      public: Quaternion()
      : qw(1), qx(0), qy(0), qz(0)
      {
        // quaternion not normalized, because that breaks
        // Pose::CoordPositionAdd(...)
      }

      /// \brief Constructor
      /// \param[in] _w W param
      /// \param[in] _x X param
      /// \param[in] _y Y param
      /// \param[in] _z Z param
      public: Quaternion(const T &_w, const T &_x, const T &_y, const T &_z)
      : qw(_w), qx(_x), qy(_y), qz(_z)
      {}

      /// \brief Constructor from Euler angles in radians
      /// \param[in] _roll  roll
      /// \param[in] _pitch pitch
      /// \param[in] _yaw   yaw
      public: Quaternion(const T &_roll, const T &_pitch, const T &_yaw)
      {
        this->SetFromEuler(Vector3<T>(_roll, _pitch, _yaw));
      }

      /// \brief Constructor from axis angle
      /// \param[in] _axis the rotation axis
      /// \param[in] _angle the rotation angle in radians
      public: Quaternion(const Vector3<T> &_axis, const T &_angle)
      {
        this->SetFromAxis(_axis, _angle);
      }

      /// \brief Constructor
      /// \param[in] _rpy euler angles
      public: Quaternion(const Vector3<T> &_rpy)
      {
        this->SetFromEuler(_rpy);
      }

      /// \brief Copy constructor
      /// \param qt Quaternion<T> to copy
      public: Quaternion(const Quaternion<T> &_qt)
      {
        this->qw = _qt.qw;
        this->qx = _qt.qx;
        this->qy = _qt.qy;
        this->qz = _qt.qz;
      }

      /// \brief Destructor
      public: ~Quaternion() {}

      /// \brief Equal operator
      /// \param[in] _qt Quaternion<T> to copy
      public: Quaternion<T> &operator=(const Quaternion<T> &_qt)
      {
        this->qw = _qt.qw;
        this->qx = _qt.qx;
        this->qy = _qt.qy;
        this->qz = _qt.qz;

        return *this;
      }

      /// \brief Invert the quaternion
      public: void Invert()
      {
        this->Normalize();
        // this->qw = this->qw;
        this->qx = -this->qx;
        this->qy = -this->qy;
        this->qz = -this->qz;
      }

      /// \brief Get the inverse of this quaternion
      /// \return Inverse quarenion
      public: inline Quaternion<T> GetInverse() const
              {
                T s = 0;
                Quaternion<T> q(this->qw, this->qx, this->qy, this->qz);

                // use s to test if quaternion is valid
                s = q.qw * q.qw + q.qx * q.qx + q.qy * q.qy + q.qz * q.qz;

                if (equal<T>(s, static_cast<T>(0)))
                {
                  q.qw = 1.0;
                  q.qx = 0.0;
                  q.qy = 0.0;
                  q.qz = 0.0;
                }
                else
                {
                  // deal with non-normalized quaternion
                  // div by s so q * qinv = identity
                  q.qw =  q.qw / s;
                  q.qx = -q.qx / s;
                  q.qy = -q.qy / s;
                  q.qz = -q.qz / s;
                }
                return q;
              }

      /// \brief Set the quatern to the identity
      public: void SetToIdentity()
      {
        this->qw = static_cast<T>(1);
        this->qx = static_cast<T>(0);
        this->qy = static_cast<T>(0);
        this->qz = static_cast<T>(0);
      }

      /// \brief Return the logarithm
      /// \return the log
      public: Quaternion<T> GetLog() const
      {
        // If q = cos(A)+sin(A)*(x*i+y*j+z*k) where (x, y, z) is unit length,
        // then log(q) = A*(x*i+y*j+z*k).  If sin(A) is near zero, use log(q) =
        // sin(A)*(x*i+y*j+z*k) since sin(A)/A has limit 1.

        Quaternion<T> result;
        result.qw = 0.0;

        if (std::abs(this->qw) < 1.0)
        {
          T fAngle = acos(this->qw);
          T fSin = sin(fAngle);
          if (std::abs(fSin) >= 1e-3)
          {
            T fCoeff = fAngle/fSin;
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

      /// \brief Return the exponent
      /// \return the exp
      public: Quaternion<T> GetExp() const
      {
        // If q = A*(x*i+y*j+z*k) where (x, y, z) is unit length, then
        // exp(q) = cos(A)+sin(A)*(x*i+y*j+z*k).  If sin(A) is near zero,
        // use exp(q) = cos(A)+A*(x*i+y*j+z*k) since A/sin(A) has limit 1.

        T fAngle = sqrt(this->qx*this->qx+
            this->qy*this->qy+this->qz*this->qz);
        T fSin = sin(fAngle);

        Quaternion<T> result;
        result.qw = cos(fAngle);

        if (std::abs(fSin) >= 1e-3)
        {
          T fCoeff = fSin/fAngle;
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

      /// \brief Normalize the quaternion
      public: void Normalize()
      {
        T s = 0;

        s = sqrt(this->qw * this->qw + this->qx * this->qx +
            this->qy * this->qy + this->qz * this->qz);

        if (equal<T>(s, static_cast<T>(0)))
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

      /// \brief Set the quaternion from an axis and angle
      /// \param[in] _x X axis
      /// \param[in] _y Y axis
      /// \param[in] _z Z axis
      /// \param[in] _a Angle in radians
      public: void SetFromAxis(T _ax, T _ay, T _az, T _aa)
      {
        T l;

        l = _ax * _ax + _ay * _ay + _az * _az;

        if (equal<T>(l, static_cast<T>(0)))
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

      /// \brief Set the quaternion from an axis and angle
      /// \param[in] _axis Axis
      /// \param[in] _a Angle in radians
      public: void SetFromAxis(const Vector3<T> &_axis, T _a)
      {
        this->SetFromAxis(_axis.x(), _axis.y(), _axis.z(), _a);
      }

      /// \brief Set this quaternion from 4 floating numbers
      /// \param[in] _u u
      /// \param[in] _x x
      /// \param[in] _y y
      /// \param[in] _z z
      public: void Set(T _w, T _x, T _y, T _z)
      {
        this->qw = _w;
        this->qx = _x;
        this->qy = _y;
        this->qz = _z;
      }

      /// \brief Set the quaternion from Euler angles. The order of operations
      /// are roll, pitch, yaw.
      /// \param[in] vec  Euler angle
      public: void SetFromEuler(const Vector3<T> &_vec)
      {
        this->SetFromEuler(_vec.x(), _vec.y(), _vec.z());
      }

      /// \brief Set the quaternion from Euler angles.
      /// \param[in] _roll Roll angle (radians).
      /// \param[in] _pitch Roll angle (radians).
      /// \param[in] _yaw Roll angle (radians).
      public: void SetFromEuler(T _roll, T _pitch, T _yaw)
      {
        T phi, the, psi;

        phi = _roll / 2.0;
        the = _pitch / 2.0;
        psi = _yaw / 2.0;

        this->qw = cos(phi) * cos(the) * cos(psi) +
          sin(phi) * sin(the) * sin(psi);
        this->qx = sin(phi) * cos(the) * cos(psi) -
          cos(phi) * sin(the) * sin(psi);
        this->qy = cos(phi) * sin(the) * cos(psi) +
          sin(phi) * cos(the) * sin(psi);
        this->qz = cos(phi) * cos(the) * sin(psi) -
          sin(phi) * sin(the) * cos(psi);

        this->Normalize();
      }

      /// \brief Return the rotation in Euler angles
      /// \return This quaternion as an Euler vector
      public: Vector3<T> GetAsEuler() const
      {
        Vector3<T> vec;

        Quaternion<T> copy = *this;
        T squ;
        T sqx;
        T sqy;
        T sqz;

        copy.Normalize();

        squ = copy.qw * copy.qw;
        sqx = copy.qx * copy.qx;
        sqy = copy.qy * copy.qy;
        sqz = copy.qz * copy.qz;

        // Roll
        vec.x(atan2(2 * (copy.qy*copy.qz + copy.qw*copy.qx),
              squ - sqx - sqy + sqz));

        // Pitch
        T sarg = -2 * (copy.qx*copy.qz - copy.qw * copy.qy);
        vec.y(sarg <= -1.0 ? -0.5*M_PI :
            (sarg >= 1.0 ? 0.5*M_PI : asin(sarg)));

        // Yaw
        vec.z(atan2(2 * (copy.qx*copy.qy + copy.qw*copy.qz),
              squ + sqx - sqy - sqz));

        return vec;
      }

      /// \brief Convert euler angles to quatern.
      /// \param[in]
      public: static Quaternion<T> EulerToQuaternion(const Vector3<T> &_vec)
      {
        Quaternion<T> result;
        result.SetFromEuler(_vec);
        return result;
      }

      /// \brief Convert euler angles to quatern.
      /// \param[in] _x rotation along x
      /// \param[in] _y rotation along y
      /// \param[in] _z rotation along z
      public: static Quaternion<T> EulerToQuaternion(T _x, T _y, T _z)
      {
        return EulerToQuaternion(Vector3<T>(_x, _y, _z));
      }

      /// \brief Get the Euler roll angle in radians
      /// \return the roll
      public: T GetRoll()
      {
        return this->GetAsEuler().x();
      }

      /// \brief Get the Euler pitch angle in radians
      /// \return the pitch
      public: T GetPitch()
      {
        return this->GetAsEuler().y();
      }

      /// \brief Get the Euler yaw angle in radians
      /// \return the yaw
      public: T GetYaw()
      {
        return this->GetAsEuler().z();
      }

      /// \brief Return rotation as axis and angle
      /// \param[in] _axis rotation axis
      /// \param[in] _angle ccw angle in radians
      public: void GetAsAxis(Vector3<T> &_axis, T &_angle) const
      {
        T len = this->qx*this->qx + this->qy*this->qy + this->qz*this->qz;
        if (equal<T>(len, static_cast<T>(0)))
        {
          _angle = 0.0;
          _axis.Set(1, 0, 0);
        }
        else
        {
          _angle = 2.0 * acos(this->qw);
          T invLen =  1.0 / sqrt(len);
          _axis.Set(this->qx*invLen, this->qy*invLen, this->qz*invLen);
        }
      }

      /// \brief Scale a Quaternion<T>ion
      /// \param[in] _scale Amount to scale this rotation
      public: void Scale(T _scale)
      {
        Quaternion<T> b;
        Vector3<T> axis;
        T angle;

        // Convert to axis-and-angle
        this->GetAsAxis(axis, angle);
        angle *= _scale;

        this->SetFromAxis(axis.x(), axis.y(), axis.z(), angle);
      }

      /// \brief Addition operator
      /// \param[in] _qt quaternion for addition
      /// \return this quaternion + _qt
      public: Quaternion<T> operator+(const Quaternion<T> &_qt) const
      {
        Quaternion<T> result(this->qw + _qt.qw, this->qx + _qt.qx,
                             this->qy + _qt.qy, this->qz + _qt.qz);
        return result;
      }

      /// \brief Addition operator
      /// \param[in] _qt quaternion for addition
      /// \return this quaternion + qt
      public: Quaternion<T> operator+=(const Quaternion<T> &_qt)
      {
        *this = *this + _qt;

        return *this;
      }

      /// \brief Substraction operator
      /// \param[in] _qt quaternion to substract
      /// \return this quaternion - _qt
      public: Quaternion<T> operator-(const Quaternion<T> &_qt) const
      {
        Quaternion<T> result(this->qw - _qt.qw, this->qx - _qt.qx,
                       this->qy - _qt.qy, this->qz - _qt.qz);
        return result;
      }

      /// \brief Substraction operator
      /// \param[in] _qt Quaternion<T> for substraction
      /// \return This quatern - qt
      public: Quaternion<T> operator-=(const Quaternion<T> &_qt)
      {
        *this = *this - _qt;
        return *this;
      }

      /// \brief Multiplication operator
      /// \param[in] _qt Quaternion<T> for multiplication
      /// \return This quaternion multiplied by the parameter
      public: inline Quaternion<T> operator*(const Quaternion<T> &_q) const
              {
                return Quaternion<T>(
                  this->qw*_q.qw-this->qx*_q.qx-this->qy*_q.qy-this->qz*_q.qz,
                  this->qw*_q.qx+this->qx*_q.qw+this->qy*_q.qz-this->qz*_q.qy,
                  this->qw*_q.qy-this->qx*_q.qz+this->qy*_q.qw+this->qz*_q.qx,
                  this->qw*_q.qz+this->qx*_q.qy-this->qy*_q.qx+this->qz*_q.qw);
              }

      /// \brief Multiplication operator
      /// \param[in] _f factor
      /// \return quaternion multiplied by _f
      public: Quaternion<T> operator*(const T &_f) const
      {
        return Quaternion<T>(this->qw*_f, this->qx*_f,
                             this->qy*_f, this->qz*_f);
      }

      /// \brief Multiplication operator
      /// \param[in] _qt Quaternion<T> for multiplication
      /// \return This quatern multiplied by the parameter
      public: Quaternion<T> operator*=(const Quaternion<T> &qt)
      {
        *this = *this * qt;
        return *this;
      }

      /// \brief Vector3 multiplication operator
      /// \param[in] _v vector to multiply
      public: Vector3<T> operator*(const Vector3<T> &_v) const
      {
        Vector3<T> uv, uuv;
        Vector3<T> qvec(this->qx, this->qy, this->qz);
        uv = qvec.Cross(_v);
        uuv = qvec.Cross(uv);
        uv *= (2.0f * this->qw);
        uuv *= 2.0f;

        return _v + uv + uuv;
      }

      /// \brief Equal to operator
      /// \param[in] _qt Quaternion<T> for comparison
      /// \return True if equal
      public: bool operator==(const Quaternion<T> &_qt) const
      {
        return equal(this->qx, _qt.qx, static_cast<T>(0.001)) &&
               equal(this->qy, _qt.qy, static_cast<T>(0.001)) &&
               equal(this->qz, _qt.qz, static_cast<T>(0.001)) &&
               equal(this->qw, _qt.qw, static_cast<T>(0.001));
      }

      /// \brief Not equal to operator
      /// \param[in] _qt Quaternion<T> for comparison
      /// \return True if not equal
      public: bool operator!=(const Quaternion<T> &_qt) const
      {
        return !equal(this->qx, _qt.qx, static_cast<T>(0.001)) ||
               !equal(this->qy, _qt.qy, static_cast<T>(0.001)) ||
               !equal(this->qz, _qt.qz, static_cast<T>(0.001)) ||
               !equal(this->qw, _qt.qw, static_cast<T>(0.001));
      }

      /// \brief Unary minus operator
      /// \return negates each component of the quaternion
      public: Quaternion<T> operator-() const
      {
        return Quaternion<T>(-this->qw, -this->qx, -this->qy, -this->qz);
      }

      /// \brief Rotate a vector using the quaternion
      /// \param[in] _vec vector to rotate
      /// \return the rotated vector
      public: inline Vector3<T> RotateVector(const Vector3<T> &_vec) const
      {
        Quaternion<T> tmp(static_cast<T>(0),
            _vec.x(), _vec.y(), _vec.z());
        tmp = (*this) * (tmp * this->GetInverse());
        return Vector3<T>(tmp.qx, tmp.qy, tmp.qz);
      }

      /// \brief Do the reverse rotation of a vector by this quaternion
      /// \param[in] _vec the vector
      /// \return the
      public: Vector3<T> RotateVectorReverse(Vector3<T> _vec) const
      {
        Quaternion<T> tmp(0.0, _vec.x(), _vec.y(), _vec.z());

        tmp =  this->GetInverse() * (tmp * (*this));

        return Vector3<T>(tmp.qx, tmp.qy, tmp.qz);
      }

      /// \brief See if a quatern is finite (e.g., not nan)
      /// \return True if quatern is finite
      public: bool IsFinite() const
      {
        // std::isfinite works with floating point values, need to explicit
        // cast to avoid ambiguity in vc++.
        return std::isfinite(static_cast<double>(this->qw)) &&
               std::isfinite(static_cast<double>(this->qx)) &&
               std::isfinite(static_cast<double>(this->qy)) &&
               std::isfinite(static_cast<double>(this->qz));
      }

      /// \brief Correct any nan
      public: inline void Correct()
      {
        // std::isfinite works with floating point values, need to explicit
        // cast to avoid ambiguity in vc++.
        if (!std::isfinite(static_cast<double>(this->qx)))
          this->qx = 0;
        if (!std::isfinite(static_cast<double>(this->qy)))
          this->qy = 0;
        if (!std::isfinite(static_cast<double>(this->qz)))
          this->qz = 0;
        if (!std::isfinite(static_cast<double>(this->qw)))
          this->qw = 1;

        if (equal(this->qw, static_cast<T>(0)) &&
            equal(this->qx, static_cast<T>(0)) &&
            equal(this->qy, static_cast<T>(0)) &&
            equal(this->qz, static_cast<T>(0)))
        {
          this->qw = 1;
        }
      }

      /// \brief Return the X axis
      /// \return the vector
      public: Vector3<T> GetXAxis() const
      {
        T fTy  = 2.0f*this->qy;
        T fTz  = 2.0f*this->qz;

        T fTwy = fTy*this->qw;
        T fTwz = fTz*this->qw;
        T fTxy = fTy*this->qx;
        T fTxz = fTz*this->qx;
        T fTyy = fTy*this->qy;
        T fTzz = fTz*this->qz;

        return Vector3<T>(1.0f-(fTyy+fTzz), fTxy+fTwz, fTxz-fTwy);
      }

      /// \brief Return the Y axis
      /// \return the vector
      public: Vector3<T> GetYAxis() const
      {
        T fTx  = 2.0f*this->qx;
        T fTy  = 2.0f*this->qy;
        T fTz  = 2.0f*this->qz;
        T fTwx = fTx*this->qw;
        T fTwz = fTz*this->qw;
        T fTxx = fTx*this->qx;
        T fTxy = fTy*this->qx;
        T fTyz = fTz*this->qy;
        T fTzz = fTz*this->qz;

        return Vector3<T>(fTxy-fTwz, 1.0f-(fTxx+fTzz), fTyz+fTwx);
      }

      /// \brief Return the Z axis
      /// \return the vector
      public: Vector3<T> GetZAxis() const
      {
        T fTx  = 2.0f*this->qx;
        T fTy  = 2.0f*this->qy;
        T fTz  = 2.0f*this->qz;
        T fTwx = fTx*this->qw;
        T fTwy = fTy*this->qw;
        T fTxx = fTx*this->qx;
        T fTxz = fTz*this->qx;
        T fTyy = fTy*this->qy;
        T fTyz = fTz*this->qy;

        return Vector3<T>(fTxz+fTwy, fTyz-fTwx, 1.0f-(fTxx+fTyy));
      }

      /// \brief Round all values to _precision decimal places
      /// \param[in] _precision the precision
      public: void Round(int _precision)
      {
        this->qx = precision(this->qx, _precision);
        this->qy = precision(this->qy, _precision);
        this->qz = precision(this->qz, _precision);
        this->qw = precision(this->qw, _precision);
      }

      /// \brief Dot product
      /// \param[in] _q the other quaternion
      /// \return the product
      public: T Dot(const Quaternion<T> &_q) const
      {
        return this->qw*_q.qw + this->qx * _q.qx +
               this->qy*_q.qy + this->qz*_q.qz;
      }

      /// \brief Spherical quadratic interpolation
      /// given the ends and an interpolation parameter between 0 and 1
      /// \param[in] _ft the interpolation parameter
      /// \param[in] _rkP the beginning quaternion
      /// \param[in] _rkA first intermediate quaternion
      /// \param[in] _rkB second intermediate quaternion
      /// \param[in] _rkQ the end quaternion
      /// \param[in] _shortestPath when true, the rotation may be inverted to
      /// get to minimize rotation
      public: static Quaternion<T> Squad(T _fT,
                  const Quaternion<T> &_rkP, const Quaternion<T> &_rkA,
                  const Quaternion<T> &_rkB, const Quaternion<T> &_rkQ,
                  bool _shortestPath = false)
      {
        T fSlerpT = 2.0f*_fT*(1.0f-_fT);
        Quaternion<T> kSlerpP = Slerp(_fT, _rkP, _rkQ, _shortestPath);
        Quaternion<T> kSlerpQ = Slerp(_fT, _rkA, _rkB);
        return Slerp(fSlerpT, kSlerpP, kSlerpQ);
      }

      /// \brief Spherical linear interpolation between 2 quaternions,
      /// given the ends and an interpolation parameter between 0 and 1
      /// \param[in] _ft the interpolation parameter
      /// \param[in] _rkP the beginning quaternion
      /// \param[in] _rkQ the end quaternion
      /// \param[in] _shortestPath when true, the rotation may be inverted to
      /// get to minimize rotation
      public: static Quaternion<T> Slerp(T _fT,
                  const Quaternion<T> &_rkP, const Quaternion<T> &_rkQ,
                  bool _shortestPath = false)
      {
        T fCos = _rkP.Dot(_rkQ);
        Quaternion<T> rkT;

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
          T fSin = sqrt(1 - (fCos*fCos));
          T fAngle = atan2(fSin, fCos);
          // FIXME: should check if (std::abs(fSin) >= 1e-3)
          T fInvSin = 1.0f / fSin;
          T fCoeff0 = sin((1.0f - _fT) * fAngle) * fInvSin;
          T fCoeff1 = sin(_fT * fAngle) * fInvSin;
          return _rkP * fCoeff0 + rkT * fCoeff1;
        }
        else
        {
          // There are two situations:
          // 1. "rkP" and "rkQ" are very close (fCos ~= +1),
          // so we can do a linear interpolation safely.
          // 2. "rkP" and "rkQ" are almost inverse of each
          // other (fCos ~= -1), there
          // are an infinite number of possibilities interpolation.
          // but we haven't have method to fix this case, so just use
          // linear interpolation here.
          Quaternion<T> t = _rkP * (1.0f - _fT) + rkT * _fT;
          // taking the complement requires renormalisation
          t.Normalize();
          return t;
        }
      }
      /// \brief Get the w component.
      /// \return The w quaternion component.
      public: inline T w() const
      {
        return this->qw;
      }

      /// \brief Get the x component.
      /// \return The x quaternion component.
      public: inline T x() const
      {
        return this->qx;
      }

      /// \brief Get the y component.
      /// \return The y quaternion component.
      public: inline T y() const
      {
        return this->qy;
      }

      /// \brief Get the z component.
      /// \return The z quaternion component.
      public: inline T z() const
      {
        return this->qz;
      }

      /// \brief Set the x component.
      /// \param[in] _v The new value for the x quaternion component.
      public: inline void x(T _v)
      {
        this->qx = _v;
      }

      /// \brief Set the y component.
      /// \param[in] _v The new value for the y quaternion component.
      public: inline void y(T _v)
      {
        this->qy = _v;
      }

      /// \brief Set the z component.
      /// \param[in] _v The new value for the z quaternion component.
      public: inline void z(T _v)
      {
        this->qz = _v;
      }

      /// \brief Set the w component.
      /// \param[in] _v The new value for the w quaternion component.
      public: inline void w(T _v)
      {
        this->qw = _v;
      }

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _q quaternion to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const ignition::math::Quaternion<T> &_q)
      {
        Vector3<T> v(_q.GetAsEuler());
        _out << precision(v.x(), 6) << " " << precision(v.y(), 6) << " "
             << precision(v.z(), 6);
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param[in] _in input stream
      /// \param[in] _q Quaternion<T> to read values into
      /// \return The istream
      public: friend std::istream &operator>>(std::istream &_in,
          ignition::math::Quaternion<T> &_q)
      {
        Angle roll, pitch, yaw;

        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> roll >> pitch >> yaw;

        _q.SetFromEuler(Vector3<T>(*roll, *pitch, *yaw));

        return _in;
      }

      /// \brief Attributes of the quaternion
      private: T qw;

      /// \brief Attributes of the quaternion
      private: T qx;

      /// \brief Attributes of the quaternion
      private: T qy;

      /// \brief Attributes of the quaternion
      private: T qz;
    };

    typedef Quaternion<double> Quaterniond;
    typedef Quaternion<float> Quaternionf;
    typedef Quaternion<int> Quaternioni;
  }
}
#endif

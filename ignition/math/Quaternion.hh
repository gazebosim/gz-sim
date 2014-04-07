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

#include <math.h>
#include <iostream>
#include <cmath>

#include "ignition/math/Helpers.hh"
#include "ignition/math/Angle.hh"
#include "ignition/math/Vector3d.hh"
#include "ignition/math/Matrix3.hh"
#include "ignition/math/Matrix4.hh"

namespace ignition
{
  namespace math
  {
    /// \class Quaternion Quaternion.hh ignition/math.hh
    /// \brief A quaternion class
    class Quaternion
    {
      /// \brief Default Constructor
      public: Quaternion();

      /// \brief Constructor
      /// \param[in] _w W param
      /// \param[in] _x X param
      /// \param[in] _y Y param
      /// \param[in] _z Z param
      public: Quaternion(const double &_w, const double &_x, const double &_y,
                          const double &_z);

      /// \brief Constructor from Euler angles in radians
      /// \param[in] _roll  roll
      /// \param[in] _pitch pitch
      /// \param[in] _yaw   yaw
      public: Quaternion(const double &_roll, const double &_pitch,
                          const double &_yaw);

      /// \brief Constructor from axis angle
      /// \param[in] _axis the rotation axis
      /// \param[in] _angle the rotation angle in radians
      public: Quaternion(const Vector3d &_axis, const double &_angle);

      /// \brief Constructor
      /// \param[in] _rpy euler angles
      public: Quaternion(const Vector3d &_rpy);

      /// \brief Copy constructor
      /// \param qt Quaternion to copy
      public: Quaternion(const Quaternion &_qt);

      /// \brief Destructor
      public: ~Quaternion();

      /// \brief Equal operator
      /// \param[in] _qt Quaternion to copy
      public: Quaternion &operator =(const Quaternion &_qt);

      /// \brief Invert the quaternion
      public: void Invert();

      /// \brief Get the inverse of this quaternion
      /// \return Inverse quarenion
      public: inline Quaternion GetInverse() const
              {
                double s = 0;
                Quaternion q(this->w, this->x, this->y, this->z);

                // use s to test if quaternion is valid
                s = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

                if (math::equal(s, 0.0))
                {
                  q.w = 1.0;
                  q.x = 0.0;
                  q.y = 0.0;
                  q.z = 0.0;
                }
                else
                {
                  // deal with non-normalized quaternion
                  // div by s so q * qinv = identity
                  q.w =  q.w / s;
                  q.x = -q.x / s;
                  q.y = -q.y / s;
                  q.z = -q.z / s;
                }
                return q;
              }

      /// \brief Set the quatern to the identity
      public: void SetToIdentity();

      /// \brief Return the logarithm
      /// \return the log
      public: Quaternion GetLog() const;

      /// \brief Return the exponent
      /// \return the exp
      public: Quaternion GetExp() const;

      /// \brief Normalize the quaternion
      public: void Normalize();

      /// \brief Set the quaternion from an axis and angle
      /// \param[in] _x X axis
      /// \param[in] _y Y axis
      /// \param[in] _z Z axis
      /// \param[in] _a Angle in radians
      public: void SetFromAxis(double _x, double _y, double _z, double _a);

      /// \brief Set the quaternion from an axis and angle
      /// \param[in] _axis Axis
      /// \param[in] _a Angle in radians
      public: void SetFromAxis(const Vector3d &_axis, double _a);

      /// \brief Set this quaternion from 4 floating numbers
      /// \param[in] _u u
      /// \param[in] _x x
      /// \param[in] _y y
      /// \param[in] _z z
      public: void Set(double _u, double _x, double _y, double _z);

      /// \brief Set the quaternion from Euler angles. The order of operations
      /// are roll, pitch, yaw.
      /// \param[in] vec  Euler angle
      public: void SetFromEuler(const Vector3d &_vec);

      /// \brief Set the quaternion from Euler angles.
      /// \param[in] _roll Roll angle (radians).
      /// \param[in] _pitch Roll angle (radians).
      /// \param[in] _yaw Roll angle (radians).
      public: void SetFromEuler(double _roll, double _pitch, double _yaw);

      /// \brief Return the rotation in Euler angles
      /// \return This quaternion as an Euler vector
      public: Vector3d GetAsEuler() const;

      /// \brief Convert euler angles to quatern.
      /// \param[in]
      public: static Quaternion EulerToQuaternion(const Vector3d &_vec);

      /// \brief Convert euler angles to quatern.
      /// \param[in] _x rotation along x
      /// \param[in] _y rotation along y
      /// \param[in] _z rotation along z
      public: static Quaternion EulerToQuaternion(double _x,
                                                  double _y,
                                                  double _z);

      /// \brief Get the Euler roll angle in radians
      /// \return the roll
      public: double GetRoll();

      /// \brief Get the Euler pitch angle in radians
      /// \return the pitch
      public: double GetPitch();

      /// \brief Get the Euler yaw angle in radians
      /// \return the yaw
      public: double GetYaw();

      /// \brief Return rotation as axis and angle
      /// \param[in] _axis rotation axis
      /// \param[in] _angle ccw angle in radians
      public: void GetAsAxis(Vector3d &_axis, double &_angle) const;

      /// \brief Scale a Quaternionion
      /// \param[in] _scale Amount to scale this rotation
      public: void Scale(double _scale);

      /// \brief Addition operator
      /// \param[in] _qt quaternion for addition
      /// \return this quaternion + _qt
      public: Quaternion operator+(const Quaternion &_qt) const;

      /// \brief Addition operator
      /// \param[in] _qt quaternion for addition
      /// \return this quaternion + qt
      public: Quaternion operator+=(const Quaternion &_qt);

      /// \brief Substraction operator
      /// \param[in] _qt quaternion to substract
      /// \return this quaternion - _qt
      public: Quaternion operator-(const Quaternion &_qt) const;

      /// \brief Substraction operator
      /// \param[in] _qt Quaternion for substraction
      /// \return This quatern - qt
      public: Quaternion operator-=(const Quaternion &_qt);

      /// \brief Multiplication operator
      /// \param[in] _qt Quaternion for multiplication
      /// \return This quaternion multiplied by the parameter
      public: inline Quaternion operator*(const Quaternion &_q) const
              {
                return Quaternion(
                    this->w*_q.w - this->x*_q.x - this->y*_q.y - this->z*_q.z,
                    this->w*_q.x + this->x*_q.w + this->y*_q.z - this->z*_q.y,
                    this->w*_q.y - this->x*_q.z + this->y*_q.w + this->z*_q.x,
                    this->w*_q.z + this->x*_q.y - this->y*_q.x + this->z*_q.w);
              }

      /// \brief Multiplication operator
      /// \param[in] _f factor
      /// \return quaternion multiplied by _f
      public: Quaternion operator*(const double &_f) const;

      /// \brief Multiplication operator
      /// \param[in] _qt Quaternion for multiplication
      /// \return This quatern multiplied by the parameter
      public: Quaternion operator*=(const Quaternion &qt);

      /// \brief Vector3 multiplication operator
      /// \param[in] _v vector to multiply
      public: Vector3d operator*(const Vector3d &_v) const;

      /// \brief Equal to operator
      /// \param[in] _qt Quaternion for comparison
      /// \return True if equal
      public: bool operator ==(const Quaternion &_qt) const;

      /// \brief Not equal to operator
      /// \param[in] _qt Quaternion for comparison
      /// \return True if not equal
      public: bool operator!=(const Quaternion &_qt) const;

      /// \brief Unary minus operator
      /// \return negates each component of the quaternion
      public: Quaternion operator-() const;

      /// \brief Rotate a vector using the quaternion
      /// \param[in] _vec vector to rotate
      /// \return the rotated vector
      public: inline Vector3d RotateVector(const Vector3d &_vec) const
              {
                Quaternion tmp(0.0, _vec.x(), _vec.y(), _vec.z());
                tmp = (*this) * (tmp * this->GetInverse());
                return Vector3d(tmp.x, tmp.y, tmp.z);
              }

      /// \brief Do the reverse rotation of a vector by this quaternion
      /// \param[in] _vec the vector
      /// \return the
      public: Vector3d RotateVectorReverse(Vector3d _vec) const;

      /// \brief See if a quatern is finite (e.g., not nan)
      /// \return True if quatern is finite
      public: bool IsFinite() const;

      /// \brief Correct any nan
      public: inline void Correct()
              {
                if (!finite(this->x))
                  this->x = 0;
                if (!finite(this->y))
                  this->y = 0;
                if (!finite(this->z))
                  this->z = 0;
                if (!finite(this->w))
                  this->w = 1;

                if (math::equal(this->w, 0.0) &&
                    math::equal(this->x, 0.0) &&
                    math::equal(this->y, 0.0) &&
                    math::equal(this->z, 0.0))
                {
                  this->w = 1;
                }
              }

      /// \brief Get the quaternion as a 3x3 matrix
      public: Matrix3 GetAsMatrix3() const;

      /// \brief Get the quaternion as a 4x4 matrix
      /// \return a 4x4 matrix
      public: Matrix4 GetAsMatrix4() const;

      /// \brief Return the X axis
      /// \return the vector
      public: Vector3d GetXAxis() const;

      /// \brief Return the Y axis
      /// \return the vector
      public: Vector3d GetYAxis() const;

      /// \brief Return the Z axis
      /// \return the vector
      public: Vector3d GetZAxis() const;

      /// \brief Round all values to _precision decimal places
      /// \param[in] _precision the precision
      public: void Round(int _precision);

      /// \brief Dot product
      /// \param[in] _q the other quaternion
      /// \return the product
      public: double Dot(const Quaternion &_q) const;

      /// \brief Spherical quadratic interpolation
      /// given the ends and an interpolation parameter between 0 and 1
      /// \param[in] _ft the interpolation parameter
      /// \param[in] _rkP the beginning quaternion
      /// \param[in] _rkA first intermediate quaternion
      /// \param[in] _rkB second intermediate quaternion
      /// \param[in] _rkQ the end quaternion
      /// \param[in] _shortestPath when true, the rotation may be inverted to
      /// get to minimize rotation
      public: static Quaternion Squad(double _fT, const Quaternion &_rkP,
                  const Quaternion &_rkA, const Quaternion &_rkB,
                  const Quaternion &_rkQ, bool _shortestPath = false);

      /// \brief Spherical linear interpolation between 2 quaternions,
      /// given the ends and an interpolation parameter between 0 and 1
      /// \param[in] _ft the interpolation parameter
      /// \param[in] _rkP the beginning quaternion
      /// \param[in] _rkQ the end quaternion
      /// \param[in] _shortestPath when true, the rotation may be inverted to
      /// get to minimize rotation
      public: static Quaternion Slerp(double _fT, const Quaternion &_rkP,
                  const Quaternion &_rkQ, bool _shortestPath = false);


      /// \brief Attributes of the quaternion
      public: double w;

      /// \brief Attributes of the quaternion
      public: double x;

      /// \brief Attributes of the quaternion
      public: double y;

      /// \brief Attributes of the quaternion
      public: double z;

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _q quaternion to output
      /// \return the stream
      public: friend  std::ostream &operator<<(std::ostream &_out,
                  const ignition::math::Quaternion &_q)
      {
        Vector3d v(_q.GetAsEuler());
        _out << precision(v.x(), 6) << " " << precision(v.y(), 6) << " "
             << precision(v.z(), 6);
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param[in] _in input stream
      /// \param[in] _q Quaternion to read values into
      /// \return The istream
      public: friend std::istream &operator>>(std::istream &_in,
                                               ignition::math::Quaternion &_q)
      {
        Angle roll, pitch, yaw;

        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> roll >> pitch >> yaw;

        _q.SetFromEuler(Vector3d(*roll, *pitch, *yaw));

        return _in;
      }
    };
  }
}
#endif


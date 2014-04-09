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
#ifndef IGN_QUATERNION
#error This class should not be used directly. Use Quaterniond.hh,\
Quaterniond.hh, or Quaterioni.hh.
#endif

class IGN_MATRIX3;
class IGN_MATRIX4;

/// \class IGN_QUATERNION IGN_QUATERNION.hh ignition/math.hh
/// \brief A quaternion class
class IGN_QUATERNION
{
  /// \brief Default Constructor
  public: IGN_QUATERNION();

  /// \brief Constructor
  /// \param[in] _w W param
  /// \param[in] _x X param
  /// \param[in] _y Y param
  /// \param[in] _z Z param
  public: IGN_QUATERNION(const IGN_NUMERIC &_w, const IGN_NUMERIC &_x,
              const IGN_NUMERIC &_y, const IGN_NUMERIC &_z);

  /// \brief Constructor from Euler angles in radians
  /// \param[in] _roll  roll
  /// \param[in] _pitch pitch
  /// \param[in] _yaw   yaw
  public: IGN_QUATERNION(const IGN_NUMERIC &_roll, const IGN_NUMERIC &_pitch,
              const IGN_NUMERIC &_yaw);

  /// \brief Constructor from axis angle
  /// \param[in] _axis the rotation axis
  /// \param[in] _angle the rotation angle in radians
  public: IGN_QUATERNION(const IGN_VECTOR3 &_axis, const IGN_NUMERIC &_angle);

  /// \brief Constructor
  /// \param[in] _rpy euler angles
  public: IGN_QUATERNION(const IGN_VECTOR3 &_rpy);

  /// \brief Copy constructor
  /// \param qt IGN_QUATERNION to copy
  public: IGN_QUATERNION(const IGN_QUATERNION &_qt);

  /// \brief Destructor
  public: ~IGN_QUATERNION();

  /// \brief Equal operator
  /// \param[in] _qt IGN_QUATERNION to copy
  public: IGN_QUATERNION &operator =(const IGN_QUATERNION &_qt);

  /// \brief Invert the quaternion
  public: void Invert();

  /// \brief Get the inverse of this quaternion
  /// \return Inverse quarenion
  public: inline IGN_QUATERNION GetInverse() const
          {
            IGN_NUMERIC s = 0;
            IGN_QUATERNION q(this->qw, this->qx, this->qy, this->qz);

            // use s to test if quaternion is valid
            s = q.qw * q.qw + q.qx * q.qx + q.qy * q.qy + q.qz * q.qz;

            if (math::equal(s, static_cast<IGN_NUMERIC>(0)))
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
  public: void SetToIdentity();

  /// \brief Return the logarithm
  /// \return the log
  public: IGN_QUATERNION GetLog() const;

  /// \brief Return the exponent
  /// \return the exp
  public: IGN_QUATERNION GetExp() const;

  /// \brief Normalize the quaternion
  public: void Normalize();

  /// \brief Set the quaternion from an axis and angle
  /// \param[in] _x X axis
  /// \param[in] _y Y axis
  /// \param[in] _z Z axis
  /// \param[in] _a Angle in radians
  public: void SetFromAxis(IGN_NUMERIC _x, IGN_NUMERIC _y,
              IGN_NUMERIC _z, IGN_NUMERIC _a);

  /// \brief Set the quaternion from an axis and angle
  /// \param[in] _axis Axis
  /// \param[in] _a Angle in radians
  public: void SetFromAxis(const IGN_VECTOR3 &_axis, IGN_NUMERIC _a);

  /// \brief Set this quaternion from 4 floating numbers
  /// \param[in] _u u
  /// \param[in] _x x
  /// \param[in] _y y
  /// \param[in] _z z
  public: void Set(IGN_NUMERIC _u, IGN_NUMERIC _x,
              IGN_NUMERIC _y, IGN_NUMERIC _z);

  /// \brief Set the quaternion from Euler angles. The order of operations
  /// are roll, pitch, yaw.
  /// \param[in] vec  Euler angle
  public: void SetFromEuler(const IGN_VECTOR3 &_vec);

  /// \brief Set the quaternion from Euler angles.
  /// \param[in] _roll Roll angle (radians).
  /// \param[in] _pitch Roll angle (radians).
  /// \param[in] _yaw Roll angle (radians).
  public: void SetFromEuler(IGN_NUMERIC _roll, IGN_NUMERIC _pitch,
              IGN_NUMERIC _yaw);

  /// \brief Return the rotation in Euler angles
  /// \return This quaternion as an Euler vector
  public: IGN_VECTOR3 GetAsEuler() const;

  /// \brief Convert euler angles to quatern.
  /// \param[in]
  public: static IGN_QUATERNION EulerToQuaternion(const IGN_VECTOR3 &_vec);

  /// \brief Convert euler angles to quatern.
  /// \param[in] _x rotation along x
  /// \param[in] _y rotation along y
  /// \param[in] _z rotation along z
  public: static IGN_QUATERNION EulerToQuaternion(IGN_NUMERIC _x,
                                                  IGN_NUMERIC _y,
                                                  IGN_NUMERIC _z);

  /// \brief Get the Euler roll angle in radians
  /// \return the roll
  public: IGN_NUMERIC GetRoll();

  /// \brief Get the Euler pitch angle in radians
  /// \return the pitch
  public: IGN_NUMERIC GetPitch();

  /// \brief Get the Euler yaw angle in radians
  /// \return the yaw
  public: IGN_NUMERIC GetYaw();

  /// \brief Return rotation as axis and angle
  /// \param[in] _axis rotation axis
  /// \param[in] _angle ccw angle in radians
  public: void GetAsAxis(IGN_VECTOR3 &_axis, IGN_NUMERIC &_angle) const;

  /// \brief Scale a IGN_QUATERNIONion
  /// \param[in] _scale Amount to scale this rotation
  public: void Scale(IGN_NUMERIC _scale);

  /// \brief Addition operator
  /// \param[in] _qt quaternion for addition
  /// \return this quaternion + _qt
  public: IGN_QUATERNION operator+(const IGN_QUATERNION &_qt) const;

  /// \brief Addition operator
  /// \param[in] _qt quaternion for addition
  /// \return this quaternion + qt
  public: IGN_QUATERNION operator+=(const IGN_QUATERNION &_qt);

  /// \brief Substraction operator
  /// \param[in] _qt quaternion to substract
  /// \return this quaternion - _qt
  public: IGN_QUATERNION operator-(const IGN_QUATERNION &_qt) const;

  /// \brief Substraction operator
  /// \param[in] _qt IGN_QUATERNION for substraction
  /// \return This quatern - qt
  public: IGN_QUATERNION operator-=(const IGN_QUATERNION &_qt);

  /// \brief Multiplication operator
  /// \param[in] _qt IGN_QUATERNION for multiplication
  /// \return This quaternion multiplied by the parameter
  public: inline IGN_QUATERNION operator*(const IGN_QUATERNION &_q) const
          {
            return IGN_QUATERNION(
                this->qw*_q.qw-this->qx*_q.qx-this->qy*_q.qy-this->qz*_q.qz,
                this->qw*_q.qx+this->qx*_q.qw+this->qy*_q.qz-this->qz*_q.qy,
                this->qw*_q.qy-this->qx*_q.qz+this->qy*_q.qw+this->qz*_q.qx,
                this->qw*_q.qz+this->qx*_q.qy-this->qy*_q.qx+this->qz*_q.qw);
          }

  /// \brief Multiplication operator
  /// \param[in] _f factor
  /// \return quaternion multiplied by _f
  public: IGN_QUATERNION operator*(const IGN_NUMERIC &_f) const;

  /// \brief Multiplication operator
  /// \param[in] _qt IGN_QUATERNION for multiplication
  /// \return This quatern multiplied by the parameter
  public: IGN_QUATERNION operator*=(const IGN_QUATERNION &qt);

  /// \brief Vector3 multiplication operator
  /// \param[in] _v vector to multiply
  public: IGN_VECTOR3 operator*(const IGN_VECTOR3 &_v) const;

  /// \brief Equal to operator
  /// \param[in] _qt IGN_QUATERNION for comparison
  /// \return True if equal
  public: bool operator ==(const IGN_QUATERNION &_qt) const;

  /// \brief Not equal to operator
  /// \param[in] _qt IGN_QUATERNION for comparison
  /// \return True if not equal
  public: bool operator!=(const IGN_QUATERNION &_qt) const;

  /// \brief Unary minus operator
  /// \return negates each component of the quaternion
  public: IGN_QUATERNION operator-() const;

  /// \brief Rotate a vector using the quaternion
  /// \param[in] _vec vector to rotate
  /// \return the rotated vector
  public: inline IGN_VECTOR3 RotateVector(const IGN_VECTOR3 &_vec) const
          {
            IGN_QUATERNION tmp(static_cast<IGN_NUMERIC>(0),
                _vec.x(), _vec.y(), _vec.z());
            tmp = (*this) * (tmp * this->GetInverse());
            return IGN_VECTOR3(tmp.qx, tmp.qy, tmp.qz);
          }

  /// \brief Do the reverse rotation of a vector by this quaternion
  /// \param[in] _vec the vector
  /// \return the
  public: IGN_VECTOR3 RotateVectorReverse(IGN_VECTOR3 _vec) const;

  /// \brief See if a quatern is finite (e.g., not nan)
  /// \return True if quatern is finite
  public: bool IsFinite() const;

  /// \brief Correct any nan
  public: inline void Correct()
          {
            if (!finite(this->qx))
              this->qx = 0;
            if (!finite(this->qy))
              this->qy = 0;
            if (!finite(this->qz))
              this->qz = 0;
            if (!finite(this->qw))
              this->qw = 1;

            if (math::equal(this->qw, static_cast<IGN_NUMERIC>(0)) &&
                math::equal(this->qx, static_cast<IGN_NUMERIC>(0)) &&
                math::equal(this->qy, static_cast<IGN_NUMERIC>(0)) &&
                math::equal(this->qz, static_cast<IGN_NUMERIC>(0)))
            {
              this->qw = 1;
            }
          }

  /// \brief Get the quaternion as a 3x3 matrix
  public: IGN_MATRIX3 GetAsMatrix3() const;

  /// \brief Get the quaternion as a 4x4 matrix
  /// \return a 4x4 matrix
  public: IGN_MATRIX4 GetAsMatrix4() const;

  /// \brief Return the X axis
  /// \return the vector
  public: IGN_VECTOR3 GetXAxis() const;

  /// \brief Return the Y axis
  /// \return the vector
  public: IGN_VECTOR3 GetYAxis() const;

  /// \brief Return the Z axis
  /// \return the vector
  public: IGN_VECTOR3 GetZAxis() const;

  /// \brief Round all values to _precision decimal places
  /// \param[in] _precision the precision
  public: void Round(int _precision);

  /// \brief Dot product
  /// \param[in] _q the other quaternion
  /// \return the product
  public: IGN_NUMERIC Dot(const IGN_QUATERNION &_q) const;

  /// \brief Spherical quadratic interpolation
  /// given the ends and an interpolation parameter between 0 and 1
  /// \param[in] _ft the interpolation parameter
  /// \param[in] _rkP the beginning quaternion
  /// \param[in] _rkA first intermediate quaternion
  /// \param[in] _rkB second intermediate quaternion
  /// \param[in] _rkQ the end quaternion
  /// \param[in] _shortestPath when true, the rotation may be inverted to
  /// get to minimize rotation
  public: static IGN_QUATERNION Squad(IGN_NUMERIC _fT,
              const IGN_QUATERNION &_rkP, const IGN_QUATERNION &_rkA,
              const IGN_QUATERNION &_rkB, const IGN_QUATERNION &_rkQ,
              bool _shortestPath = false);

  /// \brief Spherical linear interpolation between 2 quaternions,
  /// given the ends and an interpolation parameter between 0 and 1
  /// \param[in] _ft the interpolation parameter
  /// \param[in] _rkP the beginning quaternion
  /// \param[in] _rkQ the end quaternion
  /// \param[in] _shortestPath when true, the rotation may be inverted to
  /// get to minimize rotation
  public: static IGN_QUATERNION Slerp(IGN_NUMERIC _fT,
              const IGN_QUATERNION &_rkP, const IGN_QUATERNION &_rkQ,
              bool _shortestPath = false);

  /// \brief Get the w component.
  /// \return The w quaternion component.
  public: inline IGN_NUMERIC w() const
          {
            return this->qw;
          }

  /// \brief Get the x component.
  /// \return The x quaternion component.
  public: inline IGN_NUMERIC x() const
          {
            return this->qx;
          }

  /// \brief Get the y component.
  /// \return The y quaternion component.
  public: inline IGN_NUMERIC y() const
          {
            return this->qy;
          }

  /// \brief Get the z component.
  /// \return The z quaternion component.
  public: inline IGN_NUMERIC z() const
          {
            return this->qz;
          }

  /// \brief Set the x component.
  /// \param[in] _v The new value for the x quaternion component.
  public: inline void x(IGN_NUMERIC _v)
          {
            this->qx = _v;
          }

  /// \brief Set the y component.
  /// \param[in] _v The new value for the y quaternion component.
  public: inline void y(IGN_NUMERIC _v)
          {
            this->qy = _v;
          }

  /// \brief Set the z component.
  /// \param[in] _v The new value for the z quaternion component.
  public: inline void z(IGN_NUMERIC _v)
          {
            this->qz = _v;
          }

  /// \brief Set the w component.
  /// \param[in] _v The new value for the w quaternion component.
  public: inline void w(IGN_NUMERIC _v)
          {
            this->qw = _v;
          }

  /// \brief Attributes of the quaternion
  private: IGN_NUMERIC qw;

  /// \brief Attributes of the quaternion
  private: IGN_NUMERIC qx;

  /// \brief Attributes of the quaternion
  private: IGN_NUMERIC qy;

  /// \brief Attributes of the quaternion
  private: IGN_NUMERIC qz;
};

/// \brief Stream insertion operator
/// \param[in] _out output stream
/// \param[in] _q quaternion to output
/// \return the stream
std::ostream &operator<<(std::ostream &_out,
            const ignition::math::IGN_QUATERNION &_q);

/// \brief Stream extraction operator
/// \param[in] _in input stream
/// \param[in] _q IGN_QUATERNION to read values into
/// \return The istream
std::istream &operator>>(std::istream &_in,
    ignition::math::IGN_QUATERNION &_q);

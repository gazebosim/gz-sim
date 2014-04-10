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
#ifndef IGN_POSE3
#error This class should not be used directly. Use Pose3d.hh, \
Pose3f.hh, or Pose3i.hh.
#endif

/// \class Pose3 Pose3.hh ignition/math.hh
/// \brief Encapsulates a position and rotation in three space
class IGN_POSE3
{
  /// \brief math::IGN_POSE3(0, 0, 0, 0, 0, 0)
  public: static const IGN_POSE3 Zero;

  /// \brief Default constructors
  public: IGN_POSE3();

  /// \brief Constructor
  /// \param[in] _pos A position
  /// \param[in] _rot A rotation
  public: IGN_POSE3(const IGN_VECTOR3 &_pos, const IGN_QUATERNION &_rot);

  /// \brief Constructor
  /// \param[in] _x x position in meters.
  /// \param[in] _y y position in meters.
  /// \param[in] _z z position in meters.
  /// \param[in] _roll Roll (rotation about X-axis) in radians.
  /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
  /// \param[in] _yaw Yaw (rotation about z-axis) in radians.
  public: IGN_POSE3(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z,
               IGN_NUMERIC _roll, IGN_NUMERIC _pitch, IGN_NUMERIC _yaw);

  /// \brief Constructor
  /// \param[in] _x x position in meters.
  /// \param[in] _y y position in meters.
  /// \param[in] _z z position in meters.
  /// \param[in] _qw Quaternion w value.
  /// \param[in] _qx Quaternion x value.
  /// \param[in] _qy Quaternion y value.
  /// \param[in] _qz Quaternion z value.
  public: IGN_POSE3(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z,
              IGN_NUMERIC _qw, IGN_NUMERIC _qx,
              IGN_NUMERIC _qy, IGN_NUMERIC _qz);

  /// \brief Copy constructor
  /// \param[in] _pose IGN_POSE3 to copy
  public: IGN_POSE3(const IGN_POSE3 &_pose);

  /// \brief Destructor
  public: virtual ~IGN_POSE3();

  /// \brief Set the pose from a Vector3 and a IGN_QUATERNION
  /// \param[in] _pos The position.
  /// \param[in] _rot The rotation.
  public: void Set(const IGN_VECTOR3 &_pos, const IGN_QUATERNION &_rot);

  /// \brief Set the pose from  pos and rpy vectors
  /// \param[in] _pos The position.
  /// \param[in] _rpy The rotation expressed as Euler angles.
  public: void Set(const IGN_VECTOR3 &_pos, const IGN_VECTOR3 &_rpy);

  /// \brief Set the pose from a six tuple.
  /// \param[in] _x x position in meters.
  /// \param[in] _y y position in meters.
  /// \param[in] _z z position in meters.
  /// \param[in] _roll Roll (rotation about X-axis) in radians.
  /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
  /// \param[in] _yaw Pitch (rotation about z-axis) in radians.
  public: void Set(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z,
                   IGN_NUMERIC _roll, IGN_NUMERIC _pitch, IGN_NUMERIC _yaw);

  /// \brief See if a pose is finite (e.g., not nan)
  public: bool IsFinite() const;

  /// \brief Fix any nan values
  public: inline void Correct()
          {
            this->p.Correct();
            this->q.Correct();
          }

  /// \brief Get the inverse of this pose
  /// \return the inverse pose
  public: IGN_POSE3 GetInverse() const;

  /// \brief Addition operator
  /// A is the transform from O to P specified in frame O
  /// B is the transform from P to Q specified in frame P
  /// then, B + A is the transform from O to Q specified in frame O
  /// \param[in] _pose IGN_POSE3 to add to this pose
  /// \return The resulting pose
  public: IGN_POSE3 operator+(const IGN_POSE3 &_pose) const;

  /// \brief Add-Equals operator
  /// \param[in] _pose IGN_POSE3 to add to this pose
  /// \return The resulting pose
  public: const IGN_POSE3 &operator+=(const IGN_POSE3 &_pose);

  /// \brief Negation operator
  /// A is the transform from O to P in frame O
  /// then -A is transform from P to O specified in frame P
  /// \return The resulting pose
  public: inline IGN_POSE3 operator-() const
          {
            return IGN_POSE3() - *this;
          }

  /// \brief Subtraction operator
  /// A is the transform from O to P in frame O
  /// B is the transform from O to Q in frame O
  /// B - A is the transform from P to Q in frame P
  /// \param[in] _pose IGN_POSE3 to subtract from this one
  /// \return The resulting pose
  public: inline IGN_POSE3 operator-(const IGN_POSE3 &_pose) const
          {
            return IGN_POSE3(this->CoordPositionSub(_pose),
                        this->CoordRotationSub(_pose.q));
          }

  /// \brief Subtraction operator
  /// \param[in] _pose IGN_POSE3 to subtract from this one
  /// \return The resulting pose
  public: const IGN_POSE3 &operator-=(const IGN_POSE3 &_pose);

  /// \brief Equality operator
  /// \param[in] _pose IGN_POSE3 for comparison
  /// \return True if equal
  public: bool operator ==(const IGN_POSE3 &_pose) const;

  /// \brief Inequality operator
  /// \param[in] _pose IGN_POSE3 for comparison
  /// \return True if not equal
  public: bool operator!=(const IGN_POSE3 &_pose) const;

  /// \brief Multiplication operator
  /// \param[in] _pose the other pose
  /// \return itself
  public: IGN_POSE3 operator*(const IGN_POSE3 &_pose);

  /// \brief Equal operator
  /// \param[in] _pose IGN_POSE3 to copy
  public: IGN_POSE3 &operator=(const IGN_POSE3 &_pose);

  /// \brief Add one point to a vector: result = this + pos
  /// \param[in] _pos Position to add to this pose
  /// \return the resulting position
  public: IGN_VECTOR3 CoordPositionAdd(const IGN_VECTOR3 &_pos) const;

  /// \brief Add one point to another: result = this + pose
  /// \param[in] _pose The IGN_POSE3 to add
  /// \return The resulting position
  public: IGN_VECTOR3 CoordPositionAdd(const IGN_POSE3 &_pose) const;

  /// \brief Subtract one position from another: result = this - pose
  /// \param[in] _pose IGN_POSE3 to subtract
  /// \return The resulting position
  public: inline IGN_VECTOR3 CoordPositionSub(const IGN_POSE3 &_pose) const
          {
            IGN_QUATERNION tmp(0,
                this->p.x() - _pose.p.x(),
                this->p.y() - _pose.p.y(),
                this->p.z() - _pose.p.z());

            tmp = _pose.q.GetInverse() * (tmp * _pose.q);
            return IGN_VECTOR3(tmp.x(), tmp.y(), tmp.z());
          }

  /// \brief Add one rotation to another: result =  this->q + rot
  /// \param[in] _rot Rotation to add
  /// \return The resulting rotation
  public: IGN_QUATERNION CoordRotationAdd(const IGN_QUATERNION &_rot) const;

  /// \brief Subtract one rotation from another: result = this->q - rot
  /// \param[in] _rot The rotation to subtract
  /// \return The resulting rotation
  public: inline IGN_QUATERNION CoordRotationSub(
              const IGN_QUATERNION &_rot) const
          {
            IGN_QUATERNION result(_rot.GetInverse() * this->q);
            result.Normalize();
            return result;
          }

  /// \brief Find the inverse of a pose; i.e., if b = this + a, given b and
  ///        this, find a
  /// \param[in] _b the other pose
  public: IGN_POSE3 CoordPoseSolve(const IGN_POSE3 &_b) const;

  /// \brief Reset the pose
  public: void Reset();

  /// \brief Rotate vector part of a pose about the origin
  /// \param[in] _rot rotation
  /// \return the rotated pose
  public: IGN_POSE3 RotatePositionAboutOrigin(const IGN_QUATERNION &_rot) const;

  /// \brief Round all values to _precision decimal places
  /// \param[in] _precision
  public: void Round(int _precision);

  /// \brief Get the position.
  /// \return Origin of the pose.
  public: inline const IGN_VECTOR3 &pos() const
          {
            return this->p;
          }

  /// \brief Get a mutable reference to the position.
  /// \return Origin of the pose.
  public: inline IGN_VECTOR3 &pos()
          {
            return this->p;
          }

  /// \brief Get the rotation.
  /// \return Quaternion representation of the rotation.
  public: inline const IGN_QUATERNION &rot() const
          {
            return this->q;
          }

  /// \brief Get a mutuable reference to the rotation.
  /// \return Quaternion representation of the rotation.
  public: inline IGN_QUATERNION &rot()
          {
            return this->q;
          }

  /// \brief The position
  private: IGN_VECTOR3 p;

  /// \brief The rotation
  private: IGN_QUATERNION q;
};

/// \brief Stream insertion operator
/// \param[in] _out output stream
/// \param[in] _pose pose to output
/// \return the stream
std::ostream &operator<<(std::ostream &_out,
    const ignition::math::IGN_POSE3 &_pose);

/// \brief Stream extraction operator
/// \param[in] _in the input stream
/// \param[in] _pose the pose
/// \return the stream
std::istream &operator>>(std::istream &_in,
    ignition::math::IGN_POSE3 &_pose);

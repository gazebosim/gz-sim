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
#ifndef _IGNITION_POSE_HH_
#define _IGNITION_POSE_HH_

#include <iostream>

#include "ignition/math/Vector3d.hh"
#include "ignition/math/Quaternion.hh"

namespace ignition
{
  namespace math
  {
    /// \class Pose Pose.hh ignition/math.hh
    /// \brief Encapsulates a position and rotation in three space
    class Pose
    {
      /// \brief math::Pose(0, 0, 0, 0, 0, 0)
      public: static const Pose Zero;

      /// \brief Default constructors
      public: Pose();

      /// \brief Constructor
      /// \param[in] _pos A position
      /// \param[in] _rot A rotation
      public: Pose(const Vector3d &_pos, const Quaternion &_rot);

      /// \brief Constructor
      /// \param[in] _x x position in meters.
      /// \param[in] _y y position in meters.
      /// \param[in] _z z position in meters.
      /// \param[in] _roll Roll (rotation about X-axis) in radians.
      /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
      /// \param[in] _yaw Yaw (rotation about z-axis) in radians.
      public: Pose(double _x, double _y, double _z,
                   double _roll, double _pitch, double _yaw);

      /// \brief Constructor
      /// \param[in] _x x position in meters.
      /// \param[in] _y y position in meters.
      /// \param[in] _z z position in meters.
      /// \param[in] _qw Quaternion x value.
      /// \param[in] _qx Quaternion y value.
      /// \param[in] _qy Quaternion z value.
      /// \param[in] _qz Quaternion w value.
      public: Pose(double _x, double _y, double _z,
                   double _qw, double _qx, double _qy, double _qz);

      /// \brief Copy constructor
      /// \param[in] _pose Pose to copy
      public: Pose(const Pose &_pose);

      /// \brief Destructor
      public: virtual ~Pose();

      /// \brief Set the pose from a Vector3 and a Quaternion
      /// \param[in] _pos The position.
      /// \param[in] _rot The rotation.
      public: void Set(const Vector3d &_pos, const Quaternion &_rot);

      /// \brief Set the pose from  pos and rpy vectors
      /// \param[in] _pos The position.
      /// \param[in] _rpy The rotation expressed as Euler angles.
      public: void Set(const Vector3d &_pos, const Vector3d &_rpy);

      /// \brief Set the pose from a six tuple.
      /// \param[in] _x x position in meters.
      /// \param[in] _y y position in meters.
      /// \param[in] _z z position in meters.
      /// \param[in] _roll Roll (rotation about X-axis) in radians.
      /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
      /// \param[in] _yaw Pitch (rotation about z-axis) in radians.
      public: void Set(double _x, double _y, double _z,
                       double _roll, double _pitch, double _yaw);

      /// \brief See if a pose is finite (e.g., not nan)
      public: bool IsFinite() const;

      /// \brief Fix any nan values
      public: inline void Correct()
              {
                this->pos.Correct();
                this->rot.Correct();
              }

      /// \brief Get the inverse of this pose
      /// \return the inverse pose
      public: Pose GetInverse() const;

      /// \brief Addition operator
      /// A is the transform from O to P specified in frame O
      /// B is the transform from P to Q specified in frame P
      /// then, B + A is the transform from O to Q specified in frame O
      /// \param[in] _pose Pose to add to this pose
      /// \return The resulting pose
      public: Pose operator+(const Pose &_pose) const;

      /// \brief Add-Equals operator
      /// \param[in] _pose Pose to add to this pose
      /// \return The resulting pose
      public: const Pose &operator+=(const Pose &_pose);

      /// \brief Negation operator
      /// A is the transform from O to P in frame O
      /// then -A is transform from P to O specified in frame P
      /// \return The resulting pose
      public: inline Pose operator-() const
              {
                return Pose() - *this;
              }

      /// \brief Subtraction operator
      /// A is the transform from O to P in frame O
      /// B is the transform from O to Q in frame O
      /// B - A is the transform from P to Q in frame P
      /// \param[in] _pose Pose to subtract from this one
      /// \return The resulting pose
      public: inline Pose operator-(const Pose &_pose) const
              {
                return Pose(this->CoordPositionSub(_pose),
                            this->CoordRotationSub(_pose.rot));
              }

      /// \brief Subtraction operator
      /// \param[in] _pose Pose to subtract from this one
      /// \return The resulting pose
      public: const Pose &operator-=(const Pose &_pose);

      /// \brief Equality operator
      /// \param[in] _pose Pose for comparison
      /// \return True if equal
      public: bool operator ==(const Pose &_pose) const;

      /// \brief Inequality operator
      /// \param[in] _pose Pose for comparison
      /// \return True if not equal
      public: bool operator!=(const Pose &_pose) const;

      /// \brief Multiplication operator
      /// \param[in] _pose the other pose
      /// \return itself
      public: Pose operator*(const Pose &_pose);

      /// \brief Equal operator
      /// \param[in] _pose Pose to copy
      public: Pose &operator=(const Pose &_pose);

      /// \brief Add one point to a vector: result = this + pos
      /// \param[in] _pos Position to add to this pose
      /// \return the resulting position
      public: Vector3d CoordPositionAdd(const Vector3d &_pos) const;

      /// \brief Add one point to another: result = this + pose
      /// \param[in] _pose The Pose to add
      /// \return The resulting position
      public: Vector3d CoordPositionAdd(const Pose &_pose) const;

      /// \brief Subtract one position from another: result = this - pose
      /// \param[in] _pose Pose to subtract
      /// \return The resulting position
      public: inline Vector3d CoordPositionSub(const Pose &_pose) const
              {
                Quaternion tmp(0.0,
                    this->pos.x() - _pose.pos.x(),
                    this->pos.y() - _pose.pos.y(),
                    this->pos.z() - _pose.pos.z());

                tmp = _pose.rot.GetInverse() * (tmp * _pose.rot);
                return Vector3d(tmp.x, tmp.y, tmp.z);
              }

      /// \brief Add one rotation to another: result =  this->rot + rot
      /// \param[in] _rot Rotation to add
      /// \return The resulting rotation
      public: Quaternion CoordRotationAdd(const Quaternion &_rot) const;

      /// \brief Subtract one rotation from another: result = this->rot - rot
      /// \param[in] _rot The rotation to subtract
      /// \return The resulting rotation
      public: inline Quaternion CoordRotationSub(const Quaternion &_rot) const
              {
                Quaternion result(_rot.GetInverse() * this->rot);
                result.Normalize();
                return result;
              }

      /// \brief Find the inverse of a pose; i.e., if b = this + a, given b and
      ///        this, find a
      /// \param[in] _b the other pose
      public: Pose CoordPoseSolve(const Pose &_b) const;

      /// \brief Reset the pose
      public: void Reset();

      /// \brief Rotate vector part of a pose about the origin
      /// \param[in] _rot rotation
      /// \return the rotated pose
      public: Pose RotatePositionAboutOrigin(const Quaternion &_rot) const;

      /// \brief Round all values to _precision decimal places
      /// \param[in] _precision
      public: void Round(int _precision);

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _pose pose to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const ignition::math::Pose &_pose)
              {
                _out << _pose.pos << " " << _pose.rot;
                return _out;
              }

      /// \brief Stream extraction operator
      /// \param[in] _in the input stream
      /// \param[in] _pose the pose
      /// \return the stream
      public: friend std::istream &operator>>(std::istream &_in,
                ignition::math::Pose &_pose)
            {
              // Skip white spaces
              _in.setf(std::ios_base::skipws);
              _in >> _pose.pos >> _pose.rot;
              return _in;
            }

      /// \brief The position
      public: Vector3d pos;

      /// \brief The rotation
      public: Quaternion rot;
    };
  }
}
#endif

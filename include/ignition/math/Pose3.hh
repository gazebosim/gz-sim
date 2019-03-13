/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_POSE_HH_
#define IGNITION_MATH_POSE_HH_

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Pose3 Pose3.hh ignition/math/Pose3.hh
    /// \brief Encapsulates a position and rotation in three space
    template<typename T>
    class Pose3
    {
      /// \brief math::Pose3<T>(0, 0, 0, 0, 0, 0)
      public: static const Pose3<T> Zero;

      /// \brief Default constructors
      public: Pose3() : p(0, 0, 0), q(1, 0, 0, 0)
      {
      }

      /// \brief Constructor
      /// \param[in] _pos A position
      /// \param[in] _rot A rotation
      public: Pose3(const Vector3<T> &_pos, const Quaternion<T> &_rot)
      : p(_pos), q(_rot)
      {
      }

      /// \brief Constructor
      /// \param[in] _x x position in meters.
      /// \param[in] _y y position in meters.
      /// \param[in] _z z position in meters.
      /// \param[in] _roll Roll (rotation about X-axis) in radians.
      /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
      /// \param[in] _yaw Yaw (rotation about z-axis) in radians.
      public: Pose3(T _x, T _y, T _z, T _roll, T _pitch, T _yaw)
      : p(_x, _y, _z), q(_roll, _pitch, _yaw)
      {
      }

      /// \brief Constructor
      /// \param[in] _x x position in meters.
      /// \param[in] _y y position in meters.
      /// \param[in] _z z position in meters.
      /// \param[in] _qw Quaternion w value.
      /// \param[in] _qx Quaternion x value.
      /// \param[in] _qy Quaternion y value.
      /// \param[in] _qz Quaternion z value.
      public: Pose3(T _x, T _y, T _z, T _qw, T _qx, T _qy, T _qz)
      : p(_x, _y, _z), q(_qw, _qx, _qy, _qz)
      {
      }

      /// \brief Copy constructor
      /// \param[in] _pose Pose3<T> to copy
      public: Pose3(const Pose3<T> &_pose)
      : p(_pose.p), q(_pose.q)
      {
      }

      /// \brief Destructor
      public: virtual ~Pose3()
      {
      }

      /// \brief Set the pose from a Vector3 and a Quaternion<T>
      /// \param[in] _pos The position.
      /// \param[in] _rot The rotation.
      public: void Set(const Vector3<T> &_pos, const Quaternion<T> &_rot)
      {
        this->p = _pos;
        this->q = _rot;
      }

      /// \brief Set the pose from  pos and rpy vectors
      /// \param[in] _pos The position.
      /// \param[in] _rpy The rotation expressed as Euler angles.
      public: void Set(const Vector3<T> &_pos, const Vector3<T> &_rpy)
      {
        this->p = _pos;
        this->q.Euler(_rpy);
      }

      /// \brief Set the pose from a six tuple.
      /// \param[in] _x x position in meters.
      /// \param[in] _y y position in meters.
      /// \param[in] _z z position in meters.
      /// \param[in] _roll Roll (rotation about X-axis) in radians.
      /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
      /// \param[in] _yaw Pitch (rotation about z-axis) in radians.
      public: void Set(T _x, T _y, T _z, T _roll, T _pitch, T _yaw)
      {
        this->p.Set(_x, _y, _z);
        this->q.Euler(math::Vector3<T>(_roll, _pitch, _yaw));
      }

      /// \brief See if a pose is finite (e.g., not nan)
      public: bool IsFinite() const
      {
        return this->p.IsFinite() && this->q.IsFinite();
      }

      /// \brief Fix any nan values
      public: inline void Correct()
      {
        this->p.Correct();
        this->q.Correct();
      }

      /// \brief Get the inverse of this pose
      /// \return the inverse pose
      public: Pose3<T> Inverse() const
      {
        Quaternion<T> inv = this->q.Inverse();
        return Pose3<T>(inv * (this->p*-1), inv);
      }

      /// \brief Addition operator
      /// A is the transform from O to P specified in frame O
      /// B is the transform from P to Q specified in frame P
      /// then, B + A is the transform from O to Q specified in frame O
      /// \param[in] _pose Pose3<T> to add to this pose
      /// \return The resulting pose
      public: Pose3<T> operator+(const Pose3<T> &_pose) const
      {
        Pose3<T> result;

        result.p = this->CoordPositionAdd(_pose);
        result.q = this->CoordRotationAdd(_pose.q);

        return result;
      }

      /// \brief Add-Equals operator
      /// \param[in] _pose Pose3<T> to add to this pose
      /// \return The resulting pose
      public: const Pose3<T> &operator+=(const Pose3<T> &_pose)
      {
        this->p = this->CoordPositionAdd(_pose);
        this->q = this->CoordRotationAdd(_pose.q);

        return *this;
      }

      /// \brief Negation operator
      /// A is the transform from O to P in frame O
      /// then -A is transform from P to O specified in frame P
      /// \return The resulting pose
      public: inline Pose3<T> operator-() const
      {
        return Pose3<T>() - *this;
      }

      /// \brief Subtraction operator
      /// A is the transform from O to P in frame O
      /// B is the transform from O to Q in frame O
      /// B - A is the transform from P to Q in frame P
      /// \param[in] _pose Pose3<T> to subtract from this one
      /// \return The resulting pose
      public: inline Pose3<T> operator-(const Pose3<T> &_pose) const
      {
        return Pose3<T>(this->CoordPositionSub(_pose),
          this->CoordRotationSub(_pose.q));
      }

      /// \brief Subtraction operator
      /// \param[in] _pose Pose3<T> to subtract from this one
      /// \return The resulting pose
      public: const Pose3<T> &operator-=(const Pose3<T> &_pose)
      {
        this->p = this->CoordPositionSub(_pose);
        this->q = this->CoordRotationSub(_pose.q);

        return *this;
      }

      /// \brief Equality operator
      /// \param[in] _pose Pose3<T> for comparison
      /// \return True if equal
      public: bool operator==(const Pose3<T> &_pose) const
      {
        return this->p == _pose.p && this->q == _pose.q;
      }

      /// \brief Inequality operator
      /// \param[in] _pose Pose3<T> for comparison
      /// \return True if not equal
      public: bool operator!=(const Pose3<T> &_pose) const
      {
        return this->p != _pose.p || this->q != _pose.q;
      }

      /// \brief Multiplication operator
      /// \param[in] _pose the other pose
      /// \return itself
      public: Pose3<T> operator*(const Pose3<T> &_pose) const
      {
        return Pose3<T>(_pose.CoordPositionAdd(*this),  this->q * _pose.q);
      }

      /// \brief Multiplication assignment operator. This pose will become
      /// equal to this * _pose.
      /// \param[in] _pose Pose3<T> to multiply to this pose
      /// \return The resulting pose
      public: const Pose3<T> &operator*=(const Pose3<T> &_pose)
      {
        *this = *this * _pose;
        return *this;
      }

      /// \brief Equal operator
      /// \param[in] _pose Pose3<T> to copy
      public: Pose3<T> &operator=(const Pose3<T> &_pose)
      {
        this->p = _pose.p;
        this->q = _pose.q;
        return *this;
      }

      /// \brief Add one point to a vector: result = this + pos
      /// \param[in] _pos Position to add to this pose
      /// \return the resulting position
      public: Vector3<T> CoordPositionAdd(const Vector3<T> &_pos) const
      {
        Quaternion<T> tmp(0.0, _pos.X(), _pos.Y(), _pos.Z());

        // result = pose.q + pose.q * this->p * pose.q!
        tmp = this->q * (tmp * this->q.Inverse());

        return Vector3<T>(this->p.X() + tmp.X(),
                          this->p.Y() + tmp.Y(),
                          this->p.Z() + tmp.Z());
      }

      /// \brief Add one point to another: result = this + pose
      /// \param[in] _pose The Pose3<T> to add
      /// \return The resulting position
      public: Vector3<T> CoordPositionAdd(const Pose3<T> &_pose) const
      {
        Quaternion<T> tmp(static_cast<T>(0),
            this->p.X(), this->p.Y(), this->p.Z());

        // result = _pose.q + _pose.q * this->p * _pose.q!
        tmp = _pose.q * (tmp * _pose.q.Inverse());

        return Vector3<T>(_pose.p.X() + tmp.X(),
                          _pose.p.Y() + tmp.Y(),
                          _pose.p.Z() + tmp.Z());
      }

      /// \brief Subtract one position from another: result = this - pose
      /// \param[in] _pose Pose3<T> to subtract
      /// \return The resulting position
      public: inline Vector3<T> CoordPositionSub(const Pose3<T> &_pose) const
      {
        Quaternion<T> tmp(0,
            this->p.X() - _pose.p.X(),
            this->p.Y() - _pose.p.Y(),
            this->p.Z() - _pose.p.Z());

        tmp = _pose.q.Inverse() * (tmp * _pose.q);
        return Vector3<T>(tmp.X(), tmp.Y(), tmp.Z());
      }

      /// \brief Add one rotation to another: result =  this->q + rot
      /// \param[in] _rot Rotation to add
      /// \return The resulting rotation
      public: Quaternion<T> CoordRotationAdd(const Quaternion<T> &_rot) const
      {
        return Quaternion<T>(_rot * this->q);
      }

      /// \brief Subtract one rotation from another: result = this->q - rot
      /// \param[in] _rot The rotation to subtract
      /// \return The resulting rotation
      public: inline Quaternion<T> CoordRotationSub(
                  const Quaternion<T> &_rot) const
      {
        Quaternion<T> result(_rot.Inverse() * this->q);
        result.Normalize();
        return result;
      }

      /// \brief Find the inverse of a pose; i.e., if b = this + a, given b and
      /// this, find a
      /// \param[in] _b the other pose
      public: Pose3<T> CoordPoseSolve(const Pose3<T> &_b) const
      {
        Quaternion<T> qt;
        Pose3<T> a;

        a.q = this->q.Inverse() * _b.q;
        qt = a.q * Quaternion<T>(0, this->p.X(), this->p.Y(), this->p.Z());
        qt = qt * a.q.Inverse();
        a.p = _b.p - Vector3<T>(qt.X(), qt.Y(), qt.Z());

        return a;
      }

      /// \brief Reset the pose
      public: void Reset()
      {
        // set the position to zero
        this->p.Set();
        this->q = Quaterniond::Identity;
      }

      /// \brief Rotate vector part of a pose about the origin
      /// \param[in] _rot rotation
      /// \return the rotated pose
      public: Pose3<T> RotatePositionAboutOrigin(const Quaternion<T> &_q) const
      {
        Pose3<T> a = *this;
        a.p.X((1.0 - 2.0*_q.Y()*_q.Y() - 2.0*_q.Z()*_q.Z()) * this->p.X()
                +(2.0*(_q.X()*_q.Y()+_q.W()*_q.Z())) * this->p.Y()
                +(2.0*(_q.X()*_q.Z()-_q.W()*_q.Y())) * this->p.Z());
        a.p.Y((2.0*(_q.X()*_q.Y()-_q.W()*_q.Z())) * this->p.X()
                +(1.0 - 2.0*_q.X()*_q.X() - 2.0*_q.Z()*_q.Z()) * this->p.Y()
                +(2.0*(_q.Y()*_q.Z()+_q.W()*_q.X())) * this->p.Z());
        a.p.Z((2.0*(_q.X()*_q.Z()+_q.W()*_q.Y())) * this->p.X()
                +(2.0*(_q.Y()*_q.Z()-_q.W()*_q.X())) * this->p.Y()
                +(1.0 - 2.0*_q.X()*_q.X() - 2.0*_q.Y()*_q.Y()) * this->p.Z());
        return a;
      }

      /// \brief Round all values to _precision decimal places
      /// \param[in] _precision
      public: void Round(int _precision)
      {
        this->q.Round(_precision);
        this->p.Round(_precision);
      }

      /// \brief Get the position.
      /// \return Origin of the pose.
      public: inline const Vector3<T> &Pos() const
      {
        return this->p;
      }

      /// \brief Get a mutable reference to the position.
      /// \return Origin of the pose.
      public: inline Vector3<T> &Pos()
      {
        return this->p;
      }

      /// \brief Get the rotation.
      /// \return Quaternion representation of the rotation.
      public: inline const Quaternion<T> &Rot() const
      {
        return this->q;
      }

      /// \brief Get a mutuable reference to the rotation.
      /// \return Quaternion representation of the rotation.
      public: inline Quaternion<T> &Rot()
      {
        return this->q;
      }

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _pose pose to output
      /// \return the stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const ignition::math::Pose3<T> &_pose)
      {
        _out << _pose.Pos() << " " << _pose.Rot();
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param[in] _in the input stream
      /// \param[in] _pose the pose
      /// \return the stream
      public: friend std::istream &operator>>(
                  std::istream &_in, ignition::math::Pose3<T> &_pose)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        Vector3<T> pos;
        Quaternion<T> rot;
        _in >> pos >> rot;
        _pose.Set(pos, rot);
        return _in;
      }

      /// \brief The position
      private: Vector3<T> p;

      /// \brief The rotation
      private: Quaternion<T> q;
    };
    template<typename T> const Pose3<T> Pose3<T>::Zero(0, 0, 0, 0, 0, 0);

    typedef Pose3<int> Pose3i;
    typedef Pose3<double> Pose3d;
    typedef Pose3<float> Pose3f;
    }
  }
}
#endif

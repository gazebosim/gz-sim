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
#ifndef GZ_MATH_POSE_HH_
#define GZ_MATH_POSE_HH_

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/config.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \class Pose3 Pose3.hh gz/math/Pose3.hh
    /// \brief The Pose3 class represents a 3D position and rotation. The
    /// position  component is a Vector3, and the rotation is a Quaternion.
    ///
    /// The following two type definitions are provided:
    ///
    /// * \ref Pose3f
    /// * \ref Pose3d
    /// ## Examples
    ///
    /// * C++
    ///
    /// \snippet examples/pose3_example.cc complete
    ///
    /// * Ruby
    ///
    /// \code{.rb}
    /// # $ export RUBYLIB=/usr/lib/ruby:$RUBYLIB
    /// #
    /// require 'gz/math'
    ///
    /// # Construct a default Pose3d.
    /// p = Gz::Math::Pose3d.new
    /// printf("A default Pose3d has the following values\n" +
    ///        "%f %f %f %f %f %f\n", p.Pos().X(), p.Pos().Y(), p.Pos().Z(),
    ///        p.Rot().Euler().X(), p.Rot().Euler().Y(), p.Rot().Euler().Z())
    ///
    /// # Construct a pose at position 1, 2, 3 with a yaw of PI radians.
    /// p1 = Gz::Math::Pose3d.new(1, 2, 3, 0, 0, Math::PI)
    /// printf("A pose3d(1, 2, 3, 0, 0, GZ_PI) has the following values\n" +
    ///        "%f %f %f %f %f %f\n", p1.Pos().X(), p1.Pos().Y(), p1.Pos().Z(),
    ///        p1.Rot().Euler().X(), p1.Rot().Euler().Y(), p1.Rot().Euler().Z())
    ///
    /// # Set the position of a pose to 10, 20, 30
    /// p.Pos().Set(10, 20, 30)
    ///
    /// p3 = p * p1
    /// printf("Result of combining two poses is\n"+
    ///         "%f %f %f %f %f %f\n", p3.Pos().X(), p3.Pos().Y(), p3.Pos().Z(),
    ///        p3.Rot().Euler().X(), p3.Rot().Euler().Y(), p3.Rot().Euler().Z())
    /// \endcode
    template<typename T>
    class Pose3
    {
      /// \brief A Pose3 initialized to zero.
      /// This is equivalent to math::Pose3<T>(0, 0, 0, 0, 0, 0).
      public: static const Pose3<T> &Zero;

      /// \brief Default constructor. This initializes the position
      /// component to zero and the quaternion to identity.
      public: Pose3() = default;

      /// \brief Create a Pose3 based on a position and rotation.
      /// \param[in] _pos The position
      /// \param[in] _rot The rotation
      public: Pose3(const Vector3<T> &_pos, const Quaternion<T> &_rot)
      : p(_pos), q(_rot)
      {
      }

      /// \brief Create a Pose3 using a 6-tuple consisting of
      ///  x, y, z, roll, pitch, and yaw.
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

      /// \brief Create a Pose3 using a 7-tuple consisting of
      /// x, y, z, qw, qx, qy, qz. The first three values are the position
      /// and the last four the rotation represented as a quaternion.
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

      /// \brief Copy constructor.
      /// \param[in] _pose Pose3<T> to copy
      public: Pose3(const Pose3<T> &_pose) = default;

      /// \brief Destructor.
      public: ~Pose3() = default;

      /// \brief Set the pose from a Vector3<T> and a Quaternion<T>
      /// \param[in] _pos The position.
      /// \param[in] _rot The rotation.
      public: void Set(const Vector3<T> &_pos, const Quaternion<T> &_rot)
      {
        this->p = _pos;
        this->q = _rot;
      }

      /// \brief Set the pose from a position and Euler angles.
      /// \param[in] _pos The position.
      /// \param[in] _rpy The rotation expressed as Euler angles.
      public: void Set(const Vector3<T> &_pos, const Vector3<T> &_rpy)
      {
        this->p = _pos;
        this->q.SetFromEuler(_rpy);
      }

      /// \brief Set the pose from a six tuple consisting of
      ///  x, y, z, roll, pitch, and yaw.
      /// \param[in] _x x position in meters.
      /// \param[in] _y y position in meters.
      /// \param[in] _z z position in meters.
      /// \param[in] _roll Roll (rotation about X-axis) in radians.
      /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
      /// \param[in] _yaw Pitch (rotation about z-axis) in radians.
      public: void Set(T _x, T _y, T _z, T _roll, T _pitch, T _yaw)
      {
        this->p.Set(_x, _y, _z);
        this->q.SetFromEuler(math::Vector3<T>(_roll, _pitch, _yaw));
      }

      /// \brief See if a pose is finite (e.g., not nan)
      /// \return True if this pose is finite.
      public: bool IsFinite() const
      {
        return this->p.IsFinite() && this->q.IsFinite();
      }

      /// \brief Fix any nan values.
      public: inline void Correct()
      {
        this->p.Correct();
        this->q.Correct();
      }

      /// \brief Get the inverse of this pose.
      /// \return The inverse pose.
      public: Pose3<T> Inverse() const
      {
        Quaternion<T> inv = this->q.Inverse();
        return Pose3<T>(inv * (this->p*-1), inv);
      }

      /// \brief Addition operator.
      /// A is the transform from O to P specified in frame O
      /// B is the transform from P to Q specified in frame P
      /// then, B + A is the transform from O to Q specified in frame O
      /// \param[in] _pose Pose3<T> to add to this pose.
      /// \return The resulting pose.
      public: GZ_DEPRECATED(7) Pose3<T> operator+(const Pose3<T> &_pose) const
      {
        Pose3<T> result;

        result.p = this->CoordPositionAdd(_pose);
        result.q = this->CoordRotationAdd(_pose.q);

        return result;
      }

      /// \brief Addition assignment operator.
      /// \param[in] _pose Pose3<T> to add to this pose.
      /// \sa operator+(const Pose3<T> &_pose) const.
      /// \return The resulting pose.
      public: GZ_DEPRECATED(7) const Pose3<T> &
              operator+=(const Pose3<T> &_pose)
      {
        this->p = this->CoordPositionAdd(_pose);
        this->q = this->CoordRotationAdd(_pose.q);

        return *this;
      }

      /// \brief Negation operator.
      /// A is the transform from O to P in frame O
      /// then -A is transform from P to O specified in frame P
      /// \return The resulting pose.
      public: GZ_DEPRECATED(7) Pose3<T> operator-() const
      {
        return this->Inverse();
      }

      /// \brief Subtraction operator.
      /// A is the transform from O to P in frame O
      /// B is the transform from O to Q in frame O
      /// B - A is the transform from P to Q in frame P
      /// \param[in] _pose Pose3<T> to subtract from this one.
      /// \return The resulting pose.
      public: GZ_DEPRECATED(7) Pose3<T> operator-(const Pose3<T> &_pose) const
      {
        return Pose3<T>(this->CoordPositionSub(_pose),
          this->CoordRotationSub(_pose.q));
      }

      /// \brief Subtraction assignment operator.
      /// \param[in] _pose Pose3<T> to subtract from this one
      /// \sa operator-(const Pose3<T> &_pose) const.
      /// \return The resulting pose
      public: GZ_DEPRECATED(7) const Pose3<T> &
              operator-=(const Pose3<T> &_pose)
      {
        this->p = this->CoordPositionSub(_pose);
        this->q = this->CoordRotationSub(_pose.q);

        return *this;
      }

      /// \brief Equality operator.
      /// \param[in] _pose Pose3<T> for comparison.
      /// \return True if this pose is equal to the given pose.
      public: bool operator==(const Pose3<T> &_pose) const
      {
        return this->p == _pose.p && this->q == _pose.q;
      }

      /// \brief Inequality operator.
      /// \param[in] _pose Pose3<T> for comparison.
      /// \return True if this pose is not equal to the  given pose.
      public: bool operator!=(const Pose3<T> &_pose) const
      {
        return this->p != _pose.p || this->q != _pose.q;
      }

      /// \brief Multiplication operator.
      /// Given X_OP (frame P relative to O) and X_PQ (frame Q relative to P)
      /// then X_OQ = X_OP * X_PQ (frame Q relative to O).
      /// \param[in] _pose The pose to multiply by.
      /// \return The resulting pose.
      public: Pose3<T> operator*(const Pose3<T> &_pose) const
      {
        return Pose3<T>(_pose.CoordPositionAdd(*this),  this->q * _pose.q);
      }

      /// \brief Multiplication assignment operator. This pose will become
      /// equal to this * _pose.
      /// \param[in] _pose Pose3<T> to multiply to this pose
      /// \sa operator*(const Pose3<T> &_pose) const
      /// \return The resulting pose
      public: const Pose3<T> &operator*=(const Pose3<T> &_pose)
      {
        *this = *this * _pose;
        return *this;
      }

      /// \brief Assignment operator
      /// \param[in] _pose Pose3<T> to copy
      public: Pose3<T> &operator=(const Pose3<T> &_pose) = default;

      /// \brief Add one point to a vector: result = this + pos.
      /// \param[in] _pos Position to add to this pose
      /// \return The resulting position.
      public: Vector3<T> CoordPositionAdd(const Vector3<T> &_pos) const
      {
        Quaternion<T> tmp(0.0, _pos.X(), _pos.Y(), _pos.Z());

        // result = pose.q + pose.q * this->p * pose.q!
        tmp = this->q * (tmp * this->q.Inverse());

        return Vector3<T>(this->p.X() + tmp.X(),
                          this->p.Y() + tmp.Y(),
                          this->p.Z() + tmp.Z());
      }

      /// \brief Add one pose to another: result = this + pose.
      /// \param[in] _pose The Pose3<T> to add.
      /// \return The resulting position.
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

      /// \brief Add one rotation to another: result =  this->q + rot.
      /// \param[in] _rot Rotation to add.
      /// \return The resulting rotation.
      public: Quaternion<T> CoordRotationAdd(const Quaternion<T> &_rot) const
      {
        return Quaternion<T>(_rot * this->q);
      }

      /// \brief Subtract one rotation from another: result = this->q - rot.
      /// \param[in] _rot The rotation to subtract.
      /// \return The resulting rotation.
      public: inline Quaternion<T> CoordRotationSub(
                  const Quaternion<T> &_rot) const
      {
        Quaternion<T> result(_rot.Inverse() * this->q);
        result.Normalize();
        return result;
      }

      /// \brief Find the inverse of a pose; i.e., if b = this + a, given b and
      /// this, find a.
      /// \param[in] _b the other pose.
      // \return The inverse pose.
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

      /// \brief Reset the pose. This sets the position to zero and the
      /// rotation to identify.
      public: void Reset()
      {
        // set the position to zero
        this->p.Set();
        this->q = Quaternion<T>::Identity;
      }

      /// \brief Rotate the vector part of a pose about the origin.
      /// \param[in] _q rotation.
      /// \return The rotated pose.
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

      /// \brief Round all values to _precision decimal places.
      /// \param[in] _precision Number of decimal places.
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

      /// \brief Get the X value of the position.
      /// \return Value X of the origin of the pose.
      /// \note The return is made by value since
      /// Vector3<T>.X() is already a reference.
      public: inline const T X() const
      {
        return this->p.X();
      }

      /// \brief Set X value of the position.
      public: inline void SetX(T x)
      {
       this->p.X() = x;
      }

      /// \brief Get the Y value of the position.
      /// \return Value Y of the origin of the pose.
      /// \note The return is made by value since
      /// Vector3<T>.Y() is already a reference.
      public: inline const T Y() const
      {
        return this->p.Y();
      }

      /// \brief Set the Y value of the position.
      public: inline void SetY(T y)
      {
        this->p.Y() = y;
      }

      /// \brief Get the Z value of the position.
      /// \return Value Z of the origin of the pose.
      /// \note The return is made by value since
      /// Vector3<T>.Z() is already a reference.
      public: inline const T Z() const
      {
        return this->p.Z();
      }

      /// \brief Set the Z value of the position.
      public: inline void SetZ(T z)
      {
        this->p.Z() = z;
      }

      /// \brief Get the rotation.
      /// \return Quaternion representation of the rotation.
      public: inline const Quaternion<T> &Rot() const
      {
        return this->q;
      }

      /// \brief Get a mutable reference to the rotation.
      /// \return Quaternion representation of the rotation.
      public: inline Quaternion<T> &Rot()
      {
        return this->q;
      }

      /// \brief Get the Roll value of the rotation.
      /// \return Roll value of the orientation.
      /// \note The return is made by value since
      ///  Quaternion<T>.Roll() is already a reference.
      public: inline const T Roll() const
      {
        return this->q.Roll();
      }

      /// \brief Get the Pitch value of the rotation.
      /// \return Pitch value of the orientation.
      /// \note The return is made by value since
      ///  Quaternion<T>.Pitch() is already a reference.
      public: inline const T Pitch() const
      {
        return this->q.Pitch();
      }

      /// \brief Get the Yaw value of the rotation.
      /// \return Yaw value of the orientation.
      /// \note The return is made by value since
      ///  Quaternion<T>.Yaw() is already a reference.
      public: inline const T Yaw() const
      {
        return this->q.Yaw();
      }

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _pose pose to output
      /// \return the stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const gz::math::Pose3<T> &_pose)
      {
        _out << _pose.Pos() << " " << _pose.Rot();
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param[in] _in the input stream
      /// \param[in] _pose the pose
      /// \return the stream
      public: friend std::istream &operator>>(
                  std::istream &_in, gz::math::Pose3<T> &_pose)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        Vector3<T> pos;
        Quaternion<T> rot;
        _in >> pos >> rot;
        _pose.Set(pos, rot);
        return _in;
      }

      /// \brief Equality test with tolerance.
      /// \param[in] _p The pose to compare this against. Both the position
      /// Vector3 and rotation Quaternion are compared.
      /// \param[in] _tol Equality tolerance.
      /// \return True if the position and orientation of the poses are equal
      /// within the tolerence specified by _tol.
      public: bool Equal(const Pose3 &_p, const T &_tol) const
      {
        return this->p.Equal(_p.p, _tol) && this->q.Equal(_p.q, _tol);
      }

      /// \brief The position
      private: Vector3<T> p;

      /// \brief The rotation
      private: Quaternion<T> q;
    };

    namespace detail {

      template<typename T> constexpr Pose3<T> gPose3Zero{};

    }  // namespace detail

    template<typename T> const Pose3<T> &Pose3<T>::Zero = detail::gPose3Zero<T>;

    /// typedef Pose3<double> as Pose3d.
    typedef Pose3<double> Pose3d;

    /// typedef Pose3<float> as Pose3f.
    typedef Pose3<float> Pose3f;
    }
  }
}
#endif

/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

%module quaternion
%{
  #include <gz/math/config.hh>
  #include <gz/math/Pose3.hh>
  #include <gz/math/Quaternion.hh>
  #include <gz/math/Vector3.hh>
%}

%include "std_string.i"
%include "Quaternion.i"


namespace gz
{
  namespace math
  {
    template<typename T>
    class Pose3
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      %rename("%(uppercase)s", %$isstatic, %$isvariable) "";

      public: static const Pose3<T> Zero;

      public: Pose3() = default;
      public: Pose3(const Vector3<T> &_pos, const Quaternion<T> &_rot)
      : p(_pos), q(_rot);
      public: Pose3(T _x, T _y, T _z, T _roll, T _pitch, T _yaw)
      : p(_x, _y, _z), q(_roll, _pitch, _yaw);
      public: Pose3(T _x, T _y, T _z, T _qw, T _qx, T _qy, T _qz)
      : p(_x, _y, _z), q(_qw, _qx, _qy, _qz);
      public: Pose3(const Pose3<T> &_pose) = default;
      public: ~Pose3() = default;
      public: void Set(const Vector3<T> &_pos, const Quaternion<T> &_rot);
      public: void Set(const Vector3<T> &_pos, const Vector3<T> &_rpy);
      public: void Set(T _x, T _y, T _z, T _roll, T _pitch, T _yaw);
      public: bool IsFinite() const;
      public: inline void Correct();
      public: Pose3<T> Inverse() const;
      public: Pose3<T> operator+(const Pose3<T> &_pose) const;
      public: const Pose3<T> &operator+=(const Pose3<T> &_pose);
      public: inline Pose3<T> operator-() const;
      public: inline Pose3<T> operator-(const Pose3<T> &_pose) const;
      public: const Pose3<T> &operator-=(const Pose3<T> &_pose);
      public: bool operator==(const Pose3<T> &_pose) const;
      public: bool operator!=(const Pose3<T> &_pose) const;
      public: Pose3<T> operator*(const Pose3<T> &_pose) const;
      public: const Pose3<T> &operator*=(const Pose3<T> &_pose);
      public: Vector3<T> CoordPositionAdd(const Vector3<T> &_pos) const;
      public: Vector3<T> CoordPositionAdd(const Pose3<T> &_pose) const;
      public: inline Vector3<T> CoordPositionSub(const Pose3<T> &_pose) const;
      public: Quaternion<T> CoordRotationAdd(const Quaternion<T> &_rot) const;
      public: inline Quaternion<T> CoordRotationSub(
                  const Quaternion<T> &_rot) const;
      public: Pose3<T> CoordPoseSolve(const Pose3<T> &_b) const;
      public: void Reset();
      public: Pose3<T> RotatePositionAboutOrigin(const Quaternion<T> &_q) const;
      public: void Round(int _precision);
      public: inline const Vector3<T> &Pos() const;
      public: inline Vector3<T> &Pos();
      public: inline const T X() const;
      public: inline void SetX(T x);
      public: inline const T Y() const;
      public: inline void SetY(T y);
      public: inline const T Z() const;
      public: inline void SetZ(T z);
      public: inline const Quaternion<T> &Rot() const;
      public: inline Quaternion<T> &Rot();
      public: inline const T Roll() const;
      public: inline const T Pitch() const;
      public: inline const T Yaw() const;
      public: bool Equal(const Pose3 &_p, const T &_tol) const;
    };

    %extend Pose3 {
        std::string __str__() const {
          std::ostringstream out;
          out << *$self;
          return out.str();
        }
    }

    %template(Pose3d) Pose3<double>;
    %template(Pose3f) Pose3<float>;
  }
}

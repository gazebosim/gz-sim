/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifdef SWIGRUBY
%begin %{
#define HAVE_ISFINITE 1
%}
#endif

%module pose3
%{
#include <gz/math/Pose3.hh>
%}

namespace gz
{
  namespace math
  {
    template<typename T>
    class Pose3
    {
      public: static const Pose3<T> Zero;
      public: Pose3();
      public: Pose3(const gz::math::Vector3<T> &_pos,
                    const gz::math::Quaternion<T> &_rot);
      public: Pose3(T _x, T _y, T _z, T _roll, T _pitch, T _yaw);
      public: Pose3(T _x, T _y, T _z, T _qw, T _qx, T _qy, T _qz);
      public: Pose3(const Pose3<T> &_pose);
      public: ~Pose3();
      public: void Set(const gz::math::Vector3<T> &_pos,
                       const gz::math::Quaternion<T> &_rot);
      public: void Set(const gz::math::Vector3<T> &_pos,
                  const gz::math::Vector3<T> &_rpy);
      public: void Set(T _x, T _y, T _z, T _roll, T _pitch, T _yaw);
      public: bool IsFinite() const;
      public: inline void Correct();
      public: Pose3<T> Inverse() const;
      public: Pose3<T> operator+(const Pose3<T> &_pose) const;
      public: inline Pose3<T> operator-() const;
      public: inline Pose3<T> operator-(const Pose3<T> &_pose) const;
      public: bool operator==(const Pose3<T> &_pose) const;
      public: Pose3<T> operator*(const Pose3<T> &_pose) const;
      public: gz::math::Vector3<T> CoordPositionAdd(
                  const gz::math::Vector3<T> &_pos) const;
      public: gz::math::Vector3<T> CoordPositionAdd(
                  const Pose3<T> &_pose) const;
      public: inline gz::math::Vector3<T> CoordPositionSub(
                  const Pose3<T> &_pose) const;
      public: gz::math::Quaternion<T> CoordRotationAdd(
                  const gz::math::Quaternion<T> &_rot) const;
      public: inline gz::math::Quaternion<T> CoordRotationSub(
                  const gz::math::Quaternion<T> &_rot) const;
      public: Pose3<T> CoordPoseSolve(const Pose3<T> &_b) const;
      public: void Reset();
      public: Pose3<T> RotatePositionAboutOrigin(
                  const gz::math::Quaternion<T> &_q) const;
      public: void Round(int _precision);
      public: inline const gz::math::Vector3<T> &Pos() const;
      public: inline gz::math::Vector3<T> &Pos();
      public: inline const gz::math::Quaternion<T> &Rot() const;
      public: inline gz::math::Quaternion<T> &Rot();
    };

    %template(Pose3d) Pose3<double>;

    %template(Pose3f) Pose3<float>;
  }
}

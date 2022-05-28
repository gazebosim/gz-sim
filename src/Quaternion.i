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

%module quaternion
%{
#include <gz/math/Quaternion.hh>
%}

namespace gz
{
  namespace math
  {
    template<typename T>
    class Quaternion
    {
      public: static const Quaternion Identity;
      public: static const Quaternion Zero;

      public: Quaternion();
      public: Quaternion(const T &_w, const T &_x, const T &_y, const T &_z);
      public: Quaternion(const T &_roll, const T &_pitch, const T &_yaw);
      public: Quaternion(const gz::math::Vector3<T> &_axis,
                         const T &_angle);
      public: explicit Quaternion(const gz::math::Vector3<T> &_rpy);
      public: explicit Quaternion(const gz::math::Matrix3<T> &_mat);
      public: Quaternion(const Quaternion<T> &_qt);
      public: ~Quaternion();
      public: void Invert();
      public: inline Quaternion<T> Inverse() const;
      public: Quaternion<T> Log() const;
      public: Quaternion<T> Exp() const;
      public: void Normalize();
      public: void SetFromAxisAngle(T _ax, T _ay, T _az, T _aa);
      public: void SetFromAxisAngle(const gz::math::Vector3<T> &_axis,
                  T _a);
      public: void Set(T _w, T _x, T _y, T _z);
      public: void SetFromEuler(const gz::math::Vector3<T> &_vec);
      public: void SetFromEuler(T _roll, T _pitch, T _yaw);
      public: gz::math::Vector3<T> Euler() const;
      public: static Quaternion<T> EulerToQuaternion(
                  const gz::math::Vector3<T> &_vec);
      public: static Quaternion<T> EulerToQuaternion(T _x, T _y, T _z);
      public: T Roll() const;
      public: T Pitch() const;
      public: T Yaw() const;
      public: void AxisAngle(gz::math::Vector3<T> &_axis,
                  T &_angle) const;
      void SetFromMatrix(const gz::math::Matrix3<T> &_mat);
      public: void SetFrom2Axes(const gz::math::Vector3<T> &_v1,
                  const gz::math::Vector3<T> &_v2);
      public: void Scale(T _scale);
      public: Quaternion<T> operator+(const Quaternion<T> &_qt) const;
      public: Quaternion<T> operator-(const Quaternion<T> &_qt) const;
      public: inline Quaternion<T> operator*(const Quaternion<T> &_q) const;
      public: Quaternion<T> operator*(const T &_f) const;
      public: gz::math::Vector3<T> operator*(
                  const gz::math::Vector3<T> &_v) const;
      public: bool operator==(const Quaternion<T> &_qt) const;
      public: bool Equal(const Quaternion<T> &_qt, const T &_tol) const;
      public: Quaternion<T> operator-() const;
      public: inline gz::math::Vector3<T> RotateVector(
                  const gz::math::Vector3<T> &_vec) const;
      public: gz::math::Vector3<T> RotateVectorReverse(
                  const gz::math::Vector3<T> &_vec) const;
      public: bool IsFinite() const;
      public: inline void Correct();
      public: gz::math::Vector3<T> XAxis() const;
      public: gz::math::Vector3<T> YAxis() const;
      public: gz::math::Vector3<T> ZAxis() const;
      public: void Round(int _precision);
      public: T Dot(const Quaternion<T> &_q) const;
      public: static Quaternion<T> Squad(T _fT,
                  const Quaternion<T> &_rkP, const Quaternion<T> &_rkA,
                  const Quaternion<T> &_rkB, const Quaternion<T> &_rkQ,
                  bool _shortestPath = false);
      public: static Quaternion<T> Slerp(T _fT,
                  const Quaternion<T> &_rkP, const Quaternion<T> &_rkQ,
                  bool _shortestPath = false);
      public: Quaternion<T> Integrate(
                  const gz::math::Vector3<T> &_angularVelocity,
                  const T _deltaT) const;
      public: inline T W() const;
      public: inline T X() const;
      public: inline T Y() const;
      public: inline T Z() const;
      public: inline void SetX(T _v);
      public: inline void SetY(T _v);
      public: inline void SetZ(T _v);
      public: inline void SetW(T _v);
    };

    %template(Quaterniond) Quaternion<double>;
    %template(Quaternionf) Quaternion<float>;
  }
}

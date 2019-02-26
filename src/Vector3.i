/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

%module vector3
%{
#include <ignition/math/Vector3.hh>
%}

namespace ignition
{
  namespace math
  {
    template<typename T>
    class Vector3
    {
      public: static const Vector3 Zero;
      public: static const Vector3 One;
      public: static const Vector3 UnitX;
      public: static const Vector3 UnitY;
      public: static const Vector3 UnitZ;
      public: Vector3();
      public: Vector3(const T &_x, const T &_y, const T &_z);
      public: Vector3(const Vector3<T> &_v);
      public: virtual ~Vector3();
      public: T Sum() const;
      public: T Distance(const Vector3<T> &_pt) const;
      public: T Distance(T _x, T _y, T _z) const;
      public: T Length() const;
      public: T SquaredLength() const;
      public: Vector3 Normalize();
      public: Vector3 Normalized() const;
      public: Vector3 Round();
      public: Vector3 Rounded() const;
      public: inline void Set(T _x = 0, T _y = 0, T _z = 0);
      public: Vector3 Cross(const Vector3<T> &_v) const;
      public: T Dot(const Vector3<T> &_v) const;
      public: T AbsDot(const Vector3<T> &_v) const;
      public: Vector3 Abs() const;
      public: Vector3 Perpendicular() const;
      public: static Vector3 Normal(const Vector3<T> &_v1,
                  const Vector3<T> &_v2, const Vector3<T> &_v3);
      public: T DistToLine(const Vector3<T> &_pt1, const Vector3 &_pt2);
      public: void Max(const Vector3<T> &_v);
      public: void Min(const Vector3<T> &_v);
      public: T Max() const;
      public: T Min() const;
      public: Vector3 operator+(const Vector3<T> &_v) const;
      public: inline Vector3<T> operator+(const T _s) const;
      public: inline Vector3 operator-() const;
      public: inline Vector3<T> operator-(const Vector3<T> &_pt) const;
      public: inline Vector3<T> operator-(const T _s) const;
      public: const Vector3<T> operator/(const Vector3<T> &_pt) const;
      public: const Vector3<T> operator/(T _v) const;
      public: Vector3<T> operator*(const Vector3<T> &_p) const;
      public: inline Vector3<T> operator*(T _s) const;
      public: bool Equal(const Vector3 &_v, const T &_tol) const;
      public: bool operator==(const Vector3<T> &_v) const;
      public: bool IsFinite() const;
      public: inline void Correct();
      public: void Round(int _precision);
      public: bool Equal(const Vector3<T> &_v) const;
      public: inline T X() const;
      public: inline T Y() const;
      public: inline T Z() const;
      public: inline void X(const T &_v);
      public: inline void Y(const T &_v);
      public: inline void Z(const T &_v);
      public: bool operator<(const Vector3<T> &_pt) const;
    };

    %template(Vector3i) Vector3<int>;
    %template(Vector3d) Vector3<double>;
    %template(Vector3f) Vector3<float>;
  }
}

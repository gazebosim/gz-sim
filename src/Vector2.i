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

%module vector2
%{
#include <ignition/math/Vector2.hh>
%}

namespace ignition
{
  namespace math
  {
    template<typename T>
    class Vector2
    {
      public: static const Vector2 Zero;
      public: static const Vector2 One;

      public: Vector2();
      public: Vector2(const T &_x, const T &_y);
      public: Vector2(const Vector2<T> &_v);
      public: virtual ~Vector2();
      public: double Distance(const Vector2 &_pt) const;
      public: T Length() const;
      public: T SquaredLength() const;
      public: void Normalize();
      public: void Set(T _x, T _y);
      public: T Dot(const Vector2<T> &_v) const;
      public: Vector2 operator+(const Vector2 &_v) const;
      public: inline Vector2<T> operator+(const T _s) const;
      public: inline Vector2 operator-() const;
      public: Vector2 operator-(const Vector2 &_v) const;
      public: inline Vector2<T> operator-(const T _s) const;
      public: const Vector2 operator/(const Vector2 &_v) const;
      public: const Vector2 operator/(T _v) const;
      public: const Vector2 operator*(const Vector2 &_v) const;
      public: const Vector2 operator*(T _v) const;
      public: bool operator==(const Vector2 &_v) const;
      public: bool IsFinite() const;
      public: inline T X() const;
      public: inline T Y() const;
      public: inline void X(const T &_v);
      public: inline void Y(const T &_v);
      public: bool Equal(const Vector2 &_v, const T &_tol) const;
    };

    %template(Vector2i) Vector2<int>;
    %template(Vector2d) Vector2<double>;
    %template(Vector2f) Vector2<float>;
  }
}

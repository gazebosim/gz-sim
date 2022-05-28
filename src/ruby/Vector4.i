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

%module vector4
%{
#include <gz/math/Vector4.hh>
%}

namespace gz
{
  namespace math
  {
    template<typename T>
    class Vector4
    {
      // Use %extend to override getters for static member variables so that
      // copies of the variables are returned instead of references to the variables.
      public: %extend {
        static Vector4 Zero()
        {
          return gz::math::Vector4<T>::Zero;
        }
        static Vector4 One()
        {
          return gz::math::Vector4<T>::One;
        }
        static Vector4 NaN()
        {
          return gz::math::Vector4<T>::NaN;
        }
      }
      public: Vector4();
      public: Vector4(const T &_x, const T &_y, const T &_z, const T &_w);
      public: Vector4(const Vector4<T> &_v);
      public: virtual ~Vector4();
      public: T Distance(const Vector4<T> &_pt) const;
      public: T Length() const;
      public: T SquaredLength() const;
      public: void Normalize();
      public: inline void Set(T _x = 0, T _y = 0, T _z = 0, T _w = 0);
      public: Vector4 operator+(const Vector4<T> &_v) const;
      public: inline Vector4<T> operator+(const T _s) const;
      public: inline Vector4 operator-() const;
      public: inline Vector4<T> operator-(const Vector4<T> &_pt) const;
      public: inline Vector4<T> operator-(const T _s) const;
      public: const Vector4<T> operator/(const Vector4<T> &_pt) const;
      public: const Vector4<T> operator/(T _v) const;
      public: Vector4<T> operator*(const Vector4<T> &_p) const;
      public: inline Vector4<T> operator*(T _s) const;
      public: bool operator==(const Vector4<T> &_v) const;
      public: bool Equal(const Vector4 &_v, const T &_tol) const;
      public: bool IsFinite() const;
      public: inline void Correct();
      public: inline T X() const;
      public: inline T Y() const;
      public: inline T Z() const;
      public: inline T W() const;
      public: inline void X(const T &_v);
      public: inline void Y(const T &_v);
      public: inline void Z(const T &_v);
      public: inline void W(const T &_v);
    };

    %template(Vector4i) Vector4<int>;
    %template(Vector4d) Vector4<double>;
    %template(Vector4f) Vector4<float>;
  }
}

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

%module matrix3
%{
#include <ignition/math/Matrix3.hh>
%}

namespace ignition
{
  namespace math
  {
    template<typename T>
    class Matrix3
    {
      public: static const Matrix3<T> Identity;
      public: static const Matrix3<T> Zero;
      public: Matrix3();
      public: Matrix3(const Matrix3<T> &_m);
      public: Matrix3(T _v00, T _v01, T _v02,
                      T _v10, T _v11, T _v12,
                      T _v20, T _v21, T _v22);
      public: explicit Matrix3(const ignition::math::Quaternion<T> &_q);
      public: ~Matrix3();
      public: void Set(size_t _row, size_t _col, T _v);
      public: void Set(T _v00, T _v01, T _v02,
                       T _v10, T _v11, T _v12,
                       T _v20, T _v21, T _v22);
      public: void SetAxes(const ignition::math::Vector3<T> &_xAxis,
                           const ignition::math::Vector3<T> &_yAxis,
                           const ignition::math::Vector3<T> &_zAxis);
      public: void SetFromAxisAngle(const ignition::math::Vector3<T> &_axis,
                  T _angle);
      public: void SetFrom2Axes(const ignition::math::Vector3<T> &_v1,
                  const ignition::math::Vector3<T> &_v2);
      public: void SetCol(unsigned int _c,
                  const ignition::math::Vector3<T> &_v);
      public: Matrix3<T> operator-(const Matrix3<T> &_m) const;
      public: Matrix3<T> operator+(const Matrix3<T> &_m) const;
      public: Matrix3<T> operator*(const T &_s) const;
      public: Matrix3<T> operator*(const Matrix3<T> &_m) const;
      public: ignition::math::Vector3<T> operator*(
                  const ignition::math::Vector3<T> &_vec) const;
      public: bool Equal(const Matrix3 &_m, const T &_tol) const;
      public: bool operator==(const Matrix3<T> &_m) const;
      public: inline T operator()(size_t _row, size_t _col) const;
      public: T Determinant() const;
      public: Matrix3<T> Inverse() const;
      public: void Transpose();
      public: Matrix3<T> Transposed() const;
    };

    %template(Matrix3i) Matrix3<int>;
    %template(Matrix3d) Matrix3<double>;
    %template(Matrix3f) Matrix3<float>;
  }
}

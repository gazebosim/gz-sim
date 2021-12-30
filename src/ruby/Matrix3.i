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

%module matrix3
%{
#include <ignition/math/config.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
%}

%include "std_string.i"
%include Quaternion.i

namespace ignition
{
  namespace math
  {
    template<typename T>
    class Matrix3
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      %rename("%(uppercase)s", %$isstatic, %$isvariable) "";

      public: static const Matrix3<T> Identity;
      public: static const Matrix3<T> Zero;
      public: Matrix3();
      public: Matrix3(const Matrix3<T> &_m);
      public: Matrix3(T _v00, T _v01, T _v02,
                      T _v10, T _v11, T _v12,
                      T _v20, T _v21, T _v22);
      public: explicit Matrix3(const Quaternion<T> &_q);
      public: virtual ~Matrix3() {}
      public: void Set(T _v00, T _v01, T _v02,
                       T _v10, T _v11, T _v12,
                       T _v20, T _v21, T _v22);
      public: void Axes(const Vector3<T> &_xAxis,
                        const Vector3<T> &_yAxis,
                        const Vector3<T> &_zAxis);
      public: void Axis(const Vector3<T> &_axis, T _angle);
      %rename(from_2_axes) From2Axes;
      public: void From2Axes(const Vector3<T> &_v1, const Vector3<T> &_v2);
      public: void Col(unsigned int _c, const Vector3<T> &_v);
      public: Matrix3<T> operator-(const Matrix3<T> &_m) const;
      public: Matrix3<T> operator+(const Matrix3<T> &_m) const;
      public: Matrix3<T> operator*(const T &_s) const;
      public: Matrix3<T> operator*(const Matrix3<T> &_m) const;
      public: Vector3<T> operator*(const Vector3<T> &_vec) const;
      public: bool Equal(const Matrix3 &_m, const T &_tol) const;
      public: bool operator==(const Matrix3<T> &_m) const;
      public: bool operator!=(const Matrix3<T> &_m) const;;
      public: T Determinant() const;
      public: Matrix3<T> Inverse() const;
      public: void Transpose();
      public: Matrix3<T> Transposed() const;
    };

    %extend Matrix3{
        T __call__(size_t row, size_t col) const {
          return (*$self)(row, col);
        }
    }

    %extend Matrix3 {
        std::string __str__() const {
          std::ostringstream out;
          out << *$self;
          return out.str();
        }
    }

    %extend Matrix3 {
      ignition::math::Quaternion<T> to_quaternion() {
        return ignition::math::Quaternion<T>(*$self);
      }
    }

    %template(Matrix3i) Matrix3<int>;
    %template(Matrix3d) Matrix3<double>;
    %template(Matrix3f) Matrix3<float>;
  }
}

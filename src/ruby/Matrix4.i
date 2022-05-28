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

%module matrix4
%{
#include <gz/math/config.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
%}

%include "std_string.i"

namespace gz
{
  namespace math
  {
    template<typename T>
    class Matrix4
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      %rename("%(uppercase)s", %$isstatic, %$isvariable) "";
      
      public: static const Matrix4<T> Identity;
      public: static const Matrix4<T> Zero;

      public: Matrix4();
      public: Matrix4(const Matrix4<T> &_m);
      public: Matrix4(T _v00, T _v01, T _v02, T _v03,
                      T _v10, T _v11, T _v12, T _v13,
                      T _v20, T _v21, T _v22, T _v23,
                      T _v30, T _v31, T _v32, T _v33);
      public: explicit Matrix4(const Quaternion<T> &_q);
      public: explicit Matrix4(const Pose3<T> &_pose) : Matrix4(_pose.Rot());
      public: virtual ~Matrix4() {};
      public: void Set(
            T _v00, T _v01, T _v02, T _v03,
            T _v10, T _v11, T _v12, T _v13,
            T _v20, T _v21, T _v22, T _v23,
            T _v30, T _v31, T _v32, T _v33);
      public: void Axis(const Vector3<T> &_axis, T _angle);
      public: void SetTranslation(const Vector3<T> &_t);
      public: void SetTranslation(T _x, T _y, T _z);
      public: Vector3<T> Translation() const;
      public: Vector3<T> Scale() const;
      public: Quaternion<T> Rotation() const;
      public: Vector3<T> EulerRotation(bool _firstSolution) const;
      public: Pose3<T> Pose() const;
      public: void Scale(const Vector3<T> &_s);
      public: void Scale(T _x, T _y, T _z);
      public: bool IsAffine() const;
      public: bool TransformAffine(const Vector3<T> &_v,
                                   Vector3<T> &_result) const;
      public: T Determinant() const;
      public: Matrix4<T> Inverse() const;
      public: void Transpose();
      public: Matrix4<T> Transposed() const;
      public: Matrix4<T> operator*=(const Matrix4<T> &_m2);
      public: Matrix4<T> operator*(const Matrix4<T> &_m2) const;
      public: Vector3<T> operator*(const Vector3<T> &_vec) const;
      public: bool Equal(const Matrix4 &_m, const T &_tol) const;
      public: bool operator==(const Matrix4<T> &_m) const;
      public: bool operator!=(const Matrix4<T> &_m) const;
      public: static Matrix4<T> LookAt(const Vector3<T> &_eye,
          const Vector3<T> &_target, const Vector3<T> &_up = Vector3<T>::UnitZ);
    };

    %extend Matrix4{
        T __call__(size_t row, size_t col) const {
          return (*$self)(row, col);
        }
    }

    %extend Matrix4 {
        std::string __str__() const {
          std::ostringstream out;
          out << *$self;
          return out.str();
        }
    }

    %template(Matrix4i) Matrix4<int>;
    %template(Matrix4d) Matrix4<double>;
    %template(Matrix4f) Matrix4<float>;
  }
}

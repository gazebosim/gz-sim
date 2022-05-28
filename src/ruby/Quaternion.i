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
#include <gz/math/Angle.hh>
#include <gz/math/config.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
%}

%include "std_string.i"

%inline %{
  template<typename D>
  struct AxisAngleOutput {
    gz::math::Vector3<D> axis;
    D angle;
  };
%}

namespace gz
{
  namespace math
  {
    template<typename T>
    class Quaternion
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      %rename("%(uppercase)s", %$isstatic, %$isvariable) "";

      public: static const Quaternion Identity;
      public: static const Quaternion Zero;

      public: Quaternion()
      : qw(1), qx(0), qy(0), qz(0);
      public: Quaternion(const T &_w, const T &_x, const T &_y, const T &_z)
      : qw(_w), qx(_x), qy(_y), qz(_z);
      public: Quaternion(const T &_roll, const T &_pitch, const T &_yaw);
      public: Quaternion(const Vector3<T> &_axis, const T &_angle);
      public: explicit Quaternion(const Vector3<T> &_rpy);
      public: Quaternion(const Quaternion<T> &_qt) = default;
      public: ~Quaternion() = default;
      public: void Invert();
      public: inline Quaternion<T> Inverse() const;
      public: Quaternion<T> Log() const;
      public: Quaternion<T> Exp() const;
      public: void Normalize();
      public: Quaternion<T> Normalized() const;
      public: void SetFromAxisAngle(T _ax, T _ay, T _az, T _aa);
      public: void SetFromAxisAngle(const Vector3<T> &_axis, T _a);
      public: void Set(T _w, T _x, T _y, T _z);
      public: void SetFromEuler(const Vector3<T> &_vec);
      public: void SetFromEuler(T _roll, T _pitch, T _yaw);
      public: Vector3<T> Euler() const;
      public: static Quaternion<T> EulerToQuaternion(const Vector3<T> &_vec);
      public: static Quaternion<T> EulerToQuaternion(T _x, T _y, T _z);
      public: T Roll() const;
      public: T Pitch() const;
      public: T Yaw() const;
      %rename(set_from_2_axes) SetFrom2Axes;
      public: void SetFrom2Axes(const Vector3<T> &_v1, const Vector3<T> &_v2);
      public: void Scale(T _scale);
      public: Quaternion<T> operator+(const Quaternion<T> &_qt) const;
      public: Quaternion<T> operator+=(const Quaternion<T> &_qt);
      public: Quaternion<T> operator-(const Quaternion<T> &_qt) const;
      public: Quaternion<T> operator-=(const Quaternion<T> &_qt);
      public: inline Quaternion<T> operator*(const Quaternion<T> &_q) const;
      public: Quaternion<T> operator*(const T &_f) const;
      public: Quaternion<T> operator*=(const Quaternion<T> &_qt);
      public: Vector3<T> operator*(const Vector3<T> &_v) const;
      public: bool Equal(const Quaternion<T> &_qt, const T &_tol) const;
      public: bool operator==(const Quaternion<T> &_qt) const;
      public: bool operator!=(const Quaternion<T> &_qt) const;
      public: Quaternion<T> operator-() const;
      public: inline Vector3<T> RotateVector(const Vector3<T> &_vec) const;
      public: Vector3<T> RotateVectorReverse(const Vector3<T> &_vec) const;
      public: bool IsFinite() const;
      public: inline void Correct();
      %rename(x_axis) XAxis;
      public: Vector3<T> XAxis() const;
      %rename(y_axis) YAxis;
      public: Vector3<T> YAxis() const;
      %rename(z_axis) ZAxis;
      public: Vector3<T> ZAxis() const;
      public: void Round(int _precision);
      public: T Dot(const Quaternion<T> &_q) const;
      public: static Quaternion<T> Squad(T _fT,
                  const Quaternion<T> &_rkP, const Quaternion<T> &_rkA,
                  const Quaternion<T> &_rkB, const Quaternion<T> &_rkQ,
                  bool _shortestPath = false);
      public: static Quaternion<T> Slerp(T _fT,
                  const Quaternion<T> &_rkP, const Quaternion<T> &_rkQ,
                  bool _shortestPath = false);
      public: Quaternion<T> Integrate(const Vector3<T> &_angularVelocity,
                                      const T _deltaT) const;

      public: inline void SetX(T _v);
      public: inline void SetY(T _v);
      public: inline void SetZ(T _v);
      public: inline void SetW(T _v);

      %pythoncode %{
      def axis_angle(self):
          axis_angle_output = self._axis_angle()
          return [axis_angle_output.axis, axis_angle_output.angle]
      %}
    };

    %extend Quaternion{
        inline AxisAngleOutput<T> _axis_angle() {
          gz::math::Vector3<T> axis;
          T angle;
          (*$self).AxisAngle(axis, angle);
          AxisAngleOutput<T> output;
          output.axis = axis;
          output.angle = angle;
          return output;
        }
    }

    %extend Quaternion{
        inline T W() const
        {
            return (*$self).W();
        }

        inline T X() const
        {
            return (*$self).X();
        }

        inline T Y() const
        {
            return (*$self).Y();
        }

        inline T Z() const
        {
            return (*$self).Z();
        }
    }

    %extend Quaternion {
        std::string __str__() const {
          std::ostringstream out;
          out << *$self;
          return out.str();
        }
    }

    %template(Quaterniond) Quaternion<double>;
    %template(Quaternionf) Quaternion<float>;
  }
}
    %template(AxisAngleOutputd) AxisAngleOutput<double>;
    %template(AxisAngleOutputf) AxisAngleOutput<float>;

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

%module inertial
%{
#include <gz/math/Inertial.hh>
#include <gz/math/config.hh>
#include "gz/math/MassMatrix3.hh"
#include "gz/math/Pose3.hh"
%}

namespace gz
{
  namespace math
  {
    template<typename T>
    class Inertial
    {
      public: Inertial();

      public: Inertial(const MassMatrix3<T> &_massMatrix,
                       const Pose3<T> &_pose);

      public: Inertial(const Inertial<T> &_inertial) = default;

      public: ~Inertial() = default;

      public: bool SetMassMatrix(const MassMatrix3<T> &_m,
                  const T _tolerance = GZ_MASSMATRIX3_DEFAULT_TOLERANCE<T>);

      public: const MassMatrix3<T> &MassMatrix() const;

      public: bool SetPose(const Pose3<T> &_pose);

      public: const Pose3<T> &Pose() const;

      public: Matrix3<T> Moi() const;

      public: bool SetInertialRotation(const Quaternion<T> &_q);

      public: bool SetMassMatrixRotation(const Quaternion<T> &_q,
                                         const T _tol = 1e-6);

      public: bool operator==(const Inertial<T> &_inertial) const;

      public: bool operator!=(const Inertial<T> &_inertial) const;

      public: Inertial<T> &operator+=(const Inertial<T> &_inertial);

      public: Inertial<T> &operator-=(const Inertial<T> &_inertial);

      public: const Inertial<T> operator+(const Inertial<T> &_inertial) const;

      public: const Inertial<T> operator-(const Inertial<T> &_inertial) const;
    };
    %template(Inertiald) Inertial<double>;

  }
}

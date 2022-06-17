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

%module massmatrix3
%{
#include "gz/math/MassMatrix3.hh"

#include "gz/math/Helpers.hh"
#include "gz/math/Material.hh"
#include "gz/math/Quaternion.hh"
#include "gz/math/Vector2.hh"
#include "gz/math/Vector3.hh"
#include "gz/math/Matrix3.hh"
%}

namespace gz
{
  namespace math
  {
    template<typename T>
    class MassMatrix3
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: MassMatrix3();
      public: MassMatrix3(const T &_mass,
                          const Vector3<T> &_ixxyyzz,
                          const Vector3<T> &_ixyxzyz);
      public: MassMatrix3(const MassMatrix3<T> &_m);
      public: ~MassMatrix3();
      public: bool SetMass(const T &_m);
      public: T Mass() const;
      public: bool SetInertiaMatrix(
                  const T &_ixx, const T &_iyy, const T &_izz,
                  const T &_ixy, const T &_ixz, const T &_iyz);
      public: Vector3<T> DiagonalMoments() const;
      public: Vector3<T> OffDiagonalMoments() const;
      public: bool SetDiagonalMoments(const Vector3<T> &_ixxyyzz);
      public: bool SetOffDiagonalMoments(const Vector3<T> &_ixyxzyz);
      public: T Ixx() const;
      public: T Iyy() const;
      public: T Izz() const;
      public: T Ixy() const;
      public: T Ixz() const;
      public: T Iyz() const;
      public: bool SetIxx(const T &_v);
      public: bool SetIyy(const T &_v);
      public: bool SetIzz(const T &_v);
      public: bool SetIxy(const T &_v);
      public: bool SetIxz(const T &_v);
      public: bool SetIyz(const T &_v);
      public: Matrix3<T> Moi() const;
      public: bool SetMoi(const Matrix3<T> &_moi);
      public: bool operator==(const MassMatrix3<T> &_m) const;
      public: bool operator!=(const MassMatrix3<T> &_m) const;
      public: bool IsNearPositive(const T _tolerance =
                  GZ_MASSMATRIX3_DEFAULT_TOLERANCE<T>) const;
      public: bool IsPositive(const T _tolerance =
                  GZ_MASSMATRIX3_DEFAULT_TOLERANCE<T>) const;
      public: T Epsilon(const T _tolerance =
                  GZ_MASSMATRIX3_DEFAULT_TOLERANCE<T>) const;
      public: static T Epsilon(const Vector3<T> &_moments,
                  const T _tolerance =
                  GZ_MASSMATRIX3_DEFAULT_TOLERANCE<T>);
      public: bool IsValid(const T _tolerance =
                  GZ_MASSMATRIX3_DEFAULT_TOLERANCE<T>) const;
      public: static bool ValidMoments(const Vector3<T> &_moments,
                  const T _tolerance = GZ_MASSMATRIX3_DEFAULT_TOLERANCE<T>);
      public: Vector3<T> PrincipalMoments(const T _tol = 1e-6) const;
      public: Quaternion<T> PrincipalAxesOffset(const T _tol = 1e-6) const;
      public: bool EquivalentBox(Vector3<T> &_size,
                                 Quaternion<T> &_rot,
                                 const T _tol = 1e-6) const;
      public: bool SetFromBox(const Material &_mat,
                              const Vector3<T> &_size,
                              const Quaternion<T> &_rot = Quaternion<T>::Identity);
      public: bool SetFromBox(const T _mass,
                              const Vector3<T> &_size,
                              const Quaternion<T> &_rot = Quaternion<T>::Identity);
      public: bool SetFromBox(const Vector3<T> &_size,
                              const Quaternion<T> &_rot = Quaternion<T>::Identity);
      public: bool SetFromCylinderZ(const Material &_mat,
                                    const T _length,
                                    const T _radius,
                                    const Quaternion<T> &_rot = Quaternion<T>::Identity);
      public: bool SetFromCylinderZ(const T _mass,
                                    const T _length,
                                    const T _radius,
                            const Quaternion<T> &_rot = Quaternion<T>::Identity);
      public: bool SetFromCylinderZ(const T _length,
                                    const T _radius,
                                    const Quaternion<T> &_rot);
      public: bool SetFromSphere(const Material &_mat, const T _radius);
      public: bool SetFromSphere(const T _mass, const T _radius);
      public: bool SetFromSphere(const T _radius);
    };

    %template(MassMatrix3d) MassMatrix3<double>;
    %template(MassMatrix3f) MassMatrix3<float>;
  }
}

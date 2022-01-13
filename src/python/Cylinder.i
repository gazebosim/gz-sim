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

%module cylinder
%{
#include <ignition/math/Cylinder.hh>
#include <ignition/math/config.hh>
#include <ignition/math/MassMatrix3.hh>
#include <ignition/math/Material.hh>
#include <ignition/math/Quaternion.hh>
%}

namespace ignition
{
  namespace math
  {
    template<typename Precision>
    class Cylinder
    {
      %rename("%(undercase)s", %$isfunction, notregexmatch$name="^[A-Z]*$") "";

      public: Cylinder() = default;

      public: Cylinder(const Precision _length, const Precision _radius,
                  const ignition::math::Quaternion<Precision> &_rotOffset =
                  ignition::math::Quaternion<Precision>::Identity);

      public: Cylinder(const Precision _length, const Precision _radius,
                  const ignition::math::Material &_mat,
                  const ignition::math::Quaternion<Precision> &_rotOffset =
                  ignition::math::Quaternion<Precision>::Identity);

      public: ~Cylinder() = default;

      public: Precision Radius() const;

      public: void SetRadius(const Precision _radius);

      public: Precision Length() const;

      public: void SetLength(const Precision _length);

      public: ignition::math::Quaternion<Precision> RotationalOffset() const;

      public: void SetRotationalOffset(
                  const ignition::math::Quaternion<Precision> &_rotOffset);

      public: const ignition::math::Material &Mat() const;

      public: void SetMat(const ignition::math::Material &_mat);

      public: bool MassMatrix(ignition::math::MassMatrix3<double> &_massMat) const;

      public: bool operator==(const Cylinder &_cylinder) const;

      public: Precision Volume() const;

      public: Precision DensityFromMass(const Precision _mass) const;

      public: bool SetDensityFromMass(const Precision _mass);
    };
    %template(Cylinderd) Cylinder<double>;
  }
}

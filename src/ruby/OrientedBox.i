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

%module orientedbox
%{
#include <iostream>
#include <gz/math/OrientedBox.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Material.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/config.hh>
%}

namespace ignition
{
  namespace math
  {
    template<typename T>
    class ignition::math::OrientedBox
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: OrientedBox();

      public: OrientedBox(
        const ignition::math::Vector3<T> &_size, const ignition::math::Pose3<T> &_pose);

      public: OrientedBox(const ignition::math::Vector3<T> &_size, const ignition::math::Pose3<T> &_pose,
                  const ignition::math::Material &_mat);

      public: explicit OrientedBox(const ignition::math::Vector3<T> &_size);

      public: explicit OrientedBox(const ignition::math::Vector3<T> &_size,
                                   const ignition::math::Material &_mat);

      public: OrientedBox(const ignition::math::OrientedBox<T> &_b);

      public: virtual ~OrientedBox();

      %rename(x_length) XLength;
      public: T XLength() const;

      %rename(y_length) YLength;
      public: T YLength() const;

      %rename(z_length) ZLength;
      public: T ZLength() const;

      public: const ignition::math::Vector3<T> &Size() const;

      public: const ignition::math::Pose3<T> &Pose() const;

      public: void Size(ignition::math::Vector3<T> &_size);

      public: void Pose(ignition::math::Pose3<T> &_pose);

      public: bool operator==(const ignition::math::OrientedBox<T> &_b) const;

      public: bool operator!=(const ignition::math::OrientedBox<T> &_b) const;

      public: bool Contains(const ignition::math::Vector3<double> &_p) const;

      public: const ignition::math::ignition::math::Material &ignition::math::Material() const;

      public: void ignition::math::SetMaterial(const ignition::math::ignition::math::Material &_mat);

      public: T Volume() const;

      public: T DensityFromMass(const T _mass) const;

      public: bool SetDensityFromMass(const T _mass);

      public: bool MassMatrix(MassMatrix3<T> &_massMat) const;
    };
    %template(OrientedBoxd) OrientedBox<double>;
  }
}

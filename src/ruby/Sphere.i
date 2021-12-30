/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

%module sphere
%{
#include "ignition/math/Sphere.hh"
#include "ignition/math/MassMatrix3.hh"
#include "ignition/math/Material.hh"
#include "ignition/math/Quaternion.hh"
#include "ignition/math/Plane.hh"
%}

%include "typemaps.i"
%typemap(out) (std::optional< ignition::math::Vector3< double > >) %{
  if((*(&result)).has_value()) {
    $result = SWIG_NewPointerObj(
      (new ignition::math::Vector3< double >(static_cast< const ignition::math::Vector3< double >& >((*(&result)).value()))),
      SWIGTYPE_p_ignition__math__Vector3T_double_t,
      SWIG_POINTER_OWN |  0 );
  } else {
    $result = Py_None;
    Py_INCREF(Py_None);
  }
%}

namespace ignition
{
  namespace math
  {
    template<typename Precision>
    class Sphere
    {
      public: Sphere() = default;

      public: explicit Sphere(const Precision _radius);

      public: Sphere(const Precision _radius, const ignition::math::Material &_mat);

      public: ~Sphere() = default;

      public: Precision Radius() const;

      public: void SetRadius(const Precision _radius);

      public: const ignition::math::Material &Material() const;

      public: void SetMaterial(const ignition::math::Material &_mat);

      public: bool MassMatrix(ignition::math::MassMatrix3<double> &_massMat) const;

      public: bool operator==(const Sphere &_sphere) const;

      public: bool operator!=(const Sphere &_sphere) const;

      public: Precision Volume() const;

      public: Precision VolumeBelow(const ignition::math::Plane<Precision> &_plane) const;

      public: std::optional<ignition::math::Vector3<Precision>>
        CenterOfVolumeBelow(const ignition::math::Plane<Precision> &_plane) const;

      public: Precision DensityFromMass(const Precision _mass) const;

      public: bool SetDensityFromMass(const Precision _mass);
    };
    %template(Sphered) Sphere<double>;
  }
}

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

%module box
%{
#include <ignition/math/Box.hh>
#include <ignition/math/config.hh>
#include <ignition/math/MassMatrix3.hh>
#include <ignition/math/Material.hh>
#include <ignition/math/Plane.hh>
#include <ignition/math/Vector3.hh>

#include "ignition/math/detail/WellOrderedVector.hh"

#include <set>
#include <optional>
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

%typemap(out) (std::optional< ignition::math::Vector3< int > >) %{
  if((*(&result)).has_value()) {
    $result = SWIG_NewPointerObj(
      (new ignition::math::Vector3< int >(static_cast< const ignition::math::Vector3< int >& >((*(&result)).value()))),
      SWIGTYPE_p_ignition__math__Vector3T_int_t,
      SWIG_POINTER_OWN |  0 );
  } else {
    $result = Py_None;
    Py_INCREF(Py_None);
  }
%}

#include "std_set.i"
%template(SetBoxDouble) std::set<ignition::math::Vector3<double>, ignition::math::WellOrderedVectors<double>>;
%template(SetBoxInt) std::set<ignition::math::Vector3<int>, ignition::math::WellOrderedVectors<int>>;

namespace ignition
{
  namespace math
  {
    template<typename Precision>
    class Box
    {
      %rename("%(undercase)s", %$isfunction, notregexmatch$name="^[A-Z]*$") "";

      public: Box() = default;

      public: Box(const Precision _length,
                  const Precision _width,
                  const Precision _height);

      public: Box(const Precision _length, const Precision _width,
                  const Precision _height,
                  const ignition::math::Material &_mat);

      public: explicit Box(const ignition::math::Vector3<Precision> &_size);

      public: Box(const ignition::math::Vector3<Precision> &_size,
                  const ignition::math::Material &_mat);

      public: virtual ~Box() = default;

      public: ignition::math::Vector3<Precision> Size() const;

      public: void SetSize(const ignition::math::Vector3<Precision> &_size);

      public: void SetSize(const Precision _length,
                           const Precision _width,
                           const Precision _height);

      public: bool operator==(const Box<Precision> &_b) const;

      public: bool operator!=(const Box<Precision> &_b) const;

      public: const ignition::math::Material &Material() const;

      public: void SetMaterial(const ignition::math::Material &_mat);

      public: Precision Volume() const;

      public: Precision VolumeBelow(const ignition::math::Plane<Precision> &_plane) const;

      public: std::optional<ignition::math::Vector3<Precision>>
        CenterOfVolumeBelow(const ignition::math::Plane<Precision> &_plane) const;

      public: std::set<ignition::math::Vector3<Precision>, ignition::math::WellOrderedVectors<Precision>>
        VerticesBelow(const ignition::math::Plane<Precision> &_plane) const;

      public: Precision DensityFromMass(const Precision _mass) const;

      public: bool SetDensityFromMass(const Precision _mass);

      public: bool MassMatrix(ignition::math::MassMatrix3<Precision> &_massMat) const;

      public: std::set<ignition::math::Vector3<Precision>, ignition::math::WellOrderedVectors<Precision>> Intersections(
        const ignition::math::Plane<Precision> &_plane) const;

      private: ignition::math::Vector3<Precision> size = ignition::math::Vector3<Precision>::Zero;

      private: ignition::math::Material material;
    };

    %template(Boxi) Box<int>;
    %template(Boxd) Box<double>;
  }
}

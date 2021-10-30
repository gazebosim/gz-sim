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

%module plane
%{
#include <ignition/math/Plane.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/config.hh>
#include <ignition/math/Line2.hh>
#include <ignition/math/Quaternion.hh>
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


namespace ignition
{
  namespace math
  {
    template<typename T>
    class Plane
    {
      public: enum PlaneSide
      {
        /// \brief Negative side of the plane. This is the side that is
        /// opposite the normal.
        NEGATIVE_SIDE = 0,

        /// \brief Positive side of the plane. This is the side that has the
        /// normal vector.
        POSITIVE_SIDE = 1,

        /// \brief On the plane.
        NO_SIDE = 2,

        /// \brief On both sides of the plane.
        BOTH_SIDE = 3
      };

      public: Plane();

      public: Plane(const Vector3<T> &_normal, T _offset = 0.0);

      public: Plane(const Vector3<T> &_normal, const Vector2<T> &_size,
                    T _offset);
      public: Plane(const Plane &_plane);

      public: virtual ~Plane();

      public: void Set(const Vector3<T> &_normal, T _offset);

      public: void Set(const Vector3<T> &_normal, const Vector2<T> &_size,
                       T _offset);

      public: T Distance(const Vector3<T> &_point) const;

      public: std::optional<Vector3<T>> Intersection(
        const Vector3<T> &_point,
        const Vector3<T> &_gradient,
        const double &_tolerance = 1e-6) const;

      public: PlaneSide Side(const Vector3<T> &_point) const;

      public: PlaneSide Side(const math::AxisAlignedBox &_box) const;

      public: T Distance(const Vector3<T> &_origin,
                         const Vector3<T> &_dir) const;

      public: inline const Vector2<T> &Size() const;

      public: inline Vector2<T> &Size();

      public: inline const Vector3<T> &Normal() const;

      public: inline Vector3<T> &Normal();

      public: inline T Offset() const;

      private: Vector3<T> normal;

      private: Vector2<T> size;

      private: T d;
    };

    %template(Planed) Plane<double>;
  }
}

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

%module axisalignedbox
%{
#include <iostream>
#include <tuple>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/config.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Line3.hh>
#include <ignition/math/Vector3.hh>
%}

%include "typemaps.i"

%typemap(out) (std::tuple<bool, double>) {
  $result = PyTuple_New(2);
  PyTuple_SET_ITEM($result, 0, PyBool_FromLong(std::get<0>($1)));
  PyTuple_SET_ITEM($result, 1, PyFloat_FromDouble(std::get<1>($1)));
}

%typemap(out) (std::tuple<bool, double, ignition::math::Vector3< double > >) {
  $result = PyTuple_New(3);
  std::tuple<bool, double, ignition::math::Vector3< double > > tuplecpp($1);
  PyTuple_SET_ITEM($result, 0, PyBool_FromLong(std::get<0>(tuplecpp)));
  PyTuple_SET_ITEM($result, 1, PyFloat_FromDouble(std::get<1>(tuplecpp)));

  ignition::math::Vector3<double> vector3 = std::get<2>(tuplecpp);
  PyObject *pyobject =  SWIG_NewPointerObj((new ignition::math::Vector3< double >(static_cast< const ignition::math::Vector3< double >& >(vector3))), SWIGTYPE_p_ignition__math__Vector3T_double_t, SWIG_POINTER_OWN |  0 );
  PyTuple_SET_ITEM(resultobj, 2, pyobject);
}


namespace ignition
{
  namespace math
  {
    class AxisAlignedBox
    {
      %rename("%(undercase)s", %$isfunction, notregexmatch$name="^[A-Z]*$") "";

      public: AxisAlignedBox();

      public: AxisAlignedBox(const math::Vector3<double> &_vec1, const math::Vector3<double> &_vec2);

      public: AxisAlignedBox(double _vec1X, double _vec1Y, double _vec1Z,
                  double _vec2X, double _vec2Y, double _vec2Z);

      public: AxisAlignedBox(const AxisAlignedBox &_b);

      public: virtual ~AxisAlignedBox();

      %rename(x_length) XLength;
      public: double XLength() const;

      %rename(y_length) YLength;
      public: double YLength() const;

      %rename(z_length) ZLength;
      public: double ZLength() const;

      public: math::Vector3<double> Size() const;

      public: math::Vector3<double> Center() const;

      public: void Merge(const AxisAlignedBox &_box);

      public: AxisAlignedBox operator+(const AxisAlignedBox &_b) const;

      public: const AxisAlignedBox &operator+=(const AxisAlignedBox &_b);

      public: bool operator==(const AxisAlignedBox &_b) const;

      public: bool operator!=(const AxisAlignedBox &_b) const;

      public: AxisAlignedBox operator-(const math::Vector3<double> &_v);

      public: AxisAlignedBox operator+(const math::Vector3<double> &_v);

      public: const math::Vector3<double> &Min() const;

      public: const math::Vector3<double> &Max() const;

      public: math::Vector3<double> &Min();

      public: math::Vector3<double> &Max();

      public: bool Intersects(const AxisAlignedBox &_box) const;

      public: bool Contains(const math::Vector3<double> &_p) const;

      public: bool IntersectCheck(const math::Vector3<double> &_origin, const math::Vector3<double> &_dir,
                  const double _min, const double _max) const;

      public: std::tuple<bool, double> IntersectDist(
                  const math::Vector3<double> &_origin, const math::Vector3<double> &_dir,
                  const double _min, const double _max) const;

      public: std::tuple<bool, double, math::Vector3<double>> Intersect(
                  const math::Line3<double> &_line) const;

      public: std::tuple<bool, double, math::Vector3<double>> Intersect(
          const math::Vector3<double> &_origin, const math::Vector3<double> &_dir,
          const double _min, const double _max) const;

      /// \brief Get the volume of the box in m^3.
      /// \return Volume of the box in m^3.
      public: double Volume() const;
    };
  }
}

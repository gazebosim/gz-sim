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
#include <tuple>

#include <pybind11/stl.h>

#include "Helpers.hh"
#include <ignition/math/Helpers.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
namespace math
{
namespace python
{
class Helpers
{
};

/// \brief Compute sphere volume
/// \param[in] _radius Sphere radius
/// \return sphere volume
float SphereVolume(const float _radius)
{
  return IGN_SPHERE_VOLUME(_radius);
}

/// \brief Compute cylinder volume
/// \param[in] _r Cylinder base radius
/// \param[in] _l Cylinder length
/// \return cylinder volume
float CylinderVolume(const float _r, const float _l)
{
  return IGN_CYLINDER_VOLUME(_r, _l);
}

/// \brief Compute box volume
/// \param[in] _x X length
/// \param[in] _y Y length
/// \param[in] _z Z length
/// \return box volume
float BoxVolume(const float _x, const float _y, const float _z)
{
  return IGN_BOX_VOLUME(_x, _y, _z);
}

/// \brief Compute box volume from a vector
/// \param[in] _v Vector3d that contains the box's dimensions.
/// \return box volume from a vector
float BoxVolumeV(const ignition::math::Vector3d &_v)
{
  return IGN_BOX_VOLUME_V(_v);
}

/// \brief Sort two numbers, such that _a <= _b
/// \param[out] _a the first number
/// \param[out] _b the second number
/// \return a tuple with the numbers sorted
std::tuple<float, float> Sort2(float _a, float _b)
{
  sort2(_a, _b);
  return std::make_tuple(_a, _b);
}

/// \brief Sort three numbers, such that _a <= _b <= _c
/// \param[out] _a the first number
/// \param[out] _b the second number
/// \param[out] _c the third number
/// \return a tuple with the numbers sorted
std::tuple<float, float, float> Sort3(float _a, float _b, float _c)
{
  sort3(_a, _b, _c);
  return std::make_tuple(_a, _b, _c);
}

void defineMathHelpers(py::module &m)
{
  using Class = Helpers;

  m.def("clamp", &ignition::math::clamp<float>, "Simple clamping function")
   .def("clamp", &ignition::math::clamp<int>, "Simple clamping function")
   .def("isnan",
        py::overload_cast<float>(&ignition::math::isnan),
        "Check if a float is NaN")
   .def("fixnan",
        py::overload_cast<float>(&ignition::math::fixnan),
        "Fix a nan value.")
   .def("is_even",
        py::overload_cast<int>(&ignition::math::isEven),
        "Check if parameter is even.")
   .def("is_odd",
        py::overload_cast<int>(&ignition::math::isOdd),
        "Check if parameter is odd.")
   .def("sgn",
      &ignition::math::sgn<float>,
      "Returns 0 for zero values, -1 for negative values and +1 for positive"
      " values.")
   .def("signum",
        &ignition::math::signum<float>,
        "Returns 0 for zero values, -1 for negative values and "
        "+1 for positive values.")
   .def("mean", &ignition::math::mean<float>, "Get mean of vector of values")
   .def("variance",
        &ignition::math::variance<float>,
        "Get variance of vector of values")
   .def("max",
        &ignition::math::max<float>,
        "Get the maximum value of vector of values")
   .def("max",
        &ignition::math::max<int>,
        "Get the maximum value of vector of values")
   .def("min",
        &ignition::math::min<float>,
        "Get the minimum value of vector of values")
   .def("min",
        &ignition::math::min<int>,
        "Get the minimum value of vector of values")
   .def("equal",
        &ignition::math::equal<float>,
        "check if two values are equal, within a tolerance")
   .def("less_or_near_equal",
        &ignition::math::lessOrNearEqual<float>,
        "Inequality test, within a tolerance")
   .def("greater_or_near_equal",
        &ignition::math::greaterOrNearEqual<float>,
        "Inequality test, within a tolerance")
   .def("precision",
        &ignition::math::precision<float>,
        "Get value at a specified precision")
   .def("sort2",
        &Sort2,
        "Sort two numbers, such that _a <= _b")
   .def("sort3",
        &Sort3,
        "Sort three numbers, such that _a <= _b <= _c")
   .def("is_power_of_two",
        &ignition::math::isPowerOfTwo,
        "Is this a power of 2?")
   .def("round_up_power_of_two",
        &ignition::math::roundUpPowerOfTwo,
        "Get the smallest power of two that is greater or equal to a given "
        "value")
   .def("round_up_multiple",
        &ignition::math::roundUpMultiple,
        "Round a number up to the nearest multiple")
   .def("parse_int",
        &ignition::math::parseInt,
        "parse string into an integer")
   .def("parse_float",
        &ignition::math::parseFloat,
        "parse string into an float")
   .def("ign_sphere_volume",
        &SphereVolume,
        "Compute sphere volume")
   .def("ign_cylinder_volume",
        &CylinderVolume,
        "Compute cylinder volume")
   .def("ign_box_volume",
        &BoxVolume,
        "Compute box volume")
   .def("ign_box_volume_v",
        &BoxVolumeV,
        "Compute box volume from vector");
   py::class_<Class>(m,
                    "Helpers",
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def_readonly_static("IGNZEROSIZET", &IGN_ZERO_SIZE_T, "IGN_PI")
  .def_readonly_static("IGN_ONE_SIZE_T", &IGN_ONE_SIZE_T)
  .def_readonly_static("IGN_TWO_SIZE_T", &IGN_TWO_SIZE_T)
  .def_readonly_static("IGN_THREE_SIZE_T", &IGN_THREE_SIZE_T)
  .def_readonly_static("IGN_FOUR_SIZE_T", &IGN_FOUR_SIZE_T)
  .def_readonly_static("IGN_FIVE_SIZE_T", &IGN_FIVE_SIZE_T)
  .def_readonly_static("IGN_SIX_SIZE_T", &IGN_SIX_SIZE_T)
  .def_readonly_static("IGN_SEVEN_SIZE_T", &IGN_SEVEN_SIZE_T)
  .def_readonly_static("IGN_EIGHT_SIZE_T", &IGN_EIGHT_SIZE_T)
  .def_readonly_static("IGN_NINE_SIZE_T", &IGN_NINE_SIZE_T)
  .def_readonly_static("MAX_D", &MAX_D)
  .def_readonly_static("MIN_D", &MIN_D)
  .def_readonly_static("LOW_D", &LOW_D)
  .def_readonly_static("INF_D", &INF_D)
  .def_readonly_static("NAN_D", &NAN_D)
  .def_readonly_static("MAX_F", &MAX_F)
  .def_readonly_static("MIN_F", &MIN_F)
  .def_readonly_static("LOW_F", &LOW_F)
  .def_readonly_static("INF_F", &INF_F)
  .def_readonly_static("NAN_F", &NAN_F)
  .def_readonly_static("MAX_UI16", &MAX_UI16)
  .def_readonly_static("MIN_UI16", &MIN_UI16)
  .def_readonly_static("LOW_UI16", &LOW_UI16)
  .def_readonly_static("INF_UI16", &INF_UI16)
  .def_readonly_static("MAX_I16", &MAX_I16)
  .def_readonly_static("MIN_I16", &MIN_I16)
  .def_readonly_static("LOW_I16", &LOW_I16)
  .def_readonly_static("INF_I16", &INF_I16)
  .def_readonly_static("MAX_UI32", &MAX_UI32)
  .def_readonly_static("MIN_UI32", &MIN_UI32)
  .def_readonly_static("LOW_UI32", &LOW_UI32)
  .def_readonly_static("INF_UI32", &INF_UI32)
  .def_readonly_static("MAX_I32", &MAX_I32)
  .def_readonly_static("MIN_I32", &MIN_I32)
  .def_readonly_static("LOW_I32", &LOW_I32)
  .def_readonly_static("INF_I32", &INF_I32)
  .def_readonly_static("MAX_UI64", &MAX_UI64)
  .def_readonly_static("MIN_UI64", &MIN_UI64)
  .def_readonly_static("LOW_UI64", &LOW_UI64)
  .def_readonly_static("INF_UI64", &INF_UI64)
  .def_readonly_static("MAX_I64", &MAX_I64)
  .def_readonly_static("MIN_I64", &MIN_I64)
  .def_readonly_static("LOW_I64", &LOW_I64)
  .def_readonly_static("INF_I64", &INF_I64)
  .def_readonly_static("NAN_I", &NAN_I);
}
}  // namespace python
}  // namespace gazebo
}  // namespace ignition

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
#include <limits>
#include <map>
#include <string>

#include "Material.hh"
#include <gz/math/Material.hh>
#include <gz/math/MaterialType.hh>

#include <pybind11/operators.h>
#include <pybind11/stl_bind.h>

namespace gz
{
namespace math
{
namespace python
{
void defineMathMaterial(py::module &m, const std::string &typestr)
{

  py::bind_map<std::map<
    gz::math::MaterialType, gz::math::Material>>
      (m, "MaterialMap");

  using Class = gz::math::Material;
  std::string pyclass_name = typestr;
  py::class_<Class> (m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
   .def(py::init<>())
   .def(py::init<const gz::math::MaterialType>())
   .def(py::init<const std::string&>())
   .def(py::init<const double>())
   .def(py::init<const Class&>())
   .def(py::self != py::self)
   .def(py::self == py::self)
   .def("predefined",
        &Class::Predefined,
        "Get all the built-in materials.")
   .def("set_to_nearest_density",
        &Class::SetToNearestDensity,
        py::arg("_value") = 0,
        py::arg("_epsilon") = std::numeric_limits<double>::max(),
        "Set this Material to the built-in Material with "
        "the nearest density value within _epsilon. If a built-in material "
        "could not be found, then this Material is not changed.")
   .def("type",
        &Class::Type,
        "Set the material's type. This will only set the type value. "
        "Other properties, such as density, will not be changed.")
   .def("set_type",
        &Class::SetType,
        "Set the material's type. This will only set the type value. "
        "Other properties, such as density, will not be changed.")
   .def("name",
        &Class::Name,
        "Get the name of the material. This will match the enum type "
        "names used in ::MaterialType, but in lowercase, if a built-in "
        "material is used.")
   .def("set_name",
        &Class::SetName,
        "Set the name of the material.")
   .def("set_density",
        &Class::SetDensity,
        "Set the name of the material.")
   .def("density", &Class::Density,
        "Set the density value of the material in kg/m^3.");

   py::enum_<gz::math::MaterialType>(m, "MaterialType")
       .value("STYROFOAM", gz::math::MaterialType::STYROFOAM)
       .value("PINE", gz::math::MaterialType::PINE)
       .value("WOOD", gz::math::MaterialType::WOOD)
       .value("OAK", gz::math::MaterialType::OAK)
       .value("PLASTIC", gz::math::MaterialType::PLASTIC)
       .value("CONCRETE", gz::math::MaterialType::CONCRETE)
       .value("ALUMINUM", gz::math::MaterialType::ALUMINUM)
       .value("STEEL_ALLOY", gz::math::MaterialType::STEEL_ALLOY)
       .value("STEEL_STAINLESS", gz::math::MaterialType::STEEL_STAINLESS)
       .value("IRON", gz::math::MaterialType::IRON)
       .value("BRASS", gz::math::MaterialType::BRASS)
       .value("COPPER", gz::math::MaterialType::COPPER)
       .value("TUNGSTEN", gz::math::MaterialType::TUNGSTEN)
       .value("UNKNOWN_MATERIAL",
              gz::math::MaterialType::UNKNOWN_MATERIAL)
       .export_values();
}
}  // namespace python
}  // namespace math
}  // namespace gz

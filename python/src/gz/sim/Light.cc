/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
 */


#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "Light.hh"

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{
void defineSimLight(py::object module)
{
  py::class_<gz::sim::Light>(module, "Light")
  .def(py::init<gz::sim::Entity>())
  .def(py::init<gz::sim::Light>())
  .def("entity", &gz::sim::Light::Entity,
      "Get the entity which this light is related to.")
  .def("reset_entity", &gz::sim::Light::ResetEntity,
      "Reset Entity to a new one.")
  .def("valid", &gz::sim::Light::Valid,
      py::arg("ecm"),
      "Check whether this light correctly refers to an entity that "
      "has a components::Light.")
  .def("name", &gz::sim::Light::Name,
      py::arg("ecm"),
      "Get the light's unscoped name.")
  .def("pose", &gz::sim::Light::Pose,
      py::arg("ecm"),
      "Get the pose of the light. "
      "The pose is given w.r.t the light's parent. which can be a world or "
      "a link.")
  .def("type", &gz::sim::Light::Type,
      py::arg("ecm"),
      "Get the light type.")
  .def("diffuse_color", &gz::sim::Light::DiffuseColor,
      py::arg("ecm"),
      "Get the light diffuse color.")
  .def("specular_color", &gz::sim::Light::SpecularColor,
      py::arg("ecm"),
      "Get the light specular color.")
  .def("cast_shadows", &gz::sim::Light::CastShadows,
      py::arg("ecm"),
      "Get whether the light casts shadows.")
  .def("intensity", &gz::sim::Light::Intensity,
      py::arg("ecm"),
      "Get the light intensity.")
  .def("direction", &gz::sim::Light::Direction,
      py::arg("ecm"),
      "Get the light direction.")
  .def("attenuation_range", &gz::sim::Light::AttenuationRange,
      py::arg("ecm"),
      "Get the light attenuation range. "
      "Light attenuation is not applicable to directional lights.")
  .def("attenuation_constant", &gz::sim::Light::AttenuationConstant,
      py::arg("ecm"),
      "Get the light attenuation constant value. "
      "Light attenuation is not applicable to directional lights.")
  .def("attenuation_linear", &gz::sim::Light::AttenuationLinear,
      py::arg("ecm"),
      "Get the light attenuation linear value. "
      "Light attenuation is not applicable to directional lights.")
  .def("attenuation_quadratic", &gz::sim::Light::AttenuationQuadratic,
      py::arg("ecm"),
      "Get the light attenuation quadratic value. "
      "Light attenuation is not applicable to directional lights.")
  .def("spot_inner_angle", &gz::sim::Light::SpotInnerAngle,
      py::arg("ecm"),
      "Get the inner angle of light. Applies to spot lights only.")
  .def("spot_outer_angle", &gz::sim::Light::SpotOuterAngle,
      py::arg("ecm"),
      "Get the outer angle of light. Applies to spot lights only.")
  .def("spot_falloff", &gz::sim::Light::SpotFalloff,
      py::arg("ecm"),
      "Get the fall off value of light. Applies to spot lights only.")
  .def("set_pose", &gz::sim::Light::SetPose,
      py::arg("ecm"),
      py::arg("pose"),
      "Set the pose of this light.")
  .def("set_diffuse_color", &gz::sim::Light::SetDiffuseColor,
      py::arg("ecm"),
      py::arg("color"),
      "Set the diffuse color of this light.")
  .def("set_specular_color", &gz::sim::Light::SetSpecularColor,
      py::arg("ecm"),
      py::arg("color"),
      "Set the specular color of this light.")
  .def("set_cast_shadows", &gz::sim::Light::SetCastShadows,
      py::arg("ecm"),
      py::arg("cast_shadows"),
      "Set whether the light casts shadows.")
  .def("set_intensity", &gz::sim::Light::SetIntensity,
      py::arg("ecm"),
      py::arg("value"),
      "Set light intensity.")
  .def("set_direction", &gz::sim::Light::SetDirection,
      py::arg("ecm"),
      py::arg("direction"),
      "Set light direction. Applies to directional lights.")
  .def("set_attenuation_range", &gz::sim::Light::SetAttenuationRange,
      py::arg("ecm"),
      py::arg("range"),
      "Set attenuation range of this light.")
  .def("set_attenuation_constant", &gz::sim::Light::SetAttenuationConstant,
      py::arg("ecm"),
      py::arg("value"),
      "Set attenuation constant value of this light.")
  .def("set_attenuation_linear", &gz::sim::Light::SetAttenuationLinear,
      py::arg("ecm"),
      py::arg("value"),
      "Set attenuation linear value of this light.")
  .def("set_attenuation_quadratic", &gz::sim::Light::SetAttenuationQuadratic,
      py::arg("ecm"),
      py::arg("color"),
      "Set attenuation quadratic value of this light.")
  .def("set_spot_inner_angle", &gz::sim::Light::SetSpotInnerAngle,
      py::arg("ecm"),
      py::arg("angle"),
      "Set inner angle for this light. Applies to spot lights only.")
  .def("set_spot_outer_angle", &gz::sim::Light::SetSpotOuterAngle,
      py::arg("ecm"),
      py::arg("angle"),
      "Set outer angle for this light. Applies to spot lights only.")
  .def("set_spot_falloff", &gz::sim::Light::SetSpotFalloff,
      py::arg("ecm"),
      py::arg("falloff"),
      "Set fall off value for this light. Applies to spot lights only.")
  .def("parent", &gz::sim::Light::Parent,
      py::arg("ecm"),
      "Get the parent entity. This can be a world or a link.")
  .def("__copy__",
      [](const gz::sim::Light &self)
      {
        return gz::sim::Light(self);
      })
  .def("__deepcopy__",
      [](const gz::sim::Light &self, pybind11::dict)
      {
        return gz::sim::Light(self);
      });
}
}  // namespace python
}  // namespace sim
}  // namespace gz

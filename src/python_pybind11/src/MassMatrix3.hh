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

#ifndef IGNITION_MATH_PYTHON__MASSMATRIX3_HH_
#define IGNITION_MATH_PYTHON__MASSMATRIX3_HH_

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <string>

#include <ignition/math/Helpers.hh>
#include <ignition/math/MassMatrix3.hh>

namespace py = pybind11;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::MassMatrix3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathMassMatrix3(py::module &m, const std::string &typestr);

/// Help define a pybind11 wrapper for an ignition::math::MassMatrix3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathMassMatrix3(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::MassMatrix3<T>;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def(py::init<const Class&>())
  .def(py::init<const T&, const ignition::math::Vector3<T>&,
                const ignition::math::Vector3<T>&>())
  .def("set_mass", &Class::SetMass, "Set the mass.")
  .def("mass", py::overload_cast<>(&Class::Mass, py::const_), "Get the mass")
  .def("ixx", &Class::Ixx, "Get ixx")
  .def("ixy", &Class::Ixy, "Get ixy")
  .def("ixz", &Class::Ixz, "Get ixz")
  .def("iyy", &Class::Iyy, "Get iyy")
  .def("iyz", &Class::Iyz, "Get iyz")
  .def("izz", &Class::Izz, "Get izz")
  .def("set_ixx", &Class::SetIxx, "Set ixx")
  .def("set_ixy", &Class::SetIxy, "Set ixy")
  .def("set_ixz", &Class::SetIxz, "Set ixz")
  .def("set_iyy", &Class::SetIyy, "Set iyy")
  .def("set_iyz", &Class::SetIyz, "Set iyz")
  .def("set_izz", &Class::SetIzz, "Set izz")
  .def("moi", &Class::Moi, "Returns Moments of Inertia as a Matrix")
  .def("set_moi",
       &Class::SetMoi,
       "Sets Moments of Inertia (MOI) from a Matrix3.")
  .def("set_inertia_matrix",
       &Class::SetInertiaMatrix,
       "Set the moment of inertia matrix.")
  .def("off_diagonal_moments",
       py::overload_cast<>(&Class::OffDiagonalMoments, py::const_),
       "Get the off-diagonal moments of inertia (Ixy, Ixz, Iyz).")
  .def("set_off_diagonal_moments",
       &Class::SetOffDiagonalMoments,
       "Set the off-diagonal moments of inertia (Ixy, Ixz, Iyz).")
  .def("diagonal_moments",
       py::overload_cast<>(&Class::DiagonalMoments, py::const_),
       "Get the diagonal moments of inertia (Ixx, Iyy, Izz).")
  .def("set_diagonal_moments",
       &Class::SetDiagonalMoments,
       "Set the diagonal moments of inertia (Ixx, Iyy, Izz).")
  .def("set_from_box",
       py::overload_cast<const Material&, const ignition::math::Vector3<T>&,
                         const ignition::math::Quaternion<T>&>
                         (&Class::SetFromBox),
       py::arg("_mat") = ignition::math::Material(),
       py::arg("_size") = ignition::math::Vector3<T>::Zero,
       py::arg("_rot") = ignition::math::Quaternion<T>::Identity,
       "Set inertial properties based on a Material and equivalent box.")
  .def("set_from_box",
       py::overload_cast<const T, const ignition::math::Vector3<T>&,
                         const ignition::math::Quaternion<T>&>
                         (&Class::SetFromBox),
       py::arg("_mass") = 0,
       py::arg("_size") = ignition::math::Vector3<T>::Zero,
       py::arg("_rot") = ignition::math::Quaternion<T>::Identity,
       "Set inertial properties based on a Material and equivalent box.")
  .def("set_from_box",
       py::overload_cast<const ignition::math::Vector3<T>&,
                         const ignition::math::Quaternion<T>&>
                         (&Class::SetFromBox),
       py::arg("_size") = ignition::math::Vector3<T>::Zero,
       py::arg("_rot") = ignition::math::Quaternion<T>::Identity,
       "Set inertial properties based on a Material and equivalent box.")
  .def("set_from_cylinder_z",
       py::overload_cast<const Material&, const T, const T,
                         const ignition::math::Quaternion<T>&>
                         (&Class::SetFromCylinderZ),
       py::arg("_mat") = ignition::math::Material(),
       py::arg("_length") = 0,
       py::arg("_radius") = 0,
       py::arg("_rot") = ignition::math::Quaternion<T>::Identity,
       "Set inertial properties based on a Material and equivalent "
       "cylinder aligned with Z axis.")
  .def("set_from_cylinder_z",
       py::overload_cast<const T, const T, const T,
                         const ignition::math::Quaternion<T>&>
                         (&Class::SetFromCylinderZ),
       py::arg("_mass") = 0,
       py::arg("_length") = 0,
       py::arg("_radius") = 0,
       py::arg("_rot") = ignition::math::Quaternion<T>::Identity,
       "Set inertial properties based on a Material and equivalent "
       "cylinder aligned with Z axis.")
  .def("set_from_cylinder_z",
       py::overload_cast<const T, const T,
                         const ignition::math::Quaternion<T>&>
                         (&Class::SetFromCylinderZ),
       py::arg("_length") = 0,
       py::arg("_radius") = 0,
       py::arg("_rot") = ignition::math::Quaternion<T>::Identity,
       "Set inertial properties based on a Material and equivalent "
       "cylinder aligned with Z axis.")
  .def("set_from_sphere",
       py::overload_cast<const Material&, const T>
                         (&Class::SetFromSphere),
       "Set inertial properties based on a material and "
       "equivalent sphere.")
  .def("set_from_sphere",
       py::overload_cast<const T, const T>
                         (&Class::SetFromSphere),
       "Set inertial properties based on mass and equivalent sphere.")
  .def("set_from_sphere",
       py::overload_cast<const T>
                         (&Class::SetFromSphere),
       "Set inertial properties based on equivalent sphere "
       "using the current mass value.")
  .def("equivalent_box",
       &Class::EquivalentBox,
       py::arg("_size") = ignition::math::Vector3<T>::Zero,
       py::arg("_rot") = ignition::math::Quaternion<T>::Identity,
       py::arg("_tol") = 1e-6,
       "Get dimensions and rotation offset of uniform box "
       "with equivalent mass and moment of inertia.")
  .def("principal_axes_offset",
       &Class::PrincipalAxesOffset,
       py::arg("_tol") = 1e-6,
       "Compute rotational offset of principal axes.")
  .def("principal_moments",
       &Class::PrincipalMoments,
       py::arg("_tol") = 1e-6,
       "Compute principal moments of inertia.")
  .def("valid_moments",
       &Class::ValidMoments,
       py::arg("_tolerance") = 0.1,
       "Verify that principal moments are positive")
  .def("is_valid",
       &Class::IsValid,
       py::arg("_tolerance") = IGN_MASSMATRIX3_DEFAULT_TOLERANCE<T>,
       "Verify that inertia values are positive semi-definite "
       "and satisfy the triangle inequality.")
  .def("epsilon",
       py::overload_cast<const ignition::math::Vector3<T>&, const T>
        (&Class::Epsilon),
       py::arg("_tolerance") = IGN_MASSMATRIX3_DEFAULT_TOLERANCE<T>,
       "Get an epsilon value that represents the amount of "
       "acceptable error in a MassMatrix3. The epsilon value "
       "is related to machine precision multiplied by the largest possible "
       "moment of inertia.")
  .def("is_positive",
       &Class::IsPositive,
       py::arg("_tolerance") = IGN_MASSMATRIX3_DEFAULT_TOLERANCE<T>,
       "Verify that inertia values are positive definite")
  .def("is_near_positive",
       &Class::IsNearPositive,
       py::arg("_tolerance") = IGN_MASSMATRIX3_DEFAULT_TOLERANCE<T>,
       "Verify that inertia values are positive semidefinite")
  .def(py::self != py::self)
  .def(py::self == py::self);
}
}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__MASSMATRIX3_HH_

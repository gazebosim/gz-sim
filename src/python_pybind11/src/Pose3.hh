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

#ifndef IGNITION_MATH_PYTHON__POSE3_HH_
#define IGNITION_MATH_PYTHON__POSE3_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ignition/math/Pose3.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::Pose3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathPose3(py::module &m, const std::string &typestr);

/// Help define a pybind11 wrapper for an ignition::math::Pose3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathPose3(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Pose3<T>;
  auto toString = [](const Class &si) {
    std::stringstream stream;
    stream << si;
    return stream.str();
  };
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<const ignition::math::Vector3<T>&,
                  const ignition::math::Quaternion<T>&>())
    .def(py::init<T, T, T, T, T, T>())
    .def(py::init<T, T, T, T, T, T, T>())
    .def(py::init<const Class&>())
    .def(py::self + py::self)
    .def(py::self += py::self)
    .def(-py::self)
    .def(py::self - py::self)
    .def(py::self -= py::self)
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self * py::self)
    .def(py::self *= py::self)
    .def("set",
         py::overload_cast<const ignition::math::Vector3<T>&,
                           const ignition::math::Quaternion<T>&>(&Class::Set),
         "Set the pose from a Vector3 and a Quaternion<T>")
    .def("set",
         py::overload_cast<const ignition::math::Vector3<T>&,
                           const ignition::math::Vector3<T>&>(&Class::Set),
         "Set the pose from  pos and rpy vectors")
    .def("set",
        py::overload_cast<T, T, T, T, T, T>(&Class::Set),
        "Set the pose from a six tuple.")
    .def("is_finite",
         &Class::IsFinite,
         "See if a pose is finite (e.g., not nan)")
    .def("correct",
         &Class::Correct,
         "Fix any nan values")
    .def("inverse",
         &Class::Inverse,
         "Get the inverse of this pose")
    .def("coord_position_add",
         py::overload_cast<const ignition::math::Vector3<T>&>(
           &Class::CoordPositionAdd, py::const_),
         "Add one point to a vector: result = this + pos")
    .def("coord_position_add",
         py::overload_cast<const Class&>(
           &Class::CoordPositionAdd, py::const_),
         "Add one point to another: result = this + pose")
    .def("coord_position_sub",
         &Class::CoordPositionSub,
         "Subtract one position from another: result = this - pose")
    .def("coord_rotation_add",
         &Class::CoordRotationAdd,
         "Add one rotation to another: result =  this->q + rot")
    .def("coord_rotation_sub",
         &Class::CoordRotationSub,
         "Subtract one rotation from another: result = this->q - rot")
    .def("coord_pose_solve",
         &Class::CoordPoseSolve,
         "Find the inverse of a pose; i.e., if b = this + a, given b and "
         "this, find a")
    .def("reset", &Class::Reset, "Reset the pose")
    .def("rotate_position_about_origin",
         &Class::RotatePositionAboutOrigin,
         "Rotate vector part of a pose about the origin")
    .def("round",
         &Class::Round,
         "Round all values to _precision decimal places")
    .def("pos",
         py::overload_cast<>(&Class::Pos, py::const_),
         py::return_value_policy::reference,
         "Get the position.")
    .def("x", &Class::X, "Get the X value of the position")
    .def("y", &Class::Y, "Get the Y value of the position")
    .def("z", &Class::Z, "Get the Z value of the position")
    .def("set_x", &Class::SetX, "Set the X value of the position")
    .def("set_y", &Class::SetY, "Set the Y value of the position")
    .def("set_z", &Class::SetZ, "Set the Z value of the position")
    .def("rot",
         py::overload_cast<>(&Class::Rot, py::const_),
         py::return_value_policy::reference,
         "Get the rotation.")
    .def("roll", &Class::Roll, "Get the Roll value of the position")
    .def("pitch", &Class::Pitch, "Get the Pitch value of the position")
    .def("yaw", &Class::Yaw, "Get the Yaw value of the position")
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a)
    .def_readonly_static("ZERO", &Class::Zero, "Zero matrix")
    .def("__str__", toString)
    .def("__repr__", toString);
}
}  // namespace python
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__POSE3_HH_

// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>

#include <ignition/math/Pose3.hh>
#include <ignition/gazebo/Conversions.hh>

#include "pose_v.hh"

namespace ignition
{
namespace msgs
{
namespace python
{
void
PoseV::destroy()
{
  // msgs_.reset();
}

PoseV::~PoseV()
{

}

PoseV::PoseV()
{

}

PoseV::PoseV(const ignition::msgs::Pose_V &_msg)
{
  this->msgs_ = _msg;
}

void PoseV::show()
{
  for (unsigned int i = 0; i < this->msgs_.pose_size(); ++i)
  {
    ignition::math::Pose3d pose = ignition::gazebo::convert<ignition::math::Pose3d>(this->msgs_.pose(i));
    std::cout << "Name: " << this->msgs_.pose(i).name() << "\tID: " << this->msgs_.pose(i).id();
    std::cout << "\tPose: " << pose.Pos();
    std::cout << "\tOrientation: " << pose.Rot() << '\n';
  }
}

std::vector<std::string> PoseV::GetNames()
{
  std::vector<std::string> result;
  for (int i = 0; i < this->msgs_.pose_size(); ++i)
  {
    result.emplace_back(this->msgs_.pose(i).name());
  }

  return result;
}

int PoseV::GetSize()
{
  return this->msgs_.pose_size();
}

py::object PoseV::Pose()
{
  PyObject *m, *d, *md, *globals;

#if PY_VERSION_HEX >= 0x03000000
  static struct PyModuleDef SWIG_module = {
    PyModuleDef_HEAD_INIT,
    SWIG_name,
    NULL,
    -1,
    SwigMethods,
    NULL,
    NULL,
    NULL,
    NULL
  };
#endif

#if defined(SWIGPYTHON_BUILTIN)
  static SwigPyClientData SwigPyObject_clientdata = {
    0, 0, 0, 0, 0, 0, 0
  };
  static PyGetSetDef this_getset_def = {
    (char *)"this", &SwigPyBuiltin_ThisClosure, NULL, NULL, NULL
  };
  static SwigPyGetSet thisown_getset_closure = {
    SwigPyObject_own,
    SwigPyObject_own
  };
  static PyGetSetDef thisown_getset_def = {
    (char *)"thisown", SwigPyBuiltin_GetterClosure, SwigPyBuiltin_SetterClosure, NULL, &thisown_getset_closure
  };
  PyTypeObject *builtin_pytype;
  int builtin_base_count;
  swig_type_info *builtin_basetype;
  PyObject *tuple;
  PyGetSetDescrObject *static_getset;
  PyTypeObject *metatype;
  PyTypeObject *swigpyobject;
  SwigPyClientData *cd;
  PyObject *public_interface, *public_symbol;
  PyObject *this_descr;
  PyObject *thisown_descr;
  PyObject *self = 0;
  int i;

  (void)builtin_pytype;
  (void)builtin_base_count;
  (void)builtin_basetype;
  (void)tuple;
  (void)static_getset;
  (void)self;

  /* Metaclass is used to implement static member variables */
  metatype = SwigPyObjectType();
  assert(metatype);
#endif

  (void)globals;

  /* Create singletons now to avoid potential deadlocks with multi-threaded usage after module initialization */
  SWIG_This();
  SWIG_Python_TypeCache();
  SwigPyPacked_type();
#ifndef SWIGPYTHON_BUILTIN
  SwigPyObject_type();
#endif

  /* Fix SwigMethods to carry the callback ptrs when needed */
  SWIG_Python_FixMethods(SwigMethods, swig_const_table, swig_types, swig_type_initial);

// globals = SWIG_globals();

// SwigPyObject_type();
//
// /* Fix SwigMethods to carry the callback ptrs when needed */
// SWIG_Python_FixMethods(SwigMethods, swig_const_table, swig_types, swig_type_initial);

#if PY_VERSION_HEX >= 0x03000000
  m = PyModule_Create(&SWIG_module);
#else
  m = Py_InitModule(SWIG_name, SwigMethods);
#endif

  md = d = PyModule_GetDict(m);
  (void)md;

  SWIG_InitializeModule(0);

  ignition::math::Vector3d result(-5, 6, 4);
  PyObject * resultobj = SWIG_NewPointerObj((new ignition::math::Vector3< double >(static_cast< const ignition::math::Vector3< double >& >(result))), SWIGTYPE_p_ignition__math__Vector3T_double_t, SWIG_POINTER_OWN |  0 );

  return py::reinterpret_borrow<py::object>(resultobj);
}

void PoseV::SetPose(py::object _v)
{
  ignition::math::Vector3< double > *arg1;
  void *argp1 = 0 ;
  int res1 = 0 ;

  py::handle h = _v;

  res1 = SWIG_ConvertPtr(h.ptr(), &argp1, SWIGTYPE_p_ignition__math__Vector3T_double_t, 0 |  0 );
  if (!SWIG_IsOK(res1)) {
    SWIG_exception_fail(SWIG_ArgError(res1), "in method '" "Vector3d_set" "', argument " "1"" of type '" "ignition::math::Vector3< double > *""'");
  }
  arg1 = reinterpret_cast< ignition::math::Vector3< double > * >(argp1);

  std::cout << "_v " << *arg1 << '\n';
fail:
  return;
}

std::vector<ignition::math::Vector3d> PoseV::GetPoses()
{
  std::vector<ignition::math::Vector3d> result;
  for (int i = 0; i < this->msgs_.pose_size(); ++i)
  {
    ignition::math::Pose3d pose = ignition::gazebo::convert<ignition::math::Pose3d>(this->msgs_.pose(i));
    result.emplace_back(pose.Pos());
  }

  return result;
}

void define_msgs_pose_v(py::object module)
{
  py::class_<PoseV, ignition::utils::python::Destroyable, std::shared_ptr<PoseV>>(module, "PoseV")
  .def(py::init<const ignition::msgs::Pose_V &>())
  .def(py::init<>())
  .def(
    "pose", &PoseV::Pose,
    "pose")
  .def(
    "set_pose", &PoseV::SetPose,
    "set pose")
  .def(
    "show", &PoseV::show,
    "Run server")
  .def(
    "get_size", &PoseV::GetSize,
    "Get size")
  .def(
    "get_poses", &PoseV::GetPoses,
    "Get poses")
  .def(
    "get_names", &PoseV::GetNames,
    "Get names");
}
}  // namespace python
}  // namespace gazebo
}  // namespace ignition

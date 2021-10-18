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

#ifndef IGNITION_GAZEBO_PYTHON__POSE_V_HPP_
#define IGNITION_GAZEBO_PYTHON__POSE_V_HPP_

#include <pybind11/pybind11.h>

#include <ignition/msgs/pose_v.pb.h>

#include <ignition/math/Vector3.hh>

#include "../utils/destroyable.hh"

namespace py = pybind11;

namespace ignition
{
namespace msgs
{
namespace python
{

typedef void *(*swig_converter_func)(void *, int *);
typedef struct swig_type_info *(*swig_dycast_func)(void **);
/* Structure to store information on one type */
typedef struct swig_type_info {
  const char             *name;			/* mangled name of this type */
  const char             *str;			/* human readable name of this type */
  swig_dycast_func        dcast;		/* dynamic cast function down a hierarchy */
  struct swig_cast_info  *cast;			/* linked list of types that can cast into this type */
  void                   *clientdata;		/* language specific type data */
  int                    owndata;		/* flag if the structure owns the clientdata */
} swig_type_info;

/* Structure to store a type and conversion function used for casting */
typedef struct swig_cast_info {
  swig_type_info         *type;			/* pointer to type that is equivalent to this type */
  swig_converter_func     converter;		/* function to cast the void pointers */
  struct swig_cast_info  *next;			/* pointer to next cast in linked list */
  struct swig_cast_info  *prev;			/* pointer to the previous cast */
} swig_cast_info;


typedef struct {
  PyObject_HEAD
  void *ptr;
  swig_type_info *ty;
  int own;
  PyObject *next;
#ifdef SWIGPYTHON_BUILTIN
  PyObject *dict;
#endif
} SwigPyObject;

typedef struct {
  PyObject *klass;
  PyObject *newraw;
  PyObject *newargs;
  PyObject *destroy;
  int delargs;
  int implicitconv;
  PyTypeObject *pytype;
} SwigPyClientData;
#define SWIG_POINTER_OWN           0x1
#define SWIG_Python_CallFunctor(functor, obj)	        PyObject_CallFunctionObjArgs(functor, obj, NULL);
PyObject * SwigPyObject_richcompare(SwigPyObject *v, SwigPyObject *w, int op);

#ifdef __cplusplus
#define SWIG_STATIC_POINTER(var)  var
#else
#define SWIG_STATIC_POINTER(var)  var = 0; if (!var) var
#endif

PyObject * SWIG_Py_Void(void);

PyTypeObject * SwigPyObject_type(void);

PyObject *
SwigPyObject_New(void *ptr, swig_type_info *ty, int own);

PyObject*
SwigPyObject_append(PyObject* v, PyObject* next);

PyObject*
SwigPyObject_disown(PyObject* v, PyObject *args);

PyObject*
SwigPyObject_next(PyObject* v, PyObject *args);

PyObject*
SwigPyObject_acquire(PyObject* v, PyObject *args);

PyObject *
SwigPyObject_repr2(PyObject *v, PyObject *args);

PyObject*
SwigPyObject_own(PyObject *v, PyObject *args);

int SwigPyObject_Check(PyObject *op);

static PyMethodDef
swigobject_methods[] = {
  {"disown",  SwigPyObject_disown,  METH_NOARGS,  "releases ownership of the pointer"},
  {"acquire", SwigPyObject_acquire, METH_NOARGS,  "acquires ownership of the pointer"},
  {"own",     SwigPyObject_own,     METH_VARARGS, "returns/sets ownership of the pointer"},
  {"append",  SwigPyObject_append,  METH_O,       "appends another 'this' object"},
  {"next",    SwigPyObject_next,    METH_NOARGS,  "returns the next 'this' object"},
  {"__repr__",SwigPyObject_repr2,   METH_NOARGS,  "returns object representation"},
  {0, 0, 0, 0}
};

/* TODO: I don't know how to implement the fast getset in Python 3 right now */
#if PY_VERSION_HEX>=0x03000000
#define SWIG_PYTHON_SLOW_GETSET_THIS
#endif

class PoseV : public ignition::utils::python::Destroyable, public std::enable_shared_from_this<PoseV>
{
public:
  PoseV(const ignition::msgs::Pose_V &_msg);
  PoseV();

  ~PoseV();

  void show();

  std::vector<std::string> GetNames();
  int GetSize();

  void SetPose(py::object _pyobject);

  py::list Pose();

  /// Force an early destruction of this object
  void
  destroy() override;

private:
  ignition::msgs::Pose_V msgs_;
};

/// Define a pybind11 wrapper for an ignition::gazebo::Node
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_msgs_pose_v(py::object module);
}
}
}

#endif

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

#if PY_VERSION_HEX >= 0x03000000
#  define SWIG_Python_str_FromFormat PyUnicode_FromFormat
#else
#  define SWIG_Python_str_FromFormat PyString_FromFormat
#endif

/*
  Return the pretty name associated with this type,
  that is an unmangled type name in a form presentable to the user.
*/
const char *
SWIG_TypePrettyName(const swig_type_info *type) {
  /* The "str" field contains the equivalent pretty names of the
     type, separated by vertical-bar characters.  We choose
     to print the last name, as it is often (?) the most
     specific. */
  if (!type) return NULL;
  if (type->str != NULL) {
    const char *last_name = type->str;
    const char *s;
    for (s = type->str; *s; s++)
      if (*s == '|') last_name = s+1;
    return last_name;
  }
  else
    return type->name;
}

PyObject *
SwigPyObject_New(void *ptr, swig_type_info *ty, int own)
{
  SwigPyObject *sobj = PyObject_NEW(SwigPyObject, SwigPyObject_type());
  if (sobj) {
    sobj->ptr  = ptr;
    sobj->ty   = ty;
    sobj->own  = own;
    sobj->next = 0;
  }
  return (PyObject *)sobj;
}

int
SwigPyObject_compare(SwigPyObject *v, SwigPyObject *w)
{
  void *i = v->ptr;
  void *j = w->ptr;
  return (i < j) ? -1 : ((i > j) ? 1 : 0);
}

PyObject*
SwigPyObject_richcompare(SwigPyObject *v, SwigPyObject *w, int op)
{
  PyObject* res;
  if( op != Py_EQ && op != Py_NE ) {
    Py_INCREF(Py_NotImplemented);
    return Py_NotImplemented;
  }
  res = PyBool_FromLong( (SwigPyObject_compare(v, w)==0) == (op == Py_EQ) ? 1 : 0);
  return res;
}

void
SwigPyObject_dealloc(PyObject *v)
{
  SwigPyObject *sobj = (SwigPyObject *) v;
  PyObject *next = sobj->next;
  if (sobj->own == SWIG_POINTER_OWN) {
    swig_type_info *ty = sobj->ty;
    SwigPyClientData *data = ty ? (SwigPyClientData *) ty->clientdata : 0;
    PyObject *destroy = data ? data->destroy : 0;
    if (destroy) {
      /* destroy is always a VARARGS method */
      PyObject *res;

      /* PyObject_CallFunction() has the potential to silently drop
         the active exception.  In cases of unnamed temporary
         variable or where we just finished iterating over a generator
         StopIteration will be active right now, and this needs to
         remain true upon return from SwigPyObject_dealloc.  So save
         and restore. */

      PyObject *type = NULL, *value = NULL, *traceback = NULL;
      PyErr_Fetch(&type, &value, &traceback);

      if (data->delargs) {
        /* we need to create a temporary object to carry the destroy operation */
        PyObject *tmp = SwigPyObject_New(sobj->ptr, ty, 0);
        res = SWIG_Python_CallFunctor(destroy, tmp);
        Py_DECREF(tmp);
      } else {
        PyCFunction meth = PyCFunction_GET_FUNCTION(destroy);
        PyObject *mself = PyCFunction_GET_SELF(destroy);
        res = ((*meth)(mself, v));
      }
      if (!res)
        PyErr_WriteUnraisable(destroy);

      PyErr_Restore(type, value, traceback);

      Py_XDECREF(res);
    }
#if !defined(SWIG_PYTHON_SILENT_MEMLEAK)
    else {
      const char *name = SWIG_TypePrettyName(ty);
      printf("swig/python detected a memory leak of type '%s', no destructor found.\n", (name ? name : "unknown"));
    }
#endif
  }
  Py_XDECREF(next);
  PyObject_DEL(v);
}

PyObject *
SwigPyObject_repr(SwigPyObject *v)
{
  const char *name = SWIG_TypePrettyName(v->ty);
  PyObject *repr = SWIG_Python_str_FromFormat("<Swig Object of type '%s' at %p>", (name ? name : "unknown"), (void *)v);
  if (v->next) {
    PyObject *nrep = SwigPyObject_repr((SwigPyObject *)v->next);
# if PY_VERSION_HEX >= 0x03000000
    PyObject *joined = PyUnicode_Concat(repr, nrep);
    Py_DecRef(repr);
    Py_DecRef(nrep);
    repr = joined;
# else
    PyString_ConcatAndDel(&repr,nrep);
# endif
  }
  return repr;
}

PyTypeObject*
SwigPyObject_TypeOnce(void) {
  static char swigobject_doc[] = "Swig object carries a C/C++ instance pointer";

  static PyNumberMethods SwigPyObject_as_number = {
    (binaryfunc)0, /*nb_add*/
    (binaryfunc)0, /*nb_subtract*/
    (binaryfunc)0, /*nb_multiply*/
    /* nb_divide removed in Python 3 */
#if PY_VERSION_HEX < 0x03000000
    (binaryfunc)0, /*nb_divide*/
#endif
    (binaryfunc)0, /*nb_remainder*/
    (binaryfunc)0, /*nb_divmod*/
    (ternaryfunc)0,/*nb_power*/
    (unaryfunc)0,  /*nb_negative*/
    (unaryfunc)0,  /*nb_positive*/
    (unaryfunc)0,  /*nb_absolute*/
    (inquiry)0,    /*nb_nonzero*/
    0,		   /*nb_invert*/
    0,		   /*nb_lshift*/
    0,		   /*nb_rshift*/
    0,		   /*nb_and*/
    0,		   /*nb_xor*/
    0,		   /*nb_or*/
#if PY_VERSION_HEX < 0x03000000
    0,   /*nb_coerce*/
#endif
    (unaryfunc)PyLong_FromVoidPtr, /*nb_int*/
#if PY_VERSION_HEX < 0x03000000
    (unaryfunc)PyLong_FromVoidPtr, /*nb_long*/
#else
    0, /*nb_reserved*/
#endif
    (unaryfunc)0,                 /*nb_float*/
#if PY_VERSION_HEX < 0x03000000
    (unaryfunc)SwigPyObject_oct,  /*nb_oct*/
    (unaryfunc)SwigPyObject_hex,  /*nb_hex*/
#endif
#if PY_VERSION_HEX >= 0x03050000 /* 3.5 */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 /* nb_inplace_add -> nb_inplace_matrix_multiply */
#elif PY_VERSION_HEX >= 0x03000000 /* 3.0 */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 /* nb_inplace_add -> nb_index, nb_inplace_divide removed */
#else
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 /* nb_inplace_add -> nb_index */
#endif
  };

  static PyTypeObject swigpyobject_type;
  static int type_init = 0;
  if (!type_init) {
    const PyTypeObject tmp = {
#if PY_VERSION_HEX >= 0x03000000
      PyVarObject_HEAD_INIT(NULL, 0)
#else
      PyObject_HEAD_INIT(NULL)
      0,                                    /* ob_size */
#endif
      "SwigPyObject",                       /* tp_name */
      sizeof(SwigPyObject),                 /* tp_basicsize */
      0,                                    /* tp_itemsize */
      (destructor)SwigPyObject_dealloc,     /* tp_dealloc */
      0,                                    /* tp_print */
      (getattrfunc)0,                       /* tp_getattr */
      (setattrfunc)0,                       /* tp_setattr */
#if PY_VERSION_HEX >= 0x03000000
      0, /* tp_reserved in 3.0.1, tp_compare in 3.0.0 but not used */
#else
      (cmpfunc)SwigPyObject_compare,        /* tp_compare */
#endif
      (reprfunc)SwigPyObject_repr,          /* tp_repr */
      &SwigPyObject_as_number,              /* tp_as_number */
      0,                                    /* tp_as_sequence */
      0,                                    /* tp_as_mapping */
      (hashfunc)0,                          /* tp_hash */
      (ternaryfunc)0,                       /* tp_call */
      0,                                    /* tp_str */
      PyObject_GenericGetAttr,              /* tp_getattro */
      0,                                    /* tp_setattro */
      0,                                    /* tp_as_buffer */
      Py_TPFLAGS_DEFAULT,                   /* tp_flags */
      swigobject_doc,                       /* tp_doc */
      0,                                    /* tp_traverse */
      0,                                    /* tp_clear */
      (richcmpfunc)SwigPyObject_richcompare,/* tp_richcompare */
      0,                                    /* tp_weaklistoffset */
      0,                                    /* tp_iter */
      0,                                    /* tp_iternext */
      swigobject_methods,                   /* tp_methods */
      0,                                    /* tp_members */
      0,                                    /* tp_getset */
      0,                                    /* tp_base */
      0,                                    /* tp_dict */
      0,                                    /* tp_descr_get */
      0,                                    /* tp_descr_set */
      0,                                    /* tp_dictoffset */
      0,                                    /* tp_init */
      0,                                    /* tp_alloc */
      0,                                    /* tp_new */
      0,                                    /* tp_free */
      0,                                    /* tp_is_gc */
      0,                                    /* tp_bases */
      0,                                    /* tp_mro */
      0,                                    /* tp_cache */
      0,                                    /* tp_subclasses */
      0,                                    /* tp_weaklist */
      0,                                    /* tp_del */
      0,                                    /* tp_version_tag */
#if PY_VERSION_HEX >= 0x03040000
      0,                                    /* tp_finalize */
#endif
#ifdef COUNT_ALLOCS
      0,                                    /* tp_allocs */
      0,                                    /* tp_frees */
      0,                                    /* tp_maxalloc */
      0,                                    /* tp_prev */
      0                                     /* tp_next */
#endif
    };
    swigpyobject_type = tmp;
    type_init = 1;
    if (PyType_Ready(&swigpyobject_type) < 0)
      return NULL;
  }
  return &swigpyobject_type;
}

PyTypeObject*
SwigPyObject_type(void) {
  static PyTypeObject *SWIG_STATIC_POINTER(type) = SwigPyObject_TypeOnce();
  return type;
}

/* We need a version taking two PyObject* parameters so it's a valid
 * PyCFunction to use in swigobject_methods[]. */
PyObject *
SwigPyObject_repr2(PyObject *v, PyObject *args)
{
  return SwigPyObject_repr((SwigPyObject*)v);
}

PyObject *
SWIG_Py_Void(void)
{
  PyObject *none = Py_None;
  Py_INCREF(none);
  return none;
}

PyObject*
SwigPyObject_append(PyObject* v, PyObject* next)
{
  SwigPyObject *sobj = (SwigPyObject *) v;
  if (!SwigPyObject_Check(next)) {
    PyErr_SetString(PyExc_TypeError, "Attempt to append a non SwigPyObject");
    return NULL;
  }
  sobj->next = next;
  Py_INCREF(next);
  return SWIG_Py_Void();
}

PyObject*
SwigPyObject_next(PyObject* v, PyObject *args)
{
  SwigPyObject *sobj = (SwigPyObject *) v;
  if (sobj->next) {
    Py_INCREF(sobj->next);
    return sobj->next;
  } else {
    return SWIG_Py_Void();
  }
}

PyObject*
SwigPyObject_disown(PyObject* v, PyObject *args)
{
  SwigPyObject *sobj = (SwigPyObject *)v;
  sobj->own = 0;
  return SWIG_Py_Void();
}

PyObject*
SwigPyObject_acquire(PyObject* v, PyObject *args)
{
  SwigPyObject *sobj = (SwigPyObject *)v;
  sobj->own = SWIG_POINTER_OWN;
  return SWIG_Py_Void();
}

PyObject*
SwigPyObject_own(PyObject *v, PyObject *args)
{
  PyObject *val = 0;
  if (!PyArg_UnpackTuple(args, "own", 0, 1, &val)) {
    return NULL;
  } else {
    SwigPyObject *sobj = (SwigPyObject *)v;
    PyObject *obj = PyBool_FromLong(sobj->own);
    if (val) {
      if (PyObject_IsTrue(val)) {
        SwigPyObject_acquire(v,args);
      } else {
        SwigPyObject_disown(v,args);
      }
    }
    return obj;
  }
}
static PyObject *Swig_This_global = NULL;

PyObject*
SWIG_Python_str_FromChar(const char *c)
{
#if PY_VERSION_HEX >= 0x03000000
  return PyUnicode_FromString(c);
#else
  return PyString_FromString(c);
#endif
}

PyObject *
SWIG_This(void)
{
 if (Swig_This_global == NULL)
   Swig_This_global = SWIG_Python_str_FromChar("this");
 return Swig_This_global;
}

int
SwigPyObject_Check(PyObject *op) {
  return (Py_TYPE(op) == SwigPyObject_type())
    || (strcmp(Py_TYPE(op)->tp_name,"SwigPyObject") == 0);
}

SwigPyObject *
SWIG_Python_GetSwigThis(PyObject *pyobj)
{
  PyObject *obj;

  if (SwigPyObject_Check(pyobj)){
    std::cerr << "return 1" << '\n';
    return (SwigPyObject *) pyobj;
  }

#ifdef SWIGPYTHON_BUILTIN
  (void)obj;
# ifdef PyWeakref_CheckProxy
  if (PyWeakref_CheckProxy(pyobj)) {
    pyobj = PyWeakref_GET_OBJECT(pyobj);
    if (pyobj && SwigPyObject_Check(pyobj))
      return (SwigPyObject*) pyobj;
  }
# endif
  std::cerr << "return 2" << '\n';

  return NULL;
#else

  obj = 0;

#if !defined(SWIG_PYTHON_SLOW_GETSET_THIS)
  if (PyInstance_Check(pyobj)) {
    obj = _PyInstance_Lookup(pyobj, SWIG_This());
  } else {
    PyObject **dictptr = _PyObject_GetDictPtr(pyobj);
    if (dictptr != NULL) {
      PyObject *dict = *dictptr;
      obj = dict ? PyDict_GetItem(dict, SWIG_This()) : 0;
    } else {
#ifdef PyWeakref_CheckProxy
      if (PyWeakref_CheckProxy(pyobj)) {
      	PyObject *wobj = PyWeakref_GET_OBJECT(pyobj);
        std::cerr << "return 3" << '\n';
      	return wobj ? SWIG_Python_GetSwigThis(wobj) : 0;
      }
#endif
      obj = PyObject_GetAttr(pyobj,SWIG_This());
      if (obj) {
	       Py_DECREF(obj);
      } else {
      	if (PyErr_Occurred()) PyErr_Clear();
        std::cerr << "return 4" << '\n';

      	return 0;
      }
    }
  }
#else
  obj = PyObject_GetAttr(pyobj,SWIG_This());
  if (obj) {
    Py_DECREF(obj);
  } else {
    if (PyErr_Occurred()) PyErr_Clear();
    std::cerr << "return 5" << '\n';

    return 0;
  }
#endif
  if (obj && !SwigPyObject_Check(obj)) {
    /* a PyObject is called 'this', try to get the 'real this'
       SwigPyObject from it */
       std::cerr << "return 6" << '\n';

    return SWIG_Python_GetSwigThis(obj);
  }
  std::cerr << "return 7" << '\n';

  return (SwigPyObject *)obj;
#endif
}

/*
  Cast a pointer up an inheritance hierarchy
*/
void *
SWIG_TypeCast(swig_cast_info *ty, void *ptr, int *newmemory) {
  return ((!ty) || (!ty->converter)) ? ptr : (*ty->converter)(ptr, newmemory);
}

void PoseV::SetPose(py::object _pyobject)
{
  py::handle h = _pyobject;
  PyObject * pyobj = h.ptr();

  SwigPyObject * swig_kk;

  PyObject *obj = nullptr;

  std::cerr << "op->ob_type->tp_name " << pyobj->ob_type->tp_name << "\n";

  swig_kk = SWIG_Python_GetSwigThis(pyobj);

  if (swig_kk)
  {
    std::cerr << "ok!" << '\n';
  }
  else
  {
    std::cerr << "kk" << '\n';
  }

  int newmemory = 0;

  void *ptr = SWIG_TypeCast(swig_kk->ty->cast, swig_kk->ptr, &newmemory);

  ignition::math::Vector3d * v = reinterpret_cast<ignition::math::Vector3d *>(ptr);
  std::cerr << "v " << *v << '\n';
}

py::list PoseV::Pose()
{
//   ignition::math::Vector3< double > *result = 0 ;
//   result = (ignition::math::Vector3< double > *)new ignition::math::Vector3< double >();
//   void * result2 = (void *)new ignition::math::Vector3< double >();
//
// // 	 { "Angle_swigregister", Angle_swigregister, METH_O, NULL},
//
//
//   swig_type_info _swigt__p_ignition__math__Vector3T_double_t = {
//     "_p_ignition__math__Vector3T_double_t",
//     "std::vector< ignition::math::Vector3< double > >::value_type *|std::set< ignition::math::Vector3< double >,ignition::math::WellOrderedVectors< double > >::value_type *|std::set< ignition::math::Vector3< double >,ignition::math::WellOrderedVectors< double > >::key_type *|ignition::math::Vector3< double > *",
//     0,
//     0,
//     (void*)0,
//     0};
// //SwigPyClientData_New SWIG_TypeNewClientData
// // SWIG_NewClientData
// // SWIG_Python_UnpackTuple
//
//   PyObject * pyobj = SWIG_Python_NewPointerObj(
//     result2,
//     &_swigt__p_ignition__math__Vector3T_double_t,
//     0);
//   std::cerr << "return pose" << '\n';
//   py::handle h = pyobj;
//
//   return py::reinterpret_borrow<py::object>(h);

  ignition::math::Vector3< double > result;
  py::list li;
  li.append(result.X());
  li.append(result.Y());
  li.append(result.Z());

  return li;
}

void define_msgs_pose_v(py::object module)
{
  py::class_<PoseV, ignition::utils::python::Destroyable, std::shared_ptr<PoseV>>(module, "PoseV")
  .def(py::init<const ignition::msgs::Pose_V &>())
  .def(py::init<>())
  .def(
    "show", &PoseV::show,
    "Run server")
  .def(
    "pose", &PoseV::Pose,
    "Get pose")
  .def(
    "set_pose", &PoseV::SetPose,
    "Set pose")
  .def(
    "get_size", &PoseV::GetSize,
    "Get size")
  .def(
    "get_names", &PoseV::GetNames,
    "Get names");
}
}  // namespace python
}  // namespace gazebo
}  // namespace ignition

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

#ifndef IGNITION_MATH_PYTHON__FILTER_HH_
#define IGNITION_MATH_PYTHON__FILTER_HH_

#include <string>

#include <pybind11/pybind11.h>

#include <ignition/math/Filter.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

namespace py = pybind11;

namespace ignition
{
namespace math
{
namespace python
{
template<typename T>
class FilterTrampoline : public Filter<T> {
public:
    // Inherit the constructors
    FilterTrampoline() : Filter<T>() {}

    // Trampoline (need one for each virtual function)
    void Set(const T &_val) override
    {
        PYBIND11_OVERLOAD_PURE(
            void,        // Return type (ret_type)
            Filter<T>,   // Parent class (cname)
            Set,         // Name of function in C++
            _val         // Argument(s) (...)
        );
    }
    // Trampoline (need one for each virtual function)
    void Fc(double _fc, double _fs) override
    {
        PYBIND11_OVERLOAD_PURE(
            void,         // Return type (ret_type)
            Filter<T>,    // Parent class (cname)
            Fc,           // Name of function in C++ (must match Python name)
            _fc, _fs      // Argument(s) (...)
        );
    }
    // Trampoline (need one for each virtual function)
    const T &Value() const override
    {
        PYBIND11_OVERLOAD_PURE(
            const T&,     // Return type (ret_type)
            Filter<T>,    // Parent class (cname)
            Value         // Name of function in C++ (must match Python name)
        );
    }
};

/// Help define a pybind11 wrapper for an ignition::math::Filter
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathFilter(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Filter<T>;
  std::string pyclass_name = typestr;
  py::class_<Class, FilterTrampoline<T>>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def("set",
         &Class::Set,
         "Set the output of the filter.")
    .def("fc",
         &Class::Fc,
         "Set the cutoff frequency and sample rate.")
    .def("value",
         &Class::Value,
         "Get the output of the filter.");
}

/// Define a pybind11 wrapper for an ignition::math::Filter
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathFilter(py::module &m, const std::string &typestr);

template<typename T>
class OnePoleTrampoline : public OnePole<T> {
public:
  OnePoleTrampoline() : OnePole<T>() {}
  OnePoleTrampoline(double _fc, double _fs) : OnePole<T>(_fc, _fs) {}

  // Trampoline (need one for each virtual function)
  void Fc(double _fc, double _fs) override
  {
      PYBIND11_OVERLOAD_PURE(
          void,         // Return type (ret_type)
          OnePole<T>,   // Parent class (cname)
          Fc,           // Name of function in C++ (must match Python name)
          _fc,          // Argument(s) (...)
          _fs
      );
  }
};

/// Help define a pybind11 wrapper for an ignition::math::OnePole
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathOnePole(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::OnePole<T>;
  std::string pyclass_name = typestr;
  py::class_<Class, OnePoleTrampoline<T>>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<double, double>())
    .def("set",
         &Class::Set,
         "Set the output of the filter.")
    .def("value",
         &Class::Value,
         "Get the output of the filter.")
    .def("fc",
         &Class::Fc,
         "Set the cutoff frequency and sample rate.")
    .def("process",
         &Class::Process,
         "Update the filter's output.");
}

/// Define a pybind11 wrapper for an ignition::math::OnePole
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathOnePole(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::OnePoleQuaterion
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathOnePoleQuaternion(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::OnePoleVector3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathOnePoleVector3(py::module &m, const std::string &typestr);

template<typename T>
class BiQuadTrampoline : public BiQuad<T>
{
public:
  BiQuadTrampoline() : BiQuad<T>() {}
  BiQuadTrampoline(double _fc, double _fs) : BiQuad<T>(_fc, _fs) {}

  // Trampoline (need one for each virtual function)
  void Fc(double _fc, double _fs) override
  {
      PYBIND11_OVERLOAD_PURE(
          void,         // Return type (ret_type)
          Filter<T>,    // Parent class (cname)
          Fc,           // Name of function in C++ (must match Python name)
          _fc, _fs      // Argument(s) (...)
      );
  }
  // Trampoline (need one for each virtual function)
  void Set(const T &_val) override
  {
      PYBIND11_OVERLOAD_PURE(
          void,         // Return type (ret_type)
          Filter<T>,    // Parent class (cname)
          Set,          // Name of function in C++ (must match Python name)
          _val          // Argument(s) (...)
      );
  }
  // Trampoline (need one for each virtual function)
  const T& Process(const T &_x) override
  {
      PYBIND11_OVERLOAD_PURE(
          const T&,     // Return type (ret_type)
          Filter<T>,    // Parent class (cname)
          Process,      // Name of function in C++ (must match Python name)
          _x            // Argument(s) (...)
      );
  }
};

/// Help define a pybind11 wrapper for an ignition::math::BiQuad
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathBiQuad(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::BiQuad<T>;
  std::string pyclass_name = typestr;
  py::class_<Class, BiQuadTrampoline<T>>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<double, double>())
    .def("set",
         &Class::Set,
         "Set the output of the filter.")
    .def("fc",
          py::overload_cast<double, double>(&Class::Fc),
         "Set the cutoff frequency and sample rate.")
    .def("value",
         &Class::Value,
         "Get the output of the filter.")
    .def("fc",
         py::overload_cast<double, double, double>(&Class::Fc),
         "Set the cutoff frequency, sample rate and Q coefficient.")
    .def("process",
         &Class::Process,
         "Update the filter's output.");
}

/// Define a pybind11 wrapper for an ignition::math::BiQuad
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathBiQuad(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::BiQuadVector3
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathBiQuadVector3(py::module &m, const std::string &typestr);

}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__FILTER_HH_

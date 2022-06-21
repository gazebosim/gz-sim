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
#include <pybind11/stl.h>

#include <map>
#include <string>

#include "SignalStats.hh"
#include <gz/math/SignalStats.hh>

namespace gz
{
namespace math
{
namespace python
{
//////////////////////////////////////////////////
void defineMathSignalStats(py::module &m, const std::string &typestr)
{
  using Class = gz::math::SignalStats;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def(py::init<const Class&>())
  .def("count", &Class::Count, "Get number of data points in first statistic.")
  .def("reset", &Class::Reset, "Forget all previous data.")
  .def("map", &Class::Map,
       "Get the current values of each statistical measure, "
       "stored in a map using the short name as the key.")
  .def("insert_data",
       &Class::InsertData,
       "Add a new sample to the statistical measures.")
  .def("insert_statistic",
       &Class::InsertStatistic,
       "Add a new type of statistic.")
  .def("insert_statistics",
       &Class::InsertStatistics,
       "Add multiple statistics.");
}

//////////////////////////////////////////////////
class SignalStatisticTrampoline : public gz::math::SignalStatistic
{
public:
  SignalStatisticTrampoline() : SignalStatistic() {}
  explicit SignalStatisticTrampoline(const SignalStatistic &_ss) :
    SignalStatistic(_ss) {}

  // Trampoline (need one for each virtual function)
  double Value() const override
  {
      PYBIND11_OVERLOAD_PURE(
          double,                           // Return type (ret_type)
          gz::math::SignalStatistic,  // Parent class (cname)
          // Name of function in C++ (must match Python name) (fn)
          Value,
      );
  }
  // Trampoline (need one for each virtual function)
  std::string ShortName() const override
  {
      PYBIND11_OVERLOAD_PURE(
          std::string,                      // Return type (ret_type)
          gz::math::SignalStatistic,  // Parent class (cname)
          // Name of function in C++ (must match Python name) (fn)
          ShortName,
      );
  }
  // Trampoline (need one for each virtual function)
  size_t Count() const override
  {
      PYBIND11_OVERLOAD_PURE(
          size_t,                           // Return type (ret_type)
          gz::math::SignalStatistic,  // Parent class (cname)
          // Name of function in C++ (must match Python name) (fn)
          Count,
      );
  }
  // Trampoline (need one for each virtual function)
  void InsertData(const double _data) override
  {
      PYBIND11_OVERLOAD_PURE(
          void,                             // Return type (ret_type)
          gz::math::SignalStatistic,  // Parent class (cname)
          // Name of function in C++ (must match Python name) (fn)
          InsertData,
          _data                             // Argument(s) (...)
      );
  }
  // Trampoline (need one for each virtual function)
  void Reset() override
  {
      PYBIND11_OVERLOAD_PURE(
          void,                              // Return type (ret_type)
          gz::math::SignalStatistic,   // Parent class (cname)
          // Name of function in C++ (must match Python name) (fn)
          Reset,
      );
  }
};

//////////////////////////////////////////////////
void defineMathSignalStatistic(py::module &m, const std::string &typestr)
{
  using Class = gz::math::SignalStatistic;
  std::string pyclass_name = typestr;
  py::class_<Class, SignalStatisticTrampoline>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def(py::init<const Class&>())
  .def("value",
       &Class::Value,
       "Get a short version of the name of this statistical measure.")
  .def("short_name",
       &Class::ShortName,
       "Get a short version of the name of this statistical measure.")
  .def("count",
       &Class::Count,
       "Get number of data points in measurement.")
  .def("insert_data",
       &Class::InsertData,
       "Add a new sample to the statistical measure.")
  .def("reset",
       &Class::Reset,
       "Forget all previous data.");
}

//////////////////////////////////////////////////
class SignalVarianceTrampoline : public gz::math::SignalVariance {
public:
    // Inherit the constructors
    SignalVarianceTrampoline(){}

    // Trampoline (need one for each virtual function)
    double Value() const override
    {
        PYBIND11_OVERLOAD_PURE(
            double,                           // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            Value,
        );
    }
    // Trampoline (need one for each virtual function)
    std::string ShortName() const override
    {
        PYBIND11_OVERLOAD_PURE(
            std::string,                      // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            ShortName,
        );
    }
    // Trampoline (need one for each virtual function)
    void InsertData(const double _data) override
    {
        PYBIND11_OVERLOAD_PURE(
            void,                             // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            InsertData,
            _data                             // Argument(s) (...)
        );
    }
};

//////////////////////////////////////////////////
void defineMathSignalVariance(py::module &m, const std::string &typestr)
{
  using Class = gz::math::SignalVariance;
  std::string pyclass_name = typestr;
  py::class_<Class, SignalVarianceTrampoline>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def("reset",
       &Class::Reset,
       "Forget all previous data.")
  .def("count",
       &Class::Count,
       "Get number of data points in measurement.")
  .def("value",
       &Class::Value,
       "Get a short version of the name of this statistical measure.")
  .def("short_name",
       &Class::ShortName,
       "Get a short version of the name of this statistical measure.")
  .def("insert_data",
       &Class::InsertData,
       "Add a new sample to the statistical measure.");
}

//////////////////////////////////////////////////
class SignalMaximumTrampoline : public gz::math::SignalMaximum
{
public:
    // Inherit the constructors
    SignalMaximumTrampoline(){}

    // Trampoline (need one for each virtual function)
    double Value() const override
    {
        PYBIND11_OVERLOAD_PURE(
            double,                           // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            Value,
        );
    }
    // Trampoline (need one for each virtual function)
    std::string ShortName() const override
    {
        PYBIND11_OVERLOAD_PURE(
            std::string,                      // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            ShortName,
        );
    }
    // Trampoline (need one for each virtual function)
    void InsertData(const double _data) override
    {
        PYBIND11_OVERLOAD_PURE(
            void,                             // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            InsertData,
            _data                             // Argument(s) (...)
        );
    }
};

//////////////////////////////////////////////////
void defineMathSignalMaximum(py::module &m, const std::string &typestr)
{
  using Class = gz::math::SignalMaximum;
  std::string pyclass_name = typestr;
  py::class_<Class, SignalMaximumTrampoline>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def("reset",
       &Class::Reset,
       "Forget all previous data.")
  .def("count",
       &Class::Count,
       "Get number of data points in measurement.")
  .def("value",
       &Class::Value,
       "Get a short version of the name of this statistical measure.")
  .def("short_name",
       &Class::ShortName,
       "Get a short version of the name of this statistical measure.")
  .def("insert_data",
       &Class::InsertData,
       "Add a new sample to the statistical measure.");
}

//////////////////////////////////////////////////
class SignalMinimumTrampoline : public gz::math::SignalMinimum
{
public:
    // Inherit the constructors
    SignalMinimumTrampoline(){}

    // Trampoline (need one for each virtual function)
    double Value() const override
    {
        PYBIND11_OVERLOAD_PURE(
            double,                           // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            Value,
        );
    }
    // Trampoline (need one for each virtual function)
    std::string ShortName() const override
    {
        PYBIND11_OVERLOAD_PURE(
            std::string,                      // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            ShortName,
        );
    }
    // Trampoline (need one for each virtual function)
    void InsertData(const double _data) override
    {
        PYBIND11_OVERLOAD_PURE(
            void,                             // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            InsertData,
            _data                             // Argument(s) (...)
        );
    }
};

//////////////////////////////////////////////////
void defineMathSignalMinimum(py::module &m, const std::string &typestr)
{
  using Class = gz::math::SignalMinimum;
  std::string pyclass_name = typestr;
  py::class_<Class, SignalMinimumTrampoline>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def("reset",
       &Class::Reset,
       "Forget all previous data.")
  .def("count",
       &Class::Count,
       "Get number of data points in measurement.")
  .def("value",
       &Class::Value,
       "Get a short version of the name of this statistical measure.")
  .def("short_name",
       &Class::ShortName,
       "Get a short version of the name of this statistical measure.")
  .def("insert_data",
       &Class::InsertData,
       "Add a new sample to the statistical measure.");
}

//////////////////////////////////////////////////
class SignalMeanTrampoline : public gz::math::SignalMean
{
public:
    // Inherit the constructors
    SignalMeanTrampoline(){}

    // Trampoline (need one for each virtual function)
    double Value() const override
    {
        PYBIND11_OVERLOAD_PURE(
            double,                           // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            Value,
        );
    }
    // Trampoline (need one for each virtual function)
    std::string ShortName() const override
    {
        PYBIND11_OVERLOAD_PURE(
            std::string,                      // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            ShortName,
        );
    }
    // Trampoline (need one for each virtual function)
    void InsertData(const double _data) override
    {
        PYBIND11_OVERLOAD_PURE(
            void,                             // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            InsertData,
            _data                             // Argument(s) (...)
        );
    }
};

//////////////////////////////////////////////////
void defineMathSignalMean(py::module &m, const std::string &typestr)
{
  using Class = gz::math::SignalMean;
  std::string pyclass_name = typestr;
  py::class_<Class, SignalMeanTrampoline>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def("reset",
       &Class::Reset,
       "Forget all previous data.")
  .def("count",
       &Class::Count,
       "Get number of data points in measurement.")
  .def("value",
       &Class::Value,
       "Get a short version of the name of this statistical measure.")
  .def("short_name",
       &Class::ShortName,
       "Get a short version of the name of this statistical measure.")
  .def("insert_data",
       &Class::InsertData,
       "Add a new sample to the statistical measure.");
}

//////////////////////////////////////////////////
class SignalRootMeanSquareTrampoline :
  public gz::math::SignalRootMeanSquare
{
public:
    // Inherit the constructors
    SignalRootMeanSquareTrampoline(){}

    // Trampoline (need one for each virtual function)
    double Value() const override
    {
        PYBIND11_OVERLOAD_PURE(
            double,                           // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            Value,
        );
    }
    // Trampoline (need one for each virtual function)
    std::string ShortName() const override
    {
        PYBIND11_OVERLOAD_PURE(
            std::string,                      // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            ShortName,
        );
    }
    // Trampoline (need one for each virtual function)
    void InsertData(const double _data) override
    {
        PYBIND11_OVERLOAD_PURE(
            void,                             // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            InsertData,
            _data                             // Argument(s) (...)
        );
    }
};

//////////////////////////////////////////////////
void defineMathSignalRootMeanSquare(py::module &m, const std::string &typestr)
{
  using Class = gz::math::SignalRootMeanSquare;
  std::string pyclass_name = typestr;
  py::class_<Class, SignalRootMeanSquareTrampoline>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def("reset",
       &Class::Reset,
       "Forget all previous data.")
  .def("count",
       &Class::Count,
       "Get number of data points in measurement.")
  .def("value",
       &Class::Value,
       "Get a short version of the name of this statistical measure.")
  .def("short_name",
       &Class::ShortName,
       "Get a short version of the name of this statistical measure.")
  .def("insert_data",
       &Class::InsertData,
       "Add a new sample to the statistical measure.");
}

//////////////////////////////////////////////////
class SignalMaxAbsoluteValueTrampoline :
  public gz::math::SignalMaxAbsoluteValue
{
public:
    // Inherit the constructors
    SignalMaxAbsoluteValueTrampoline(){}

    // Trampoline (need one for each virtual function)
    double Value() const override
    {
        PYBIND11_OVERLOAD_PURE(
            double,                           // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            Value,
        );
    }
    // Trampoline (need one for each virtual function)
    std::string ShortName() const override
    {
        PYBIND11_OVERLOAD_PURE(
            std::string,                       // Return type (ret_type)
            gz::math::SignalStatistic,   // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            ShortName,
        );
    }
    // Trampoline (need one for each virtual function)
    void InsertData(const double _data) override
    {
        PYBIND11_OVERLOAD_PURE(
            void,                             // Return type (ret_type)
            gz::math::SignalStatistic,  // Parent class (cname)
            // Name of function in C++ (must match Python name) (fn)
            InsertData,
            _data                             // Argument(s) (...)
        );
    }
};

//////////////////////////////////////////////////
void defineMathSignalMaxAbsoluteValue(py::module &m, const std::string &typestr)
{
  using Class = gz::math::SignalMaxAbsoluteValue;
  std::string pyclass_name = typestr;
  py::class_<Class, SignalMaxAbsoluteValueTrampoline>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<>())
  .def("reset",
       &Class::Reset,
       "Forget all previous data.")
  .def("count",
       &Class::Count,
       "Get number of data points in measurement.")
  .def("value",
       &Class::Value,
       "Get a short version of the name of this statistical measure.")
  .def("short_name",
       &Class::ShortName,
       "Get a short version of the name of this statistical measure.")
  .def("insert_data",
       &Class::InsertData,
       "Add a new sample to the statistical measure.");
}
}  // namespace python
}  // namespace math
}  // namespace gz

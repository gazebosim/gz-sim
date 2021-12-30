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

#ifndef IGNITION_MATH_PYTHON__SIGNALSTATS_HH_
#define IGNITION_MATH_PYTHON__SIGNALSTATS_HH_

#include <pybind11/pybind11.h>
#include <string>

namespace py = pybind11;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::math::SignalStats
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathSignalStats(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::SignalStatistic
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathSignalStatistic(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::SignalMaximum
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathSignalMaximum(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::SignalMinimum
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathSignalMinimum(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::SignalVariance
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathSignalVariance(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::SignalMaxAbsoluteValue
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathSignalMaxAbsoluteValue(
  py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::SignalRootMeanSquare
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathSignalRootMeanSquare(py::module &m, const std::string &typestr);

/// Define a pybind11 wrapper for an ignition::math::SignalMean
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathSignalMean(py::module &m, const std::string &typestr);
}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // IGNITION_MATH_PYTHON__SIGNALSTATS_HH_

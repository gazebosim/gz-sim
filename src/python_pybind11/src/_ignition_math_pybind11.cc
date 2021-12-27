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

#include "Angle.hh"
#include "Color.hh"
#include "Filter.hh"
#include "Helpers.hh"
#include "Line2.hh"
#include "Line3.hh"
#include "Matrix3.hh"
#include "MovingWindowFilter.hh"
#include "Pose3.hh"
#include "Quaternion.hh"
#include "Rand.hh"
#include "RollingMean.hh"
#include "SemanticVersion.hh"
#include "Spline.hh"
#include "StopWatch.hh"
#include "Triangle.hh"
#include "Vector2.hh"
#include "Vector3.hh"
#include "Vector4.hh"

namespace py = pybind11;

PYBIND11_MODULE(math, m)
{
  m.doc() = "Ignition Math Python Library.";

  ignition::math::python::defineMathAngle(m, "Angle");

  ignition::math::python::defineMathColor(m, "Color");

  ignition::math::python::defineMathHelpers(m);

  ignition::math::python::defineMathMovingWindowFilter<int>(
    m, "MovingWindowFilteri");
  ignition::math::python::defineMathMovingWindowFilter<double>(
    m, "MovingWindowFilterd");
  ignition::math::python::defineMathMovingWindowFilter
    <ignition::math::Vector3d>(m, "MovingWindowFilterv3");

  ignition::math::python::defineMathRand(m, "Rand");

  ignition::math::python::defineMathRollingMean(m, "RollingMean");

  ignition::math::python::defineMathSemanticVersion(m, "SemanticVersion");

  ignition::math::python::defineMathSpline(m, "Spline");

  ignition::math::python::defineMathStopwatch(m, "Stopwatch");

  ignition::math::python::defineMathVector2<double>(m, "Vector2d");
  ignition::math::python::defineMathVector2<int>(m, "Vector2i");
  ignition::math::python::defineMathVector2<float>(m, "Vector2f");

  ignition::math::python::defineMathVector3<double>(m, "Vector3d");
  ignition::math::python::defineMathVector3<int>(m, "Vector3i");
  ignition::math::python::defineMathVector3<float>(m, "Vector3f");

  ignition::math::python::defineMathVector4<double>(m, "Vector4d");
  ignition::math::python::defineMathVector4<int>(m, "Vector4i");
  ignition::math::python::defineMathVector4<float>(m, "Vector4f");

  ignition::math::python::defineMathLine2<int>(m, "Line2i");
  ignition::math::python::defineMathLine2<double>(m, "Line2d");
  ignition::math::python::defineMathLine2<float>(m, "Line2f");

  ignition::math::python::defineMathLine3<int>(m, "Line3i");
  ignition::math::python::defineMathLine3<double>(m, "Line3d");
  ignition::math::python::defineMathLine3<float>(m, "Line3f");

  ignition::math::python::defineMathMatrix3<int>(m, "Matrix3i");
  ignition::math::python::defineMathMatrix3<double>(m, "Matrix3d");
  ignition::math::python::defineMathMatrix3<float>(m, "Matrix3f");

  ignition::math::python::defineMathTriangle<int>(m, "Trianglei");
  ignition::math::python::defineMathTriangle<double>(m, "Triangled");
  ignition::math::python::defineMathTriangle<float>(m, "Trianglef");

  ignition::math::python::defineMathQuaternion<int>(m, "Quaternioni");
  ignition::math::python::defineMathQuaternion<double>(m, "Quaterniond");
  ignition::math::python::defineMathQuaternion<float>(m, "Quaternionf");

  ignition::math::python::defineMathPose3<int>(m, "Pose3i");
  ignition::math::python::defineMathPose3<double>(m, "Pose3d");
  ignition::math::python::defineMathPose3<float>(m, "Pose3f");

  ignition::math::python::defineMathFilter<int>(m, "Filteri");
  ignition::math::python::defineMathFilter<float>(m, "Filterf");
  ignition::math::python::defineMathFilter<double>(m, "Filterd");

  ignition::math::python::defineMathBiQuad<int>(m, "BiQuadi");
  ignition::math::python::defineMathBiQuad<float>(m, "BiQuadf");
  ignition::math::python::defineMathBiQuad<double>(m, "BiQuadd");

  ignition::math::python::defineMathBiQuadVector3(
    m, "BiQuadVector3");

  ignition::math::python::defineMathOnePole<int>(m, "OnePolei");
  ignition::math::python::defineMathOnePole<float>(m, "OnePolef");
  ignition::math::python::defineMathOnePole<double>(m, "OnePoled");

  ignition::math::python::defineMathOnePoleQuaternion(
    m, "OnePoleQuaternion");
  ignition::math::python::defineMathOnePoleVector3(
    m, "OnePoleVector3");
}

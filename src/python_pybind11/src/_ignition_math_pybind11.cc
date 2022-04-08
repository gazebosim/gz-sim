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
#include "AxisAlignedBox.hh"
#include "Box.hh"
#include "Capsule.hh"
#include "Color.hh"
#include "Cylinder.hh"
#include "DiffDriveOdometry.hh"
#include "Ellipsoid.hh"
#include "Filter.hh"
#include "Frustum.hh"
#include "GaussMarkovProcess.hh"
#include "Helpers.hh"
#include "Inertial.hh"
#include "Kmeans.hh"
#include "Line2.hh"
#include "Line3.hh"
#include "MassMatrix3.hh"
#include "Material.hh"
#include "Matrix3.hh"
#include "Matrix4.hh"
#include "MovingWindowFilter.hh"
#include "OrientedBox.hh"
#include "PID.hh"
#include "Plane.hh"
#include "Pose3.hh"
#include "Quaternion.hh"
#include "Rand.hh"
#include "RollingMean.hh"
#include "RotationSpline.hh"
#include "SemanticVersion.hh"
#include "SignalStats.hh"
#include "Sphere.hh"
#include "SphericalCoordinates.hh"
#include "Spline.hh"
#include "StopWatch.hh"
#include "Temperature.hh"
#include "Triangle.hh"
#include "Triangle3.hh"
#include "Vector2.hh"
#include "Vector3.hh"
#include "Vector3Stats.hh"
#include "Vector4.hh"

namespace py = pybind11;

PYBIND11_MODULE(math, m)
{
  m.doc() = "Ignition Math Python Library.";

  ignition::math::python::defineMathAngle(m, "Angle");

  ignition::math::python::defineMathAxisAlignedBox(m, "AxisAlignedBox");

  ignition::math::python::defineMathCapsule(m, "Capsule");

  ignition::math::python::defineMathColor(m, "Color");

  ignition::math::python::defineMathDiffDriveOdometry(
    m, "DiffDriveOdometry");

  ignition::math::python::defineMathEllipsoid(
    m, "Ellipsoid");

  ignition::math::python::defineMathGaussMarkovProcess(
    m, "GaussMarkovProcess");

  ignition::math::python::defineMathHelpers(m);

  ignition::math::python::defineMathKmeans(m, "Kmeans");

  ignition::math::python::defineMathMaterial(m, "Material");

  ignition::math::python::defineMathMovingWindowFilter(m, "MovingWindowFilter");

  ignition::math::python::defineMathPID(m, "PID");

  ignition::math::python::defineMathRand(m, "Rand");

  ignition::math::python::defineMathRollingMean(m, "RollingMean");

  ignition::math::python::defineMathSignalStats(m, "SignalStats");
  ignition::math::python::defineMathSignalStatistic(m, "SignalStatistic");
  ignition::math::python::defineMathSignalVariance(m, "SignalVariance");
  ignition::math::python::defineMathSignalMaximum(m, "SignalMaximum");
  ignition::math::python::defineMathSignalMinimum(m, "SignalMinimum");
  ignition::math::python::defineMathSignalMaxAbsoluteValue(
    m, "SignalMaxAbsoluteValue");
  ignition::math::python::defineMathSignalRootMeanSquare(
    m, "SignalRootMeanSquare");
  ignition::math::python::defineMathSignalMean(m, "SignalMean");

  ignition::math::python::defineMathRotationSpline(m, "RotationSpline");

  ignition::math::python::defineMathVector3Stats(m, "Vector3Stats");

  ignition::math::python::defineMathSemanticVersion(m, "SemanticVersion");

  ignition::math::python::defineMathSphericalCoordinates(
    m, "SphericalCoordinates");

  ignition::math::python::defineMathSpline(m, "Spline");

  ignition::math::python::defineMathStopwatch(m, "Stopwatch");

  ignition::math::python::defineMathTemperature(m, "Temperature");

  ignition::math::python::defineMathVector2(m, "Vector2");

  ignition::math::python::defineMathVector3(m, "Vector3");

  ignition::math::python::defineMathPlane<double>(m, "Planed");

  ignition::math::python::defineMathBox<double>(m, "Boxd");
  ignition::math::python::defineMathBox<float>(m, "Boxf");

  ignition::math::python::defineMathVector4(m, "Vector4");

  ignition::math::python::defineMathLine2(m, "Line2");

  ignition::math::python::defineMathLine3(m, "Line3");

  ignition::math::python::defineMathMatrix3(m, "Matrix3");

  ignition::math::python::defineMathMatrix4(m, "Matrix4");

  ignition::math::python::defineMathTriangle(m, "Triangle");

  ignition::math::python::defineMathTriangle3(m, "Triangle3");

  ignition::math::python::defineMathQuaternion(m, "Quaternion");

  ignition::math::python::defineMathOrientedBox<double>(m, "OrientedBoxd");

  ignition::math::python::defineMathPose3(m, "Pose3");

  ignition::math::python::defineMathMassMatrix3(m, "MassMatrix3");

  ignition::math::python::defineMathSphere<double>(m, "Sphered");

  ignition::math::python::defineMathCylinder<double>(m, "Cylinderd");

  ignition::math::python::defineMathInertial<double>(m, "Inertiald");

  ignition::math::python::defineMathFrustum(m, "Frustum");

  ignition::math::python::defineMathFilter(m, "Filter");

  ignition::math::python::defineMathBiQuad(m, "BiQuad");

  ignition::math::python::defineMathBiQuadVector3(
    m, "BiQuadVector3");

  ignition::math::python::defineMathOnePole(m, "OnePole");

  ignition::math::python::defineMathOnePoleQuaternion(
    m, "OnePoleQuaternion");
  ignition::math::python::defineMathOnePoleVector3(
    m, "OnePoleVector3");
}

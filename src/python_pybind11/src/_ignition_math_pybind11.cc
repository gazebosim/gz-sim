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
  m.doc() = "Gazebo Math Python Library.";

  gz::math::python::defineMathAngle(m, "Angle");

  gz::math::python::defineMathAxisAlignedBox(m, "AxisAlignedBox");

  gz::math::python::defineMathCapsule(m, "Capsule");

  gz::math::python::defineMathColor(m, "Color");

  gz::math::python::defineMathDiffDriveOdometry(
    m, "DiffDriveOdometry");

  gz::math::python::defineMathEllipsoid(
    m, "Ellipsoid");

  gz::math::python::defineMathGaussMarkovProcess(
    m, "GaussMarkovProcess");

  gz::math::python::defineMathHelpers(m);

  gz::math::python::defineMathKmeans(m, "Kmeans");

  gz::math::python::defineMathMaterial(m, "Material");

  gz::math::python::defineMathMovingWindowFilter(m, "MovingWindowFilter");

  gz::math::python::defineMathPID(m, "PID");

  gz::math::python::defineMathRand(m, "Rand");

  gz::math::python::defineMathRollingMean(m, "RollingMean");

  gz::math::python::defineMathSignalStats(m, "SignalStats");
  gz::math::python::defineMathSignalStatistic(m, "SignalStatistic");
  gz::math::python::defineMathSignalVariance(m, "SignalVariance");
  gz::math::python::defineMathSignalMaximum(m, "SignalMaximum");
  gz::math::python::defineMathSignalMinimum(m, "SignalMinimum");
  gz::math::python::defineMathSignalMaxAbsoluteValue(
    m, "SignalMaxAbsoluteValue");
  gz::math::python::defineMathSignalRootMeanSquare(
    m, "SignalRootMeanSquare");
  gz::math::python::defineMathSignalMean(m, "SignalMean");

  gz::math::python::defineMathRotationSpline(m, "RotationSpline");

  gz::math::python::defineMathVector3Stats(m, "Vector3Stats");

  gz::math::python::defineMathSemanticVersion(m, "SemanticVersion");

  gz::math::python::defineMathSphericalCoordinates(
    m, "SphericalCoordinates");

  gz::math::python::defineMathSpline(m, "Spline");

  gz::math::python::defineMathStopwatch(m, "Stopwatch");

  gz::math::python::defineMathTemperature(m, "Temperature");

  gz::math::python::defineMathVector2(m, "Vector2");

  gz::math::python::defineMathVector3(m, "Vector3");

  gz::math::python::defineMathPlane<double>(m, "Planed");

  gz::math::python::defineMathBox<double>(m, "Boxd");
  gz::math::python::defineMathBox<float>(m, "Boxf");

  gz::math::python::defineMathVector4(m, "Vector4");

  gz::math::python::defineMathLine2(m, "Line2");

  gz::math::python::defineMathLine3(m, "Line3");

  gz::math::python::defineMathMatrix3(m, "Matrix3");

  gz::math::python::defineMathMatrix4(m, "Matrix4");

  gz::math::python::defineMathTriangle(m, "Triangle");

  gz::math::python::defineMathTriangle3(m, "Triangle3");

  gz::math::python::defineMathQuaternion(m, "Quaternion");

  gz::math::python::defineMathOrientedBox<double>(m, "OrientedBoxd");

  gz::math::python::defineMathPose3(m, "Pose3");

  gz::math::python::defineMathMassMatrix3(m, "MassMatrix3");

  gz::math::python::defineMathSphere<double>(m, "Sphered");

  gz::math::python::defineMathCylinder<double>(m, "Cylinderd");

  gz::math::python::defineMathInertial<double>(m, "Inertiald");

  gz::math::python::defineMathFrustum(m, "Frustum");

  gz::math::python::defineMathFilter(m, "Filter");

  gz::math::python::defineMathBiQuad(m, "BiQuad");

  gz::math::python::defineMathBiQuadVector3(
    m, "BiQuadVector3");

  gz::math::python::defineMathOnePole(m, "OnePole");

  gz::math::python::defineMathOnePoleQuaternion(
    m, "OnePoleQuaternion");
  gz::math::python::defineMathOnePoleVector3(
    m, "OnePoleVector3");
}

/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <cmath>

#include "ignition/math/Matrix4.hh"
#include "ignition/math/PyramidFrustumPrivate.hh"
#include "ignition/math/PyramidFrustum.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
PyramidFrustum::PyramidFrustum(const double _nearClip,
                               const double _farClip,
                               const math::Angle &_fov,
                               const math::Pose3d &_pose)
  : dataPtr(new PyramidFrustumPrivate(_nearClip, _farClip, _fov))
{
  // Compute plane based on near clip, far clip, field of view, and pose
  this->SetPose(_pose);
}

/////////////////////////////////////////////////
PyramidFrustum::~PyramidFrustum()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
PyramidFrustum::PyramidFrustum(const PyramidFrustum &_p)
  : dataPtr(new PyramidFrustumPrivate(_p.NearClip(), _p.FarClip(), _p.FOV()))
{
  for (int i = 0; i < 6; ++i)
    this->dataPtr->planes[i] = _p.dataPtr->planes[i];
}

/////////////////////////////////////////////////
Planed PyramidFrustum::Plane(const PyramidFrustumPlane _plane) const
{
  return this->dataPtr->planes[_plane];
}

/////////////////////////////////////////////////
bool PyramidFrustum::Contains(const Box &_b) const
{
  // If the box is on the negative side of a plane, then the box is not
  // visible.
  for (auto const &plane : this->dataPtr->planes)
  {
    if (plane.Side(_b) == Planed::NEGATIVE_SIDE)
      return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool PyramidFrustum::Contains(const Vector3d &_p) const
{
  // If the point is on the negative side of a plane, then the point is not
  // visible.
  for (auto const &plane : this->dataPtr->planes)
  {
      std::cout << "Check[" << plane.Normal() << "] Off[" << plane.Offset() << "] Dist[" << plane.Distance(_p) << "] Dot[" << plane.Normal().Dot(_p) << "]\n";
    if (plane.Side(_p) == Planed::NEGATIVE_SIDE)
    {
      std::cout << "   bad\n";
      return false;
    }
  }

  return true;
}

/////////////////////////////////////////////////
double PyramidFrustum::NearClip() const
{
  return this->dataPtr->nearClip;
}

/////////////////////////////////////////////////
double PyramidFrustum::FarClip() const
{
  return this->dataPtr->farClip;
}

/////////////////////////////////////////////////
math::Angle PyramidFrustum::FOV() const
{
  return this->dataPtr->fov;
}

/////////////////////////////////////////////////
void PyramidFrustum::SetPose(const Pose3d &_pose)
{
  double hNear = 2.0 * std::tan(this->dataPtr->fov() * 0.5) *
                 this->dataPtr->nearClip;

  double wNear = hNear * this->dataPtr->aspectRatio;

  double hFar = 2.0 * std::tan(this->dataPtr->fov() * 0.5) *
                 this->dataPtr->farClip;

  double wFar = hFar * this->dataPtr->aspectRatio;

  math::Vector3d dir = _pose.Rot().RotateVector(
      ignition::math::Vector3d(1, 0, 0));

  math::Vector3d up = _pose.Rot().RotateVector(
      ignition::math::Vector3d(0, 0, 1));

  math::Vector3d right = _pose.Rot().RotateVector(
      ignition::math::Vector3d(0, -1, 0));

  math::Vector3d cNear = _pose.Pos() + dir * this->dataPtr->nearClip;
  math::Vector3d cFar = _pose.Pos() + dir * this->dataPtr->farClip;
  math::Vector3d cCenter = _pose.Pos() + dir *
    ((this->dataPtr->farClip - this->dataPtr->nearClip) * 0.5 +
     this->dataPtr->nearClip);

  std::cout << cCenter << std::endl;

  math::Vector3d nearTopLeft =
    cNear + (up * (hNear * 0.5)) - (right * (wNear * 0.5));

  math::Vector3d nearTopRight =
    cNear + (up * (hNear * 0.5)) + (right * (wNear * 0.5));

  math::Vector3d nearBottomLeft =
    cNear - (up * (hNear * 0.5)) - (right * (wNear * 0.5));

  math::Vector3d nearBottomRight =
    cNear - (up * (hNear * 0.5)) + (right * (wNear * 0.5));

  math::Vector3d farTopLeft =
    cFar + (up * (hFar * 0.5)) - (right * (wFar * 0.5));

  math::Vector3d farTopRight =
    cFar + (up * (hFar * 0.5)) + (right * (wFar * 0.5));

  math::Vector3d farBottomLeft =
    cFar - (up * (hFar * 0.5)) - (right * (wFar * 0.5));

  math::Vector3d farBottomRight =
    cFar - (up * (hFar * 0.5)) + (right * (wFar * 0.5));

  double leftOffset =
    ((nearTopLeft + nearBottomLeft + farTopLeft + farBottomLeft) / 4.0).Distance(cCenter);

  double rightOffset =
    ((nearTopRight + nearBottomRight + farTopRight + farBottomRight) / 4.0).Distance(cCenter);

  double topOffset =
    ((nearTopLeft + nearTopRight + farTopLeft + farTopRight) / 4.0).Distance(cCenter);

  double bottomOffset =
    ((nearBottomLeft + nearBottomRight + farBottomLeft + farBottomRight) / 4.0).Distance(cCenter);

  math::Vector3d nearNormal =
    math::Vector3d::Normal(nearTopLeft, nearTopRight, nearBottomLeft);

  math::Vector3d leftNormal =
    math::Vector3d::Normal(nearBottomLeft, nearTopLeft, farTopLeft);

  math::Vector3d topNormal =
    math::Vector3d::Normal(nearTopLeft, farTopLeft, nearTopRight);

  math::Vector3d farNormal = nearNormal * math::Vector3d(-1, 1, 1);
  math::Vector3d rightNormal = leftNormal * math::Vector3d(1, -1, 1);
  math::Vector3d bottomNormal =  topNormal * math::Vector3d(1, 1, -1);

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_NEAR].Set(
      nearNormal, this->dataPtr->nearClip);

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_FAR].Set(
      farNormal, -this->dataPtr->farClip);

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_LEFT].Set(
      leftNormal, -leftOffset);

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_RIGHT].Set(
      rightNormal, -rightOffset);

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_TOP].Set(
      topNormal, -topOffset);

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_BOTTOM].Set(
      bottomNormal, -bottomOffset);

/*  std::cout << "Dir[" << dir << "]\n";
  std::cout << "Up[" << up << "]\n";
  std::cout << "Right[" << right << "]\n";

  std::cout << "Near Top Left[" << nearTopLeft << "]\n";
  std::cout << "Near Top Right[" << nearTopRight << "]\n";
  std::cout << "Near Bottom Left[" << nearBottomLeft << "]\n";
  std::cout << "Near Bottom Right[" << nearBottomRight << "]\n";

  std::cout << "Far Top Left[" << farTopLeft << "]\n";
  std::cout << "Far Top Right[" << farTopRight << "]\n";
  std::cout << "Far Bottom Left[" << farBottomLeft << "]\n";
  std::cout << "Far Bottom Right[" << farBottomRight << "]\n";
  */

/*  math::Matrix4d mat(_pose.Rot());

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_NEAR].Set(
      math::Vector3d(mat(0, 3) + mat(0, 0),
                     mat(1, 3) + mat(1, 0),
                     mat(2, 3) + mat(2, 0)),
      mat(3, 3) + mat(3, 0));

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_FAR].Set(
      math::Vector3d(mat(0, 3) - mat(0, 0),
                     mat(1, 3) - mat(1, 0),
                     mat(2, 3) - mat(2, 0)),
      mat(3, 3) - mat(3, 0));

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_LEFT].Set(
      math::Vector3d(mat(0, 3) - mat(0, 1),
                     mat(1, 3) - mat(1, 1),
                     mat(2, 3) - mat(2, 1)),
      mat(3, 3) - mat(3, 1));

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_RIGHT].Set(
      math::Vector3d(mat(0, 3) + mat(0, 1),
                     mat(1, 3) + mat(1, 1),
                     mat(2, 3) + mat(2, 1)),
      mat(3, 3) + mat(3, 1));

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_TOP].Set(
      math::Vector3d(mat(0, 3) + mat(0, 2),
                     mat(1, 3) + mat(1, 2),
                     mat(2, 3) + mat(2, 2)),
      mat(3, 3) + mat(3, 2));

  this->dataPtr->planes[PYRAMID_FRUSTUM_PLANE_BOTTOM].Set(
      math::Vector3d(mat(0, 3) - mat(0, 2),
                     mat(1, 3) - mat(1, 2),
                     mat(2, 3) - mat(2, 2)),
      mat(3, 3) - mat(3, 2));
      */

  for (int i = 0; i < 6; ++i)
  {
    switch (i)
    {
      case 0: std::cout << "Near "; break;
      case 1: std::cout << "Far "; break;
      case 2: std::cout << "Left "; break;
      case 3: std::cout << "Right "; break;
      case 4: std::cout << "Top "; break;
      case 5: std::cout << "Bottom "; break;
    }

    std::cout << "normal["
      << this->dataPtr->planes[i].Normal() << "] "
      << "Off["
      << this->dataPtr->planes[i].Offset() << "]\n";
  }
}

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
#include "ignition/math/FrustumPrivate.hh"
#include "ignition/math/Frustum.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
Frustum::Frustum(const double _nearClip,
                               const double _farClip,
                               const Angle &_fov,
                               const double _aspectRatio,
                               const Pose3d &_pose)
  : dataPtr(new FrustumPrivate(_nearClip, _farClip, _fov, _aspectRatio))
{
  // Compute plane based on near clip, far clip, field of view, and pose
  this->SetPose(_pose);
}

/////////////////////////////////////////////////
Frustum::~Frustum()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
Frustum::Frustum(const Frustum &_p)
  : dataPtr(new FrustumPrivate(_p.NearClip(), _p.FarClip(), _p.FOV(),
        _p.AspectRatio()))
{
  for (int i = 0; i < 6; ++i)
    this->dataPtr->planes[i] = _p.dataPtr->planes[i];
}

/////////////////////////////////////////////////
Planed Frustum::Plane(const FrustumPlane _plane) const
{
  return this->dataPtr->planes[_plane];
}

/////////////////////////////////////////////////
bool Frustum::Contains(const Box &_b) const
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
bool Frustum::Contains(const Vector3d &_p) const
{
  // If the point is on the negative side of a plane, then the point is not
  // visible.
  for (auto const &plane : this->dataPtr->planes)
  {
    if (plane.Side(_p) == Planed::NEGATIVE_SIDE)
      return false;
  }

  return true;
}

/////////////////////////////////////////////////
double Frustum::NearClip() const
{
  return this->dataPtr->nearClip;
}

/////////////////////////////////////////////////
void Frustum::SetNearClip(const double _near)
{
  this->dataPtr->nearClip = _near;
  this->ComputePlanes();
}

/////////////////////////////////////////////////
double Frustum::FarClip() const
{
  return this->dataPtr->farClip;
}

/////////////////////////////////////////////////
void Frustum::SetFarClip(const double _far)
{
  this->dataPtr->farClip = _far;
  this->ComputePlanes();
}

/////////////////////////////////////////////////
Angle Frustum::FOV() const
{
  return this->dataPtr->fov;
}

/////////////////////////////////////////////////
void Frustum::SetFOV(const Angle &_angle)
{
  this->dataPtr->fov = _angle;
  this->ComputePlanes();
}

/////////////////////////////////////////////////
Pose3d Frustum::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
void Frustum::SetPose(const Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
  this->ComputePlanes();
}

/////////////////////////////////////////////////
double Frustum::AspectRatio() const
{
  return this->dataPtr->aspectRatio;
}

/////////////////////////////////////////////////
void Frustum::SetAspectRatio(const double _aspectRatio)
{
  this->dataPtr->aspectRatio = _aspectRatio;
  this->ComputePlanes();
}

/////////////////////////////////////////////////
void Frustum::ComputePlanes()
{
  // Tangent of half the field of view.
  double tanFOV2 = std::tan(this->dataPtr->fov() * 0.5);

  // Width and height of near clip plane
  double nearHeight = 2.0 * tanFOV2 * this->dataPtr->nearClip;
  double nearWidth = nearHeight * this->dataPtr->aspectRatio;

  // Width and height of far clip plane
  double farHeight = 2.0 * tanFOV2 * this->dataPtr->farClip;
  double farWidth = farHeight * this->dataPtr->aspectRatio;

  // Up, right, and forward unit vectors.
  Vector3d forward = this->dataPtr->pose.Rot().RotateVector(Vector3d::UnitX);
  Vector3d up = this->dataPtr->pose.Rot().RotateVector(Vector3d::UnitZ);
  Vector3d right = this->dataPtr->pose.Rot().RotateVector(-Vector3d::UnitY);

  // Near plane center
  Vector3d nearCenter = this->dataPtr->pose.Pos() + forward *
    this->dataPtr->nearClip;

  // Far plane center
  Vector3d farCenter = this->dataPtr->pose.Pos() + forward *
    this->dataPtr->farClip;

  // Center point between near and far planes
  Vector3d center = this->dataPtr->pose.Pos() + forward *
    ((this->dataPtr->farClip - this->dataPtr->nearClip) * 0.5 +
     this->dataPtr->nearClip);

  // These four variables are here for convenience.
  Vector3d upNearHeight2 = up * (nearHeight * 0.5);
  Vector3d rightNearWidth2 = right * (nearHeight * 0.5);
  Vector3d upFarHeight2 = up * (farHeight * 0.5);
  Vector3d rightFarWidth2 = right * (farHeight * 0.5);

  // Comptue the vertices of the near clip plane
  Vector3d nearTopLeft = nearCenter + upNearHeight2 - rightNearWidth2;
  Vector3d nearTopRight = nearCenter + upNearHeight2 + rightNearWidth2;
  Vector3d nearBottomLeft = nearCenter - upNearHeight2 - rightNearWidth2;
  Vector3d nearBottomRight = nearCenter - upNearHeight2 + rightNearWidth2;

  // Compute the vertices of the far clip plane
  Vector3d farTopLeft = farCenter + upFarHeight2 - rightFarWidth2;
  Vector3d farTopRight = farCenter + upFarHeight2 + rightFarWidth2;
  Vector3d farBottomLeft = farCenter - upFarHeight2 - rightFarWidth2;
  Vector3d farBottomRight = farCenter - upFarHeight2 + rightFarWidth2;

  // Compute plane offsets
  double leftOffset = -((nearTopLeft + nearBottomLeft + farTopLeft +
        farBottomLeft) / 4.0).Distance(center);

  double rightOffset = -((nearTopRight + nearBottomRight + farTopRight +
        farBottomRight) / 4.0).Distance(center);

  double topOffset = -((nearTopLeft + nearTopRight + farTopLeft +
        farTopRight) / 4.0).Distance(center);

  double bottomOffset = -((nearBottomLeft + nearBottomRight + farBottomLeft +
        farBottomRight) / 4.0).Distance(center);

  // Set the planes, where the first value is the plane normal and the
  // second the plane offset
  this->dataPtr->planes[FRUSTUM_PLANE_NEAR].Set(
      Vector3d::Normal(nearTopLeft, nearTopRight, nearBottomLeft),
      this->dataPtr->nearClip);

  this->dataPtr->planes[FRUSTUM_PLANE_FAR].Set(
      Vector3d::Normal(farTopRight, farTopLeft, farBottomLeft),
      -this->dataPtr->farClip);

  this->dataPtr->planes[FRUSTUM_PLANE_LEFT].Set(
      Vector3d::Normal(farTopLeft, nearTopLeft, nearBottomLeft), leftOffset);

  this->dataPtr->planes[FRUSTUM_PLANE_RIGHT].Set(
      Vector3d::Normal(nearTopRight, farTopRight, farBottomRight), rightOffset);

  this->dataPtr->planes[FRUSTUM_PLANE_TOP].Set(
      Vector3d::Normal(nearTopLeft, farTopLeft, nearTopRight), topOffset);

  this->dataPtr->planes[FRUSTUM_PLANE_BOTTOM].Set(
      Vector3d::Normal(nearBottomLeft, nearBottomRight, farBottomRight),
      bottomOffset);
}

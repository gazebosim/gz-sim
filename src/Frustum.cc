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
Frustum::Frustum(const double _near,
                 const double _far,
                 const Angle &_fov,
                 const double _aspectRatio,
                 const Pose3d &_pose)
  : dataPtr(new FrustumPrivate(_near, _far, _fov, _aspectRatio, _pose))
{
  // Compute plane based on near distance, far distance, field of view,
  // aspect ratio, and pose
  this->ComputePlanes();
}

/////////////////////////////////////////////////
Frustum::~Frustum()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
Frustum::Frustum(const Frustum &_p)
  : dataPtr(new FrustumPrivate(_p.Near(), _p.Far(), _p.FOV(),
        _p.AspectRatio(), _p.Pose()))
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
double Frustum::Near() const
{
  return this->dataPtr->near;
}

/////////////////////////////////////////////////
void Frustum::SetNear(const double _near)
{
  this->dataPtr->near = _near;
  this->ComputePlanes();
}

/////////////////////////////////////////////////
double Frustum::Far() const
{
  return this->dataPtr->far;
}

/////////////////////////////////////////////////
void Frustum::SetFar(const double _far)
{
  this->dataPtr->far = _far;
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

  // Height of near plane
  double nearHeight = 2.0 * tanFOV2 * this->dataPtr->near;

  // Height of far plane
  double farHeight = 2.0 * tanFOV2 * this->dataPtr->far;

  // Up, right, and forward unit vectors.
  Vector3d forward = this->dataPtr->pose.Rot().RotateVector(Vector3d::UnitX);
  Vector3d up = this->dataPtr->pose.Rot().RotateVector(Vector3d::UnitZ);
  Vector3d right = this->dataPtr->pose.Rot().RotateVector(-Vector3d::UnitY);

  // Near plane center
  Vector3d nearCenter = this->dataPtr->pose.Pos() + forward *
    this->dataPtr->near;

  // Far plane center
  Vector3d farCenter = this->dataPtr->pose.Pos() + forward *
    this->dataPtr->far;

  // Center point between near and far planes
  Vector3d center = this->dataPtr->pose.Pos() + forward *
    ((this->dataPtr->far - this->dataPtr->near) * 0.5 +
     this->dataPtr->near);

  // These four variables are here for convenience.
  Vector3d upNearHeight2 = up * (nearHeight * 0.5);
  Vector3d rightNearWidth2 = right * (nearHeight * 0.5);
  Vector3d upFarHeight2 = up * (farHeight * 0.5);
  Vector3d rightFarWidth2 = right * (farHeight * 0.5);

  // Comptue the vertices of the near plane
  Vector3d nearTopLeft = nearCenter + upNearHeight2 - rightNearWidth2;
  Vector3d nearTopRight = nearCenter + upNearHeight2 + rightNearWidth2;
  Vector3d nearBottomLeft = nearCenter - upNearHeight2 - rightNearWidth2;
  Vector3d nearBottomRight = nearCenter - upNearHeight2 + rightNearWidth2;

  // Compute the vertices of the far plane
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
      this->dataPtr->near);

  this->dataPtr->planes[FRUSTUM_PLANE_FAR].Set(
      Vector3d::Normal(farTopRight, farTopLeft, farBottomLeft),
      -this->dataPtr->far);

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

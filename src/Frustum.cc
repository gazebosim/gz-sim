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
Frustum::Frustum()
  : dataPtr(new FrustumPrivate(0, 1, IGN_DTOR(45), 1, Pose3d::Zero))
{
}

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
  // This is a fast test used for culling.
  // If the box is on the negative side of a plane, then the box is not
  // visible.
  int overlapping = 0;
  for (auto const &plane : this->dataPtr->planes)
  {
    auto const sign = plane.Side(_b);
    if (sign == Planed::NEGATIVE_SIDE)
      return false;
    else if (sign == Planed::BOTH_SIDE)
      ++overlapping;
  }

  // it is possible to be outside of frustum and overlapping multiple planes
  if (overlapping >= 2)
  {
    // return true if any box point is inside the frustum
    auto const &min = _b.Min();
    auto const &max = _b.Max();
    for (int p = 0; p < 8; ++p)
    {
      const double &x = (p & 4) ? min.X() : max.X();
      const double &y = (p & 2) ? min.Y() : max.Y();
      const double &z = (p & 1) ? min.Z() : max.Z();
      if (this->Contains(Vector3d(x, y, z)))
        return true;
    }
    // return true if any frustum point is inside the box
    for (auto const &pt : this->dataPtr->points)
    {
      if (_b.Contains(pt))
        return true;
    }

    // Return true if any edge of the frustum passes through the AABB
    for (const auto &edge : this->dataPtr->edges)
    {
      // If the edge projected onto a world axis does not overlapp with the AABB
      // then the edge could not be passing through the AABB.
      if (edge.first.X() < min.X() && edge.second.X() < min.X())
      {
        // both frustum edge points are below AABB on x axis
        continue;
      }
      else if (edge.first.X() > max.X() && edge.second.X() > max.X())
      {
        // both frustum edge points are above AABB on x axis
        continue;
      }
      else if (edge.first.Y() < min.Y() && edge.second.Y() < min.Y())
      {
        // both frustum edge points are below AABB on y axis
        continue;
      }
      else if (edge.first.Y() > max.Y() && edge.second.Y() > max.Y())
      {
        // both frustum edge points are above AABB on y axis
        continue;
      }
      else if (edge.first.Z() < min.Z() && edge.second.Z() < min.Z())
      {
        // both frustum edge points are below AABB on z axis
        continue;
      }
      else if (edge.first.Z() > max.Z() && edge.second.Z() > max.Z())
      {
        // both frustum edge points are above AABB on z axis
        continue;
      }
      else
      {
        // TODO prove or disprove that Frustum must penetrate AABB???
        return true;
      }
    }
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

  // Width of near plane
  double nearWidth = 2.0 * tanFOV2 * this->dataPtr->near;

  // Height of near plane
  double nearHeight = nearWidth / this->dataPtr->aspectRatio;

  // Width of far plane
  double farWidth = 2.0 * tanFOV2 * this->dataPtr->far;

  // Height of far plane
  double farHeight = farWidth / this->dataPtr->aspectRatio;

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

  // These four variables are here for convenience.
  Vector3d upNearHeight2 = up * (nearHeight * 0.5);
  Vector3d rightNearWidth2 = right * (nearWidth * 0.5);
  Vector3d upFarHeight2 = up * (farHeight * 0.5);
  Vector3d rightFarWidth2 = right * (farWidth * 0.5);

  // Compute the vertices of the near plane
  Vector3d nearTopLeft = nearCenter + upNearHeight2 - rightNearWidth2;
  Vector3d nearTopRight = nearCenter + upNearHeight2 + rightNearWidth2;
  Vector3d nearBottomLeft = nearCenter - upNearHeight2 - rightNearWidth2;
  Vector3d nearBottomRight = nearCenter - upNearHeight2 + rightNearWidth2;

  // Compute the vertices of the far plane
  Vector3d farTopLeft = farCenter + upFarHeight2 - rightFarWidth2;
  Vector3d farTopRight = farCenter + upFarHeight2 + rightFarWidth2;
  Vector3d farBottomLeft = farCenter - upFarHeight2 - rightFarWidth2;
  Vector3d farBottomRight = farCenter - upFarHeight2 + rightFarWidth2;

  // Save these vertices
  this->dataPtr->points[0] = nearTopLeft;
  this->dataPtr->points[1] = nearTopRight;
  this->dataPtr->points[2] = nearBottomLeft;
  this->dataPtr->points[3] = nearBottomRight;
  this->dataPtr->points[4] = farTopLeft;
  this->dataPtr->points[5] = farTopRight;
  this->dataPtr->points[6] = farBottomLeft;
  this->dataPtr->points[7] = farBottomRight;

  // Save the edges
  this->dataPtr->edges[0] = {nearTopLeft, nearTopRight};
  this->dataPtr->edges[1] = {nearTopLeft, nearBottomLeft};
  this->dataPtr->edges[2] = {nearTopLeft, farTopLeft};
  this->dataPtr->edges[3] = {nearTopRight, nearBottomRight};
  this->dataPtr->edges[4] = {nearTopRight, farTopRight};
  this->dataPtr->edges[5] = {nearBottomLeft, nearBottomRight};
  this->dataPtr->edges[6] = {nearBottomLeft, farBottomLeft};
  this->dataPtr->edges[7] = {farTopLeft, farTopRight};
  this->dataPtr->edges[8] = {farTopLeft, farBottomLeft};
  this->dataPtr->edges[9] = {farTopRight, farBottomRight};
  this->dataPtr->edges[10] = {farBottomLeft, farBottomRight};
  this->dataPtr->edges[11] = {farBottomRight, nearBottomRight};

  Vector3d leftCenter =
    (farTopLeft + nearTopLeft + farBottomLeft + nearBottomLeft) / 4.0;

  Vector3d rightCenter =
    (farTopRight + nearTopRight + farBottomRight + nearBottomRight) / 4.0;

  Vector3d topCenter =
    (farTopRight + nearTopRight + farTopLeft + nearTopLeft) / 4.0;

  Vector3d bottomCenter =
    (farBottomRight + nearBottomRight + farBottomLeft + nearBottomLeft) / 4.0;

  // Compute plane offsets
  // Set the planes, where the first value is the plane normal and the
  // second the plane offset
  Vector3d norm = Vector3d::Normal(nearTopLeft, nearTopRight, nearBottomLeft);
  this->dataPtr->planes[FRUSTUM_PLANE_NEAR].Set(norm, nearCenter.Dot(norm));

  norm = Vector3d::Normal(farTopRight, farTopLeft, farBottomLeft);
  this->dataPtr->planes[FRUSTUM_PLANE_FAR].Set(norm, farCenter.Dot(norm));

  norm = Vector3d::Normal(farTopLeft, nearTopLeft, nearBottomLeft);
  this->dataPtr->planes[FRUSTUM_PLANE_LEFT].Set(norm, leftCenter.Dot(norm));

  norm = Vector3d::Normal(nearTopRight, farTopRight, farBottomRight);
  this->dataPtr->planes[FRUSTUM_PLANE_RIGHT].Set(norm, rightCenter.Dot(norm));

  norm = Vector3d::Normal(nearTopLeft, farTopLeft, nearTopRight);
  this->dataPtr->planes[FRUSTUM_PLANE_TOP].Set(norm, topCenter.Dot(norm));

  norm = Vector3d::Normal(nearBottomLeft, nearBottomRight, farBottomRight);
  this->dataPtr->planes[FRUSTUM_PLANE_BOTTOM].Set(norm, bottomCenter.Dot(norm));
}

//////////////////////////////////////////////////
Frustum &Frustum::operator =(const Frustum &_f)
{
  this->dataPtr->near = _f.dataPtr->near;
  this->dataPtr->far = _f.dataPtr->far;
  this->dataPtr->fov = _f.dataPtr->fov;
  this->dataPtr->aspectRatio = _f.dataPtr->aspectRatio;
  this->dataPtr->pose = _f.dataPtr->pose;
  this->ComputePlanes();

  return *this;
}

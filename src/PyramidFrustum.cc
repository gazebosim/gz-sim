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
#include "ignition/math/PyramidFrustumPrivate.hh"
#include "ignition/math/PyramidFrustum.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
PyramidFrustum::PyramidFrustum()
  : dataPtr(new PyramidFrustumPrivate)
{
}

/////////////////////////////////////////////////
PyramidFrustum::~PyramidFrustum()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
PyramidFrustum::PyramidFrustum(const PyramidFrustum &_p)
  : dataPtr(new PyramidFrustumPrivate)
{
  for (int i = 0; i < 6; ++i)
    this->dataPtr->planes[i] = _p.dataPtr->planes[i]
}

/////////////////////////////////////////////////
Plane PyramidFrustum::Plane(const PyramidFrustumPlane _plane) const
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
    if (plane.Side(_b) == Plane::NEGATIVE_SIDE)
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
    if (plane.Side(_p) == Plane::NEGATIVE_SIDE)
      return false;
  }

  return true;
}

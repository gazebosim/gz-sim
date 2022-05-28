/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_MATH_DETAIL_BOX_HH_
#define GZ_MATH_DETAIL_BOX_HH_

#include "gz/math/Triangle3.hh"

#include <algorithm>
#include <set>
#include <utility>
#include <vector>

namespace gz
{
namespace math
{
//////////////////////////////////////////////////
template<typename T>
Box<T>::Box(T _length, T _width, T _height)
{
  this->size.X(_length);
  this->size.Y(_width);
  this->size.Z(_height);
}

//////////////////////////////////////////////////
template<typename T>
Box<T>::Box(T _length, T _width, T _height,
    const gz::math::Material &_mat)
{
  this->size.X(_length);
  this->size.Y(_width);
  this->size.Z(_height);
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
Box<T>::Box(const Vector3<T> &_size)
{
  this->size = _size;
}

//////////////////////////////////////////////////
template<typename T>
Box<T>::Box(const Vector3<T> &_size, const gz::math::Material &_mat)
{
  this->size = _size;
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
math::Vector3<T> Box<T>::Size() const
{
  return this->size;
}

//////////////////////////////////////////////////
template<typename T>
void Box<T>::SetSize(T _length, T _width, T _height)
{
  this->size.X(_length);
  this->size.Y(_width);
  this->size.Z(_height);
}

//////////////////////////////////////////////////
template<typename T>
void Box<T>::SetSize(const math::Vector3<T> &_size)
{
  this->size = _size;
}

//////////////////////////////////////////////////
template<typename T>
const gz::math::Material &Box<T>::Material() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Box<T>::SetMaterial(const gz::math::Material &_mat)
{
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
bool Box<T>::operator==(const Box<T> &_b) const
{
  return this->size == _b.size && this->material == _b.material;
}

//////////////////////////////////////////////////
template<typename T>
bool Box<T>::operator!=(const Box<T> &_b) const
{
  return !(*this == _b);
}

/////////////////////////////////////////////////
template<typename T>
T Box<T>::Volume() const
{
  return this->size.X() * this->size.Y() * this->size.Z();
}

//////////////////////////////////////////////////
/// \brief Given a *convex* polygon described by the verices in a given plane,
/// compute the list of triangles which form this polygon.
/// \param[in] _plane The plane in which the vertices exist.
/// \param[in] _vertices The vertices of the polygon.
/// \return A vector of triangles and their sign, or an empty vector
/// if _vertices in the _plane are less than 3. The sign will be +1 if the
/// triangle is outward facing, -1 otherwise.
template <typename T>
std::vector<std::pair<Triangle3<T>, T>> TrianglesInPlane(
    const Plane<T> &_plane, IntersectionPoints<T> &_vertices)
{
  std::vector<std::pair<Triangle3<T>, T>> triangles;
  std::vector<Vector3<T>> pointsInPlane;

  Vector3<T> centroid;
  for (const auto &pt : _vertices)
  {
    if (_plane.Side(pt) == Plane<T>::NO_SIDE)
    {
      pointsInPlane.push_back(pt);
      centroid += pt;
    }
  }
  centroid /= T(pointsInPlane.size());

  if (pointsInPlane.size() < 3)
    return {};

  // Choose a basis in the plane of the triangle
  auto axis1 = (pointsInPlane[0] - centroid).Normalize();
  auto axis2 = axis1.Cross(_plane.Normal()).Normalize();

  // Since the polygon is always convex, we can try to create a fan of triangles
  // by sorting the points by their angle in the plane basis.
  std::sort(pointsInPlane.begin(), pointsInPlane.end(),
    [centroid, axis1, axis2] (const Vector3<T> &_a, const Vector3<T> &_b)
    {
      auto aDisplacement = _a - centroid;
      auto bDisplacement = _b - centroid;

      // project line onto the new basis vectors
      // The axis length will never be zero as we have three different points
      // and the centroid will be different.
      auto aX = axis1.Dot(aDisplacement) / axis1.Length();
      auto aY = axis2.Dot(aDisplacement) / axis2.Length();

      auto bX = axis1.Dot(bDisplacement) / axis1.Length();
      auto bY = axis2.Dot(bDisplacement) / axis2.Length();

      return atan2(aY, aX) < atan2(bY, bX);
    });
  for (std::size_t i = 0; i < pointsInPlane.size(); ++i)
  {
    triangles.emplace_back(
      Triangle3<T>(pointsInPlane[i],
        pointsInPlane[(i + 1) % pointsInPlane.size()], centroid),
      (_plane.Side({0, 0, 0}) == Plane<T>::POSITIVE_SIDE) ? -1 : 1);
  }

  return triangles;
}

/////////////////////////////////////////////////
template<typename T>
T Box<T>::VolumeBelow(const Plane<T> &_plane) const
{
  auto verticesBelow = this->VerticesBelow(_plane);
  if (verticesBelow.empty())
    return 0;

  auto intersections = this->Intersections(_plane);
  // TODO(arjo): investigate the use of _epsilon tolerance as this method
  // implicitly uses Vector3<T>::operator==()
  verticesBelow.merge(intersections);

  // Reconstruct the cut-box as a triangle mesh by attempting to fit planes.
  std::vector<std::pair<Triangle3<T>, T>> triangles;

  std::vector<Plane<T>> planes
  {
    Plane<T>{Vector3<T>{0, 0, 1}, this->Size().Z()/2},
    Plane<T>{Vector3<T>{0, 0, -1}, this->Size().Z()/2},
    Plane<T>{Vector3<T>{1, 0, 0}, this->Size().X()/2},
    Plane<T>{Vector3<T>{-1, 0, 0}, this->Size().X()/2},
    Plane<T>{Vector3<T>{0, 1, 0}, this->Size().Y()/2},
    Plane<T>{Vector3<T>{0, -1, 0}, this->Size().Y()/2},
    _plane
  };

  for (const auto &p : planes)
  {
    auto newTriangles = TrianglesInPlane(p, verticesBelow);
    triangles.insert(triangles.end(),
      newTriangles.begin(),
      newTriangles.end());
  }

  // Calculate the volume of the triangles
  // https://n-e-r-v-o-u-s.com/blog/?p=4415
  T volume = 0;
  for (const auto &triangle : triangles)
  {
    auto crossProduct = (triangle.first[2]).Cross(triangle.first[1]);
    auto meshVolume = std::abs(crossProduct.Dot(triangle.first[0]));
    volume += triangle.second * meshVolume;
  }

  return std::abs(volume)/6;
}

/////////////////////////////////////////////////
template<typename T>
std::optional<Vector3<T>>
  Box<T>::CenterOfVolumeBelow(const Plane<T> &_plane) const
{
  auto verticesBelow = this->VerticesBelow(_plane);
  if (verticesBelow.empty())
    return std::nullopt;

  auto intersections = this->Intersections(_plane);
  verticesBelow.merge(intersections);

  Vector3<T> centroid;
  for (const auto &v : verticesBelow)
  {
    centroid += v;
  }

  return centroid / static_cast<T>(verticesBelow.size());
}

/////////////////////////////////////////////////
template<typename T>
IntersectionPoints<T> Box<T>::VerticesBelow(const Plane<T> &_plane) const
{
  // Get coordinates of all vertice of box
  // TODO(arjo): Cache this for performance
  IntersectionPoints<T> vertices
  {
    Vector3<T>{this->size.X()/2, this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{this->size.X()/2, -this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, -this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{this->size.X()/2, this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{this->size.X()/2, -this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, -this->size.Y()/2, -this->size.Z()/2}
  };

  IntersectionPoints<T> verticesBelow;
  for (const auto &v : vertices)
  {
    if (_plane.Distance(v) <= 0)
    {
      verticesBelow.insert(v);
    }
  }

  return verticesBelow;
}

/////////////////////////////////////////////////
template<typename T>
T Box<T>::DensityFromMass(const T _mass) const
{
  if (this->size.Min() <= 0|| _mass <= 0)
    return -1.0;

  return _mass / this->Volume();
}

/////////////////////////////////////////////////
template<typename T>
bool Box<T>::SetDensityFromMass(const T _mass)
{
  T newDensity = this->DensityFromMass(_mass);
  if (newDensity > 0)
    this->material.SetDensity(newDensity);
  return newDensity > 0;
}

/////////////////////////////////////////////////
template<typename T>
bool Box<T>::MassMatrix(MassMatrix3<T> &_massMat) const
{
  return _massMat.SetFromBox(this->material, this->size);
}


//////////////////////////////////////////////////
template<typename T>
IntersectionPoints<T> Box<T>::Intersections(
        const Plane<T> &_plane) const
{
  IntersectionPoints<T> intersections;
  // These are vertices via which we can describe edges. We only need 4 such
  // vertices
  std::vector<Vector3<T> > vertices
  {
    Vector3<T>{-this->size.X()/2, -this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{this->size.X()/2, this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{this->size.X()/2, -this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, this->size.Y()/2, this->size.Z()/2}
  };

  // Axes
  std::vector<Vector3<T>> axes
  {
    Vector3<T>{1, 0, 0},
    Vector3<T>{0, 1, 0},
    Vector3<T>{0, 0, 1}
  };

  // There are 12 edges, which are checked along 3 axes from 4 box corner
  // points.
  for (auto &v : vertices)
  {
    for (auto &a : axes)
    {
      auto intersection = _plane.Intersection(v, a);
      if (intersection.has_value() &&
          intersection->X() >= -this->size.X()/2 &&
          intersection->X() <= this->size.X()/2 &&
          intersection->Y() >= -this->size.Y()/2 &&
          intersection->Y() <= this->size.Y()/2 &&
          intersection->Z() >= -this->size.Z()/2 &&
          intersection->Z() <= this->size.Z()/2)
      {
        intersections.insert(intersection.value());
      }
    }
  }

  return intersections;
}

}
}
#endif

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
#ifndef GZ_MATH_DETAIL_SPHERE_HH_
#define GZ_MATH_DETAIL_SPHERE_HH_

#include "gz/math/Sphere.hh"

namespace gz
{
namespace math
{
//////////////////////////////////////////////////
template<typename T>
Sphere<T>::Sphere(const T _radius)
{
  this->radius = _radius;
}

//////////////////////////////////////////////////
template<typename T>
Sphere<T>::Sphere(const T _radius, const gz::math::Material &_mat)
{
  this->radius = _radius;
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
T Sphere<T>::Radius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
template<typename T>
void Sphere<T>::SetRadius(const T _radius)
{
  this->radius = _radius;
}

//////////////////////////////////////////////////
template<typename T>
const gz::math::Material &Sphere<T>::Material() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Sphere<T>::SetMaterial(const gz::math::Material &_mat)
{
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
bool Sphere<T>::operator==(const Sphere &_sphere) const
{
  return equal(this->radius, _sphere.Radius()) &&
    this->material == _sphere.Material();
}

//////////////////////////////////////////////////
template<typename T>
bool Sphere<T>::operator!=(const Sphere &_sphere) const
{
  return !(*this == _sphere);
}

//////////////////////////////////////////////////
template<typename T>
bool Sphere<T>::MassMatrix(MassMatrix3d &_massMat) const
{
  return _massMat.SetFromSphere(this->material, this->radius);
}

//////////////////////////////////////////////////
template<typename T>
T Sphere<T>::Volume() const
{
  return (4.0/3.0) * GZ_PI * std::pow(this->radius, 3);
}

//////////////////////////////////////////////////
template<typename T>
T Sphere<T>::VolumeBelow(const Plane<T> &_plane) const
{
  auto r = this->radius;
  // get nearest point to center on plane
  auto dist = _plane.Distance(Vector3d(0, 0, 0));

  if (dist < -r)
  {
    // sphere is below plane.
    return Volume();
  }
  else if (dist > r)
  {
    // sphere is completely above plane
    return 0.0;
  }

  auto h = r - dist;
  return GZ_PI * h * h * (3 * r - h) / 3;
}

//////////////////////////////////////////////////
template<typename T>
std::optional<Vector3<T>>
 Sphere<T>::CenterOfVolumeBelow(const Plane<T> &_plane) const
{
  auto r = this->radius;
  // get nearest point to center on plane
  auto dist = _plane.Distance(Vector3d(0, 0, 0));

  if (dist < -r)
  {
    // sphere is completely below plane
    return Vector3<T>{0, 0, 0};
  }
  else if (dist > r)
  {
    // sphere is completely above plane
    return std::nullopt;
  }

  // Get the height of the spherical cap
  auto h = r - dist;

  // Formula for geometric centorid:
  // https://mathworld.wolfram.com/SphericalCap.html
  auto numerator = 2 * r - h;

  auto z = 3 * numerator * numerator / (4 * (3 * r - h));
  return - z * _plane.Normal().Normalized();
}

//////////////////////////////////////////////////
template<typename T>
bool Sphere<T>::SetDensityFromMass(const T _mass)
{
  T newDensity = this->DensityFromMass(_mass);
  if (newDensity > 0)
    this->material.SetDensity(newDensity);
  return newDensity > 0;
}

//////////////////////////////////////////////////
template<typename T>
T Sphere<T>::DensityFromMass(const T _mass) const
{
  if (this->radius <= 0 || _mass <= 0)
    return -1.0;

  return _mass / this->Volume();
}
}
}
#endif

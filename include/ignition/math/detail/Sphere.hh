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
#ifndef IGNITION_MATH_DETAIL_SPHERE_HH_
#define IGNITION_MATH_DETAIL_SPHERE_HH_

#include "ignition/math/Sphere.hh"

namespace ignition
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
Sphere<T>::Sphere(const T _radius, const ignition::math::Material &_mat)
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
const ignition::math::Material &Sphere<T>::Material() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Sphere<T>::SetMaterial(const ignition::math::Material &_mat)
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
  return (4.0/3.0) * IGN_PI * std::pow(this->radius, 3);
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

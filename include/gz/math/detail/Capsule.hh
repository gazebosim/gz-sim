/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef GZ_MATH_DETAIL_CAPSULE_HH_
#define GZ_MATH_DETAIL_CAPSULE_HH_

#include <limits>
#include <optional>
#include <gz/math/Helpers.hh>
#include <gz/math/Inertial.hh>

namespace gz
{
namespace math
{

//////////////////////////////////////////////////
template<typename T>
Capsule<T>::Capsule(const T _length, const T _radius)
{
  this->length = _length;
  this->radius = _radius;
}

//////////////////////////////////////////////////
template<typename T>
Capsule<T>::Capsule(const T _length, const T _radius, const Material &_mat)
{
  this->length = _length;
  this->radius = _radius;
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
T Capsule<T>::Radius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
template<typename T>
void Capsule<T>::SetRadius(const T _radius)
{
  this->radius = _radius;
}

//////////////////////////////////////////////////
template<typename T>
T Capsule<T>::Length() const
{
  return this->length;
}

//////////////////////////////////////////////////
template<typename T>
void Capsule<T>::SetLength(const T _length)
{
  this->length = _length;
}

//////////////////////////////////////////////////
template<typename T>
const Material &Capsule<T>::Mat() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Capsule<T>::SetMat(const Material &_mat)
{
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
bool Capsule<T>::operator==(const Capsule &_capsule) const
{
  return equal(this->radius, _capsule.Radius()) &&
    equal(this->length, _capsule.Length()) &&
    this->material == _capsule.Mat();
}

//////////////////////////////////////////////////
template<typename T>
std::optional< MassMatrix3<T> > Capsule<T>::MassMatrix() const
{
  // mass and moment of inertia of cylinder about centroid
  MassMatrix3<T> cylinder;
  cylinder.SetFromCylinderZ(this->material, this->length, this->radius);

  // mass and moment of inertia of hemisphere about centroid
  const T r2 = this->radius * this->radius;
  const T hemisphereMass = this->material.Density() *
      2. / 3. * GZ_PI * r2 * this->radius;
  // efunda.com/math/solids/solids_display.cfm?SolidName=Hemisphere
  const T ixx = 83. / 320. * hemisphereMass * r2;
  const T izz = 2. / 5.  * hemisphereMass * r2;
  MassMatrix3<T> hemisphere(hemisphereMass, Vector3<T>(ixx, ixx, izz),
                            Vector3<T>::Zero);;

  // Distance from centroid of cylinder to centroid of hemisphere,
  // since centroid of hemisphere is 3/8 radius from its flat base
  const T dz = this->length / 2. + this->radius * 3. / 8.;
  Inertial<T> upperCap(hemisphere, Pose3<T>(0, 0, dz, 0, 0, 0));
  Inertial<T> lowerCap(hemisphere, Pose3<T>(0, 0, -dz, 0, 0, 0));

  // Use Inertial class to add MassMatrix3 objects at different poses
  Inertial<T> capsule =
      Inertial<T>(cylinder, Pose3<T>::Zero) + upperCap + lowerCap;

  if (!capsule.MassMatrix().IsValid())
  {
    return std::nullopt;
  }

  return std::make_optional(capsule.MassMatrix());
}

//////////////////////////////////////////////////
template<typename T>
T Capsule<T>::Volume() const
{
  return GZ_PI * std::pow(this->radius, 2) *
         (this->length + 4. / 3. * this->radius);
}

//////////////////////////////////////////////////
template<typename T>
bool Capsule<T>::SetDensityFromMass(const T _mass)
{
  T newDensity = this->DensityFromMass(_mass);
  if (isnan(newDensity))
    return false;

  this->material.SetDensity(newDensity);
  return true;
}

//////////////////////////////////////////////////
template<typename T>
T Capsule<T>::DensityFromMass(const T _mass) const
{
  if (this->radius <= 0 || this->length <=0 || _mass <= 0)
    return std::numeric_limits<T>::quiet_NaN();

  return _mass / this->Volume();
}

}
}
#endif

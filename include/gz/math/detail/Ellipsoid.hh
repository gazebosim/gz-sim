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
#ifndef GZ_MATH_DETAIL_ELLIPSOID_HH_
#define GZ_MATH_DETAIL_ELLIPSOID_HH_

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
Ellipsoid<T>::Ellipsoid(const Vector3<T> &_radii) : radii(_radii) {}

//////////////////////////////////////////////////
template<typename T>
Ellipsoid<T>::Ellipsoid(const Vector3<T> &_radii, const Material &_mat)
: radii(_radii), material(_mat) {}

//////////////////////////////////////////////////
template<typename T>
Vector3<T> Ellipsoid<T>::Radii() const
{
  return this->radii;
}

//////////////////////////////////////////////////
template<typename T>
void Ellipsoid<T>::SetRadii(const Vector3<T> &_radii)
{
  this->radii = _radii;
}

//////////////////////////////////////////////////
template<typename T>
const Material &Ellipsoid<T>::Mat() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Ellipsoid<T>::SetMat(const Material &_mat)
{
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
bool Ellipsoid<T>::operator==(const Ellipsoid &_ellipsoid) const
{
  return this->radii == _ellipsoid.radii &&
    this->material == _ellipsoid.material;
}

//////////////////////////////////////////////////
template<typename T>
std::optional< MassMatrix3<T> > Ellipsoid<T>::MassMatrix() const
{
  if (this->radii.X() <= 0 || this->radii.Y() <= 0 || this->radii.Z() <= 0)
    return std::nullopt;

  // mass and inertia of ellipsoid taken from
  // https://en.wikipedia.org/wiki/Ellipsoid
  const T mass = this->material.Density() * this->Volume();
  const T x2 = std::pow(this->radii.X(), 2);
  const T y2 = std::pow(this->radii.Y(), 2);
  const T z2 = std::pow(this->radii.Z(), 2);
  const T ixx = (mass / 5.) * (y2 + z2);
  const T iyy = (mass / 5.) * (x2 + z2);
  const T izz = (mass / 5.) * (x2 + y2);
  return std::make_optional<MassMatrix3<T>>(
    mass, Vector3(ixx, iyy, izz), Vector3<T>::Zero);
}

//////////////////////////////////////////////////
template<typename T>
T Ellipsoid<T>::Volume() const
{
  const T kFourThirdsPi = 4. * GZ_PI / 3.;
  return kFourThirdsPi * this->radii.X() * this->radii.Y() * this->radii.Z();
}

//////////////////////////////////////////////////
template<typename T>
bool Ellipsoid<T>::SetDensityFromMass(const T _mass)
{
  T newDensity = this->DensityFromMass(_mass);
  if (isnan(newDensity))
    return false;

  this->material.SetDensity(newDensity);
  return true;
}

//////////////////////////////////////////////////
template<typename T>
T Ellipsoid<T>::DensityFromMass(const T _mass) const
{
  if (this->radii.X() <= 0 || this->radii.Y() <= 0 || this->radii.Z() <=0
    || _mass <= 0)
    return std::numeric_limits<T>::quiet_NaN();

  return _mass / this->Volume();
}

}
}
#endif

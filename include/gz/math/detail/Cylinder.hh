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
#ifndef IGNITION_MATH_DETAIL_CYLINDER_HH_
#define IGNITION_MATH_DETAIL_CYLINDER_HH_
namespace ignition
{
namespace math
{

//////////////////////////////////////////////////
template<typename T>
Cylinder<T>::Cylinder(const T _length, const T _radius,
    const Quaternion<T> &_rotOffset)
{
  this->length = _length;
  this->radius = _radius;
  this->rotOffset = _rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
Cylinder<T>::Cylinder(const T _length, const T _radius,
    const Material &_mat, const Quaternion<T> &_rotOffset)
{
  this->length = _length;
  this->radius = _radius;
  this->material = _mat;
  this->rotOffset = _rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::Radius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
template<typename T>
void Cylinder<T>::SetRadius(const T _radius)
{
  this->radius = _radius;
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::Length() const
{
  return this->length;
}

//////////////////////////////////////////////////
template<typename T>
void Cylinder<T>::SetLength(const T _length)
{
  this->length = _length;
}

//////////////////////////////////////////////////
template<typename T>
Quaternion<T> Cylinder<T>::RotationalOffset() const
{
  return this->rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
void Cylinder<T>::SetRotationalOffset(const Quaternion<T> &_rotOffset)
{
  this->rotOffset = _rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
const Material &Cylinder<T>::Mat() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Cylinder<T>::SetMat(const Material &_mat)
{
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
bool Cylinder<T>::operator==(const Cylinder &_cylinder) const
{
  return equal(this->radius, _cylinder.Radius()) &&
    equal(this->length, _cylinder.Length()) &&
    this->material == _cylinder.Mat();
}

//////////////////////////////////////////////////
template<typename T>
bool Cylinder<T>::MassMatrix(MassMatrix3d &_massMat) const
{
  return _massMat.SetFromCylinderZ(
      this->material, this->length,
      this->radius, this->rotOffset);
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::Volume() const
{
  return IGN_PI * std::pow(this->radius, 2) *
         this->length;
}

//////////////////////////////////////////////////
template<typename T>
bool Cylinder<T>::SetDensityFromMass(const T _mass)
{
  T newDensity = this->DensityFromMass(_mass);
  if (newDensity > 0)
    this->material.SetDensity(newDensity);
  return newDensity > 0;
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::DensityFromMass(const T _mass) const
{
  if (this->radius <= 0 || this->length <=0 || _mass <= 0)
    return -1.0;

  return _mass / this->Volume();
}

}
}
#endif

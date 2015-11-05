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
#include "ignition/math/MassMatrix3.hh"

using namespace ignition;
using namespace math;

//////////////////////////////////////////////////
MassMatrix3::MassMatrix3()
{
  this->mass = 1;
  this->principals.Set(1, 1, 1);
  this->products.Set(0, 0, 0);
}

//////////////////////////////////////////////////
MassMatrix3::MassMatrix3(double _m)
{
  this->mass = _m;
  this->principals.Set(1, 1, 1);
  this->products.Set(0, 0, 0);
}

//////////////////////////////////////////////////
MassMatrix3::MassMatrix3(const MassMatrix3 &_inertial)
{
  (*this) = _inertial;
}

//////////////////////////////////////////////////
MassMatrix3::~MassMatrix3()
{
}

//////////////////////////////////////////////////
MassMatrix3 MassMatrix3::GetMassMatrix3(const math::Pose &_frameOffset) const
{
  // make a copy of the current MassMatrix3
  MassMatrix3 result(*this);

  // new MOI after link frame offset
  result.SetMOI(this->GetMOI(result.cog));

  return result;
}

//////////////////////////////////////////////////
void MassMatrix3::SetMass(double _m)
{
  this->mass = _m;
}

//////////////////////////////////////////////////
double MassMatrix3::GetMass() const
{
  return this->mass;
}

//////////////////////////////////////////////////
void MassMatrix3::SetInertiaMatrix(double ixx, double iyy, double izz,
                                double ixy, double ixz, double iyz)
{
  this->principals.Set(ixx, iyy, izz);
  this->products.Set(ixy, ixz, iyz);
}


//////////////////////////////////////////////////
math::Vector3 MassMatrix3::GetPrincipalMoments() const
{
  return this->principals;
}

//////////////////////////////////////////////////
math::Vector3 MassMatrix3::GetProductsofInertia() const
{
  return this->products;
}

//////////////////////////////////////////////////
void MassMatrix3::SetMOI(const math::Matrix3 &_moi)
{
  /// \TODO: check symmetry of incoming _moi matrix
  this->principals.Set(_moi[0][0], _moi[1][1], _moi[2][2]);
  this->products.Set(_moi[0][1], _moi[0][2], _moi[1][2]);
}

//////////////////////////////////////////////////
math::Matrix3 MassMatrix3::GetMOI() const
{
  return math::Matrix3(
    this->principals.x, this->products.x,   this->products.y,
    this->products.x,   this->principals.y, this->products.z,
    this->products.y,   this->products.z,   this->principals.z);
}

//////////////////////////////////////////////////
MassMatrix3 &MassMatrix3::operator=(const MassMatrix3 &_inertial)
{
  this->mass = _inertial.mass;
  this->cog = _inertial.cog;
  this->principals = _inertial.principals;
  this->products = _inertial.products;

  return *this;
}

//////////////////////////////////////////////////
double MassMatrix3::GetIXX() const
{
  return this->principals.x;
}

//////////////////////////////////////////////////
double MassMatrix3::GetIYY() const
{
  return this->principals.y;
}

//////////////////////////////////////////////////
double MassMatrix3::GetIZZ() const
{
  return this->principals.z;
}

//////////////////////////////////////////////////
double MassMatrix3::GetIXY() const
{
  return this->products.x;
}

//////////////////////////////////////////////////
double MassMatrix3::GetIXZ() const
{
  return this->products.y;
}

//////////////////////////////////////////////////
double MassMatrix3::GetIYZ() const
{
  return this->products.z;
}

//////////////////////////////////////////////////
void MassMatrix3::SetIXX(double _v)
{
  this->principals.x = _v;
}

//////////////////////////////////////////////////
void MassMatrix3::SetIYY(double _v)
{
  this->principals.y = _v;
}

//////////////////////////////////////////////////
void MassMatrix3::SetIZZ(double _v)
{
  this->principals.z = _v;
}

//////////////////////////////////////////////////
void MassMatrix3::SetIXY(double _v)
{
  this->products.x = _v;
}

//////////////////////////////////////////////////
void MassMatrix3::SetIXZ(double _v)
{
  this->products.y = _v;
}

//////////////////////////////////////////////////
void MassMatrix3::SetIYZ(double _v)
{
  this->products.z = _v;
}


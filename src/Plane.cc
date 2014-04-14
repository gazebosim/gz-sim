/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "ignition/math/Plane.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
Plane::Plane()
{
  this->d = 0.0;
}

/////////////////////////////////////////////////
Plane::Plane(const Vector3d &_normal, double _offset)
{
  this->normal = _normal;
  this->d = _offset;
}

/////////////////////////////////////////////////
Plane::Plane(const Vector3d &_normal, const Vector2d &_size, double _offset)
{
  this->Set(_normal, _size, _offset);
}

/////////////////////////////////////////////////
Plane::~Plane()
{
}

/////////////////////////////////////////////////
void Plane::Set(const Vector3d &_n, const Vector2d &_s, double _offset)
{
  this->normal = _n;
  this->size = _s;
  this->d = _offset;
}

//////////////////////////////////////////////////
Plane &Plane::operator =(const Plane & _p)
{
  this->normal = _p.normal;
  this->size = _p.size;
  this->d = _p.d;

  return *this;
}

//////////////////////////////////////////////////
double Plane::Distance(const Vector3d &_origin, const Vector3d &_dir) const
{
  double denom = this->normal.Dot(_dir);

  if (std::abs(denom) < 1e-3)
  {
    // parallel
    return 0;
  }
  else
  {
    double nom = _origin.Dot(this->normal) - this->d;
    double t = -(nom/denom);
    return t;
  }
}

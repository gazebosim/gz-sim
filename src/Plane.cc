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

/////////////////////////////////////////////////
IGN_PLANE::IGN_PLANE()
{
  this->d = 0.0;
}

/////////////////////////////////////////////////
IGN_PLANE::IGN_PLANE(const IGN_VECTOR3 &_normal, IGN_NUMERIC _offset)
{
  this->normal = _normal;
  this->d = _offset;
}

/////////////////////////////////////////////////
IGN_PLANE::IGN_PLANE(const IGN_VECTOR3 &_normal,
    const IGN_VECTOR2 &_size, IGN_NUMERIC _offset)
{
  this->Set(_normal, _size, _offset);
}

/////////////////////////////////////////////////
IGN_PLANE::~IGN_PLANE()
{
}

/////////////////////////////////////////////////
void IGN_PLANE::Set(const IGN_VECTOR3 &_n, const IGN_VECTOR2 &_s,
    IGN_NUMERIC _offset)
{
  this->normal = _n;
  this->size = _s;
  this->d = _offset;
}

//////////////////////////////////////////////////
IGN_PLANE &IGN_PLANE::operator =(const IGN_PLANE & _p)
{
  this->normal = _p.normal;
  this->size = _p.size;
  this->d = _p.d;

  return *this;
}

//////////////////////////////////////////////////
IGN_NUMERIC IGN_PLANE::Distance(const IGN_VECTOR3 &_origin,
    const IGN_VECTOR3 &_dir) const
{
  IGN_NUMERIC denom = this->normal.Dot(_dir);

  if (fabs(denom) < 1e-3)
  {
    // parallel
    return 0;
  }
  else
  {
    IGN_NUMERIC nom = _origin.Dot(this->normal) - this->d;
    IGN_NUMERIC t = -(nom/denom);
    return t;
  }
}

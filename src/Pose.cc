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
/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 */

#include "ignition/math/Pose.hh"

using namespace ignition;
using namespace math;

const Pose Pose::Zero = math::Pose(0, 0, 0, 0, 0, 0);

//////////////////////////////////////////////////
Pose::Pose()
  : pos(0, 0, 0), rot(1, 0, 0, 0)
{
}

//////////////////////////////////////////////////
Pose::Pose(const Vector3 &_pos, const Quaternion &_rot)
  : pos(_pos), rot(_rot)
{
}

//////////////////////////////////////////////////
Pose::Pose(double _x, double _y, double _z,
           double _roll, double _pitch, double _yaw)
: pos(_x, _y, _z), rot(_roll, _pitch, _yaw)
{
}

//////////////////////////////////////////////////
Pose::Pose(double _x, double _y, double _z,
           double _qw, double _qx, double _qy, double _qz)
: pos(_x, _y, _z), rot(_qw, _qx, _qy, _qz)
{
}

//////////////////////////////////////////////////
Pose::Pose(const Pose &_pose)
  : pos(_pose.pos), rot(_pose.rot)
{
}

//////////////////////////////////////////////////
Pose::~Pose()
{
}

//////////////////////////////////////////////////
void Pose::Set(const Vector3 &_pos, const Quaternion &_rot)
{
  this->pos = _pos;
  this->rot = _rot;
}

//////////////////////////////////////////////////
void Pose::Set(const Vector3 &_pos, const Vector3 &_rpy)
{
  this->pos = _pos;
  this->rot.SetFromEuler(_rpy);
}

//////////////////////////////////////////////////
void Pose::Set(double _x, double _y, double _z,
               double _roll, double _pitch, double _yaw)
{
  this->pos.Set(_x, _y, _z);
  this->rot.SetFromEuler(math::Vector3(_roll, _pitch, _yaw));
}

//////////////////////////////////////////////////
bool Pose::IsFinite() const
{
  return this->pos.IsFinite() && this->rot.IsFinite();
}

//////////////////////////////////////////////////
Pose Pose::GetInverse() const
{
  Quaternion inv = this->rot.GetInverse();
  return Pose(inv * (this->pos*-1), inv);
}

//////////////////////////////////////////////////
Pose Pose::operator+(const Pose &_obj) const
{
  Pose result;

  result.pos = this->CoordPositionAdd(_obj);
  result.rot = this->CoordRotationAdd(_obj.rot);

  return result;
}

//////////////////////////////////////////////////
const Pose &Pose::operator+=(const Pose &obj)
{
  this->pos = this->CoordPositionAdd(obj);
  this->rot = this->CoordRotationAdd(obj.rot);

  return *this;
}

//////////////////////////////////////////////////
const Pose &Pose::operator-=(const Pose &_obj)
{
  this->pos = this->CoordPositionSub(_obj);
  this->rot = this->CoordRotationSub(_obj.rot);

  return *this;
}

//////////////////////////////////////////////////
bool Pose::operator ==(const Pose &_pose) const
{
  return this->pos == _pose.pos && this->rot == _pose.rot;
}

//////////////////////////////////////////////////
bool Pose::operator!=(const Pose &_pose) const
{
  return this->pos != _pose.pos || this->rot != _pose.rot;
}

//////////////////////////////////////////////////
Pose Pose::operator*(const Pose &pose)
{
  return Pose(this->CoordPositionAdd(pose),  pose.rot * this->rot);
}

//////////////////////////////////////////////////
Pose &Pose::operator=(const Pose &_pose)
{
  this->pos = _pose.pos;
  this->rot = _pose.rot;
  return *this;
}

//////////////////////////////////////////////////
Vector3 Pose::CoordPositionAdd(const Vector3 &_pos) const
{
  Quaternion tmp;
  Vector3 result;

  // result = pose.rot + pose.rot * this->_pos * pose.rot!
  tmp.w = 0.0;
  tmp.x = _pos.x;
  tmp.y = _pos.y;
  tmp.z = _pos.z;

  tmp = this->rot * (tmp * this->rot.GetInverse());
  result.x = this->pos.x + tmp.x;
  result.y = this->pos.y + tmp.y;
  result.z = this->pos.z + tmp.z;

  return result;
}

//////////////////////////////////////////////////
Vector3 Pose::CoordPositionAdd(const Pose &_pose) const
{
  Quaternion tmp;
  Vector3 result;

  // result = _pose.rot + _pose.rot * this->pos * _pose.rot!
  tmp.w = 0.0;
  tmp.x = this->pos.x;
  tmp.y = this->pos.y;
  tmp.z = this->pos.z;

  tmp = _pose.rot * (tmp * _pose.rot.GetInverse());

  result.x = _pose.pos.x + tmp.x;
  result.y = _pose.pos.y + tmp.y;
  result.z = _pose.pos.z + tmp.z;

  return result;
}

//////////////////////////////////////////////////
Quaternion Pose::CoordRotationAdd(const Quaternion &_rot) const
{
  return Quaternion(_rot * this->rot);
}

//////////////////////////////////////////////////
void Pose::Reset()
{
  // set the position to zero
  this->pos.Set();
  this->rot.SetToIdentity();
}


//////////////////////////////////////////////////
Pose Pose::CoordPoseSolve(const Pose &_b) const
{
  Quaternion q;
  Pose a;

  a.rot = this->rot.GetInverse() * _b.rot;
  q = a.rot * Quaternion(0, this->pos.x, this->pos.y, this->pos.z);
  q = q * a.rot.GetInverse();
  a.pos = _b.pos - Vector3(q.x, q.y, q.z);

  return a;
}

//////////////////////////////////////////////////
Pose Pose::RotatePositionAboutOrigin(const Quaternion &_rot) const
{
  Pose a = *this;
  a.pos.x =  (1.0 - 2.0*_rot.y*_rot.y - 2.0*_rot.z*_rot.z) * this->pos.x
            +(2.0*(_rot.x*_rot.y+_rot.w*_rot.z)) * this->pos.y
            +(2.0*(_rot.x*_rot.z-_rot.w*_rot.y)) * this->pos.z;
  a.pos.y =  (2.0*(_rot.x*_rot.y-_rot.w*_rot.z)) * this->pos.x
            +(1.0 - 2.0*_rot.x*_rot.x - 2.0*_rot.z*_rot.z) * this->pos.y
            +(2.0*(_rot.y*_rot.z+_rot.w*_rot.x)) * this->pos.z;
  a.pos.z =  (2.0*(_rot.x*_rot.z+_rot.w*_rot.y)) * this->pos.x
            +(2.0*(_rot.y*_rot.z-_rot.w*_rot.x)) * this->pos.y
            +(1.0 - 2.0*_rot.x*_rot.x - 2.0*_rot.y*_rot.y) * this->pos.z;
  return a;
}

//////////////////////////////////////////////////
void Pose::Round(int _precision)
{
  this->rot.Round(_precision);
  this->pos.Round(_precision);
}


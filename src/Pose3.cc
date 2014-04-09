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

const IGN_POSE3 IGN_POSE3::Zero = math::IGN_POSE3(0, 0, 0, 0, 0, 0);

//////////////////////////////////////////////////
IGN_POSE3::IGN_POSE3()
  : p(0, 0, 0), q(1, 0, 0, 0)
{
}

//////////////////////////////////////////////////
IGN_POSE3::IGN_POSE3(const IGN_VECTOR3 &_pos, const IGN_QUATERNION &_rot)
  : p(_pos), q(_rot)
{
}

//////////////////////////////////////////////////
IGN_POSE3::IGN_POSE3(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z,
    IGN_NUMERIC _roll, IGN_NUMERIC _pitch, IGN_NUMERIC _yaw)
: p(_x, _y, _z), q(_roll, _pitch, _yaw)
{
}

//////////////////////////////////////////////////
IGN_POSE3::IGN_POSE3(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z,
    IGN_NUMERIC _qw, IGN_NUMERIC _qx, IGN_NUMERIC _qy, IGN_NUMERIC _qz)
: p(_x, _y, _z), q(_qw, _qx, _qy, _qz)
{
}

//////////////////////////////////////////////////
IGN_POSE3::IGN_POSE3(const IGN_POSE3 &_pose)
  : p(_pose.p), q(_pose.q)
{
}

//////////////////////////////////////////////////
IGN_POSE3::~IGN_POSE3()
{
}

//////////////////////////////////////////////////
void IGN_POSE3::Set(const IGN_VECTOR3 &_pos, const IGN_QUATERNION &_rot)
{
  this->p = _pos;
  this->q = _rot;
}

//////////////////////////////////////////////////
void IGN_POSE3::Set(const IGN_VECTOR3 &_pos, const IGN_VECTOR3 &_rpy)
{
  this->p = _pos;
  this->q.SetFromEuler(_rpy);
}

//////////////////////////////////////////////////
void IGN_POSE3::Set(IGN_NUMERIC _x, IGN_NUMERIC _y, IGN_NUMERIC _z,
    IGN_NUMERIC _roll, IGN_NUMERIC _pitch, IGN_NUMERIC _yaw)
{
  this->p.Set(_x, _y, _z);
  this->q.SetFromEuler(math::IGN_VECTOR3(_roll, _pitch, _yaw));
}

//////////////////////////////////////////////////
bool IGN_POSE3::IsFinite() const
{
  return this->p.IsFinite() && this->q.IsFinite();
}

//////////////////////////////////////////////////
IGN_POSE3 IGN_POSE3::GetInverse() const
{
  IGN_QUATERNION inv = this->q.GetInverse();
  return IGN_POSE3(inv * (this->p*-1), inv);
}

//////////////////////////////////////////////////
IGN_POSE3 IGN_POSE3::operator+(const IGN_POSE3 &_obj) const
{
  IGN_POSE3 result;

  result.p = this->CoordPositionAdd(_obj);
  result.q = this->CoordRotationAdd(_obj.q);

  return result;
}

//////////////////////////////////////////////////
const IGN_POSE3 &IGN_POSE3::operator+=(const IGN_POSE3 &obj)
{
  this->p = this->CoordPositionAdd(obj);
  this->q = this->CoordRotationAdd(obj.q);

  return *this;
}

//////////////////////////////////////////////////
const IGN_POSE3 &IGN_POSE3::operator-=(const IGN_POSE3 &_obj)
{
  this->p = this->CoordPositionSub(_obj);
  this->q = this->CoordRotationSub(_obj.q);

  return *this;
}

//////////////////////////////////////////////////
bool IGN_POSE3::operator ==(const IGN_POSE3 &_pose) const
{
  return this->p == _pose.p && this->q == _pose.q;
}

//////////////////////////////////////////////////
bool IGN_POSE3::operator!=(const IGN_POSE3 &_pose) const
{
  return this->p != _pose.p || this->q != _pose.q;
}

//////////////////////////////////////////////////
IGN_POSE3 IGN_POSE3::operator*(const IGN_POSE3 &pose)
{
  return IGN_POSE3(this->CoordPositionAdd(pose),  pose.q * this->q);
}

//////////////////////////////////////////////////
IGN_POSE3 &IGN_POSE3::operator=(const IGN_POSE3 &_pose)
{
  this->p = _pose.p;
  this->q = _pose.q;
  return *this;
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_POSE3::CoordPositionAdd(const IGN_VECTOR3 &_pos) const
{
  IGN_QUATERNION tmp(0.0, _pos.x(), _pos.y(), _pos.z());

  // result = pose.q + pose.q * this->p * pose.q!
  tmp = this->q * (tmp * this->q.GetInverse());

  return IGN_VECTOR3(this->p.x() + tmp.x(),
                     this->p.y() + tmp.y(),
                     this->p.z() + tmp.z());
}

//////////////////////////////////////////////////
IGN_VECTOR3 IGN_POSE3::CoordPositionAdd(const IGN_POSE3 &_pose) const
{
  IGN_QUATERNION tmp(static_cast<IGN_NUMERIC>(0),
      this->p.x(), this->p.y(), this->p.z());

  // result = _pose.q + _pose.q * this->p * _pose.q!
  tmp = _pose.q * (tmp * _pose.q.GetInverse());

  return IGN_VECTOR3(_pose.p.x() + tmp.x(),
                     _pose.p.y() + tmp.y(),
                     _pose.p.z() + tmp.z());
}

//////////////////////////////////////////////////
IGN_QUATERNION IGN_POSE3::CoordRotationAdd(const IGN_QUATERNION &_rot) const
{
  return IGN_QUATERNION(_rot * this->q);
}

//////////////////////////////////////////////////
void IGN_POSE3::Reset()
{
  // set the position to zero
  this->p.Set();
  this->q.SetToIdentity();
}


//////////////////////////////////////////////////
IGN_POSE3 IGN_POSE3::CoordPoseSolve(const IGN_POSE3 &_b) const
{
  IGN_QUATERNION qt;
  IGN_POSE3 a;

  a.q = this->q.GetInverse() * _b.q;
  qt = a.q * IGN_QUATERNION(0, this->p.x(), this->p.y(), this->p.z());
  qt = qt * a.q.GetInverse();
  a.p = _b.p - IGN_VECTOR3(qt.x(), qt.y(), qt.z());

  return a;
}

//////////////////////////////////////////////////
IGN_POSE3 IGN_POSE3::RotatePositionAboutOrigin(const IGN_QUATERNION &_q) const
{
  IGN_POSE3 a = *this;
  a.p.x((1.0 - 2.0*_q.y()*_q.y() - 2.0*_q.z()*_q.z()) * this->p.x()
          +(2.0*(_q.x()*_q.y()+_q.w()*_q.z())) * this->p.y()
          +(2.0*(_q.x()*_q.z()-_q.w()*_q.y())) * this->p.z());
  a.p.y((2.0*(_q.x()*_q.y()-_q.w()*_q.z())) * this->p.x()
          +(1.0 - 2.0*_q.x()*_q.x() - 2.0*_q.z()*_q.z()) * this->p.y()
          +(2.0*(_q.y()*_q.z()+_q.w()*_q.x())) * this->p.z());
  a.p.z((2.0*(_q.x()*_q.z()+_q.w()*_q.y())) * this->p.x()
          +(2.0*(_q.y()*_q.z()-_q.w()*_q.x())) * this->p.y()
          +(1.0 - 2.0*_q.x()*_q.x() - 2.0*_q.y()*_q.y()) * this->p.z());
  return a;
}

//////////////////////////////////////////////////
void IGN_POSE3::Round(int _precision)
{
  this->q.Round(_precision);
  this->p.Round(_precision);
}

//////////////////////////////////////////////////
std::ostream &ignition::math::operator<<(std::ostream &_out,
    const ignition::math::IGN_POSE3 &_pose)
{
  _out << _pose.pos() << " " << _pose.rot();
  return _out;
}

//////////////////////////////////////////////////
std::istream &ignition::math::operator>>(std::istream &_in,
    ignition::math::IGN_POSE3 &_pose)
{
  // Skip white spaces
  _in.setf(std::ios_base::skipws);
  IGN_VECTOR3 pos;
  IGN_QUATERNION rot;
  _in >> pos >> rot;
  _pose.Set(pos, rot);
  return _in;
}

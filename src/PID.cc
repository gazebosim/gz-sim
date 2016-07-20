/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <chrono>
#include <cmath>
#include "ignition/math/Helpers.hh"
#include "ignition/math/PID.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
PID::PID(const double _p, const double _i, const double _d,
         const double _imax, const double _imin, const double _cmdMax,
         const double _cmdMin, const double _cmdOffset)
: pGain(_p), iGain(_i), dGain(_d), iMax(_imax), iMin(_imin),
  cmdMax(_cmdMax), cmdMin(_cmdMin), cmdOffset(_cmdOffset)
{
  this->Reset();
}

/////////////////////////////////////////////////
void PID::Init(const double _p, const double _i, const double _d,
               const double _imax, const double _imin, const double _cmdMax,
               const double _cmdMin, const double _cmdOffset)
{
  this->pGain = _p;
  this->iGain = _i;
  this->dGain = _d;
  this->iMax = _imax;
  this->iMin = _imin;
  this->cmdMax = _cmdMax;
  this->cmdMin = _cmdMin;
  this->cmdOffset = _cmdOffset;

  this->Reset();
}

/////////////////////////////////////////////////
PID &PID::operator=(const PID &_p)
{
  if (this == &_p)
    return *this;

  this->pGain = _p.pGain;
  this->iGain = _p.iGain;
  this->dGain = _p.dGain;
  this->iMax = _p.iMax;
  this->iMin = _p.iMin;
  this->cmdMax = _p.cmdMax;
  this->cmdMin = _p.cmdMin;
  this->cmdOffset = _p.cmdOffset;
  this->pErrLast = _p.pErrLast;
  this->pErr = _p.pErr;
  this->iErr = _p.iErr;
  this->dErr = _p.dErr;
  this->cmd = _p.cmd;

  return *this;
}

/////////////////////////////////////////////////
void PID::SetPGain(const double _p)
{
  this->pGain = _p;
}

/////////////////////////////////////////////////
void PID::SetIGain(const double _i)
{
  this->iGain = _i;
}

/////////////////////////////////////////////////
void PID::SetDGain(const double _d)
{
  this->dGain = _d;
}

/////////////////////////////////////////////////
void PID::SetIMax(const double _i)
{
  this->iMax = _i;
}

/////////////////////////////////////////////////
void PID::SetIMin(const double _i)
{
  this->iMin = _i;
}

/////////////////////////////////////////////////
void PID::SetCmdMax(const double _c)
{
  this->cmdMax = _c;
}

/////////////////////////////////////////////////
void PID::SetCmdMin(const double _c)
{
  this->cmdMin = _c;
}

/////////////////////////////////////////////////
void PID::SetCmdOffset(const double _c)
{
  this->cmdOffset = _c;
}

/////////////////////////////////////////////////
void PID::Reset()
{
  this->pErrLast = 0.0;
  this->pErr = 0.0;
  this->iErr = 0.0;
  this->dErr = 0.0;
  this->cmd = 0.0;
}

/////////////////////////////////////////////////
double PID::Update(const double _error,
                   const std::chrono::duration<double> &_dt)
{
  if (_dt == std::chrono::duration<double>(0) ||
      ignition::math::isnan(_error) || std::isinf(_error))
  {
    return 0.0;
  }

  double pTerm, dTerm;
  this->pErr = _error;

  // Calculate proportional contribution to command
  pTerm = this->pGain * this->pErr;

  // Calculate the integral error
  this->iErr = this->iErr + this->iGain * _dt.count() * this->pErr;

  // Check the integral limits
  // If enabled, this will limit iErr so that the limit is meaningful
  // in the output
  if (this->iMax >= this->iMin)
    this->iErr = clamp(this->iErr, this->iMin, this->iMax);

  // Calculate the derivative error
  if (_dt != std::chrono::duration<double>(0))
  {
    this->dErr = (this->pErr - this->pErrLast) / _dt.count();
    this->pErrLast = this->pErr;
  }

  // Calculate derivative contribution to command
  dTerm = this->dGain * this->dErr;
  this->cmd = this->cmdOffset -pTerm - this->iErr - dTerm;

  // Check the command limits
  if (this->cmdMax >= this->cmdMin)
    this->cmd = clamp(this->cmd, this->cmdMin, this->cmdMax);

  return this->cmd;
}

/////////////////////////////////////////////////
void PID::SetCmd(const double _cmd)
{
  this->cmd = _cmd;
}

/////////////////////////////////////////////////
double PID::Cmd() const
{
  return this->cmd;
}

/////////////////////////////////////////////////
void PID::Errors(double &_pe, double &_ie, double &_de) const
{
  _pe = this->pErr;
  _ie = this->iErr;
  _de = this->dErr;
}

/////////////////////////////////////////////////
double PID::PGain() const
{
  return this->pGain;
}

/////////////////////////////////////////////////
double PID::IGain() const
{
  return this->iGain;
}

/////////////////////////////////////////////////
double PID::DGain() const
{
  return this->dGain;
}

/////////////////////////////////////////////////
double PID::IMax() const
{
  return this->iMax;
}

/////////////////////////////////////////////////
double PID::IMin() const
{
  return this->iMin;
}

/////////////////////////////////////////////////
double PID::CmdMax() const
{
  return this->cmdMax;
}

/////////////////////////////////////////////////
double PID::CmdMin() const
{
  return this->cmdMin;
}

/////////////////////////////////////////////////
double PID::CmdOffset() const
{
  return this->cmdOffset;
}

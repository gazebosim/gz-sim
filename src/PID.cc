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
#include "gz/math/Helpers.hh"
#include "gz/math/PID.hh"

using namespace gz;
using namespace math;

/////////////////////////////////////////////////
class PID::Implementation
{
  /// \brief Error at a previous step.
  public: double pErrLast = 0.0;

  /// \brief Current error.
  public: double pErr = 0.0;

  /// \brief Integral of gain times error.
  public: double iErr = 0.0;

  /// \brief Derivative error.
  public: double dErr = 0.0;

  /// \brief Gain for proportional control.
  public: double pGain;

  /// \brief Gain for integral control.
  public: double iGain = 0.0;

  /// \brief Gain for derivative control.
  public: double dGain = 0.0;

  /// \brief Maximum clamping value for integral term.
  public: double iMax = -1.0;

  /// \brief Minim clamping value for integral term.
  public: double iMin = 0.0;

  /// \brief Command value.
  public: double cmd = 0.0;

  /// \brief Max command clamping value.
  public: double cmdMax = -1.0;

  /// \brief Min command clamping value.
  public: double cmdMin = 0.0;

  /// \brief Command offset.
  public: double cmdOffset = 0.0;
};

/////////////////////////////////////////////////
PID::PID(const double _p, const double _i, const double _d,
         const double _imax, const double _imin, const double _cmdMax,
         const double _cmdMin, const double _cmdOffset)
: dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->Init(_p, _i, _d, _imax, _imin, _cmdMax, _cmdMin, _cmdOffset);
}

/////////////////////////////////////////////////
void PID::Init(const double _p, const double _i, const double _d,
               const double _imax, const double _imin, const double _cmdMax,
               const double _cmdMin, const double _cmdOffset)
{
  this->dataPtr->pGain = _p;
  this->dataPtr->iGain = _i;
  this->dataPtr->dGain = _d;
  this->dataPtr->iMax = _imax;
  this->dataPtr->iMin = _imin;
  this->dataPtr->cmdMax = _cmdMax;
  this->dataPtr->cmdMin = _cmdMin;
  this->dataPtr->cmdOffset = _cmdOffset;

  this->Reset();
}

/////////////////////////////////////////////////
void PID::SetPGain(const double _p)
{
  this->dataPtr->pGain = _p;
}

/////////////////////////////////////////////////
void PID::SetIGain(const double _i)
{
  this->dataPtr->iGain = _i;
}

/////////////////////////////////////////////////
void PID::SetDGain(const double _d)
{
  this->dataPtr->dGain = _d;
}

/////////////////////////////////////////////////
void PID::SetIMax(const double _i)
{
  this->dataPtr->iMax = _i;
}

/////////////////////////////////////////////////
void PID::SetIMin(const double _i)
{
  this->dataPtr->iMin = _i;
}

/////////////////////////////////////////////////
void PID::SetCmdMax(const double _c)
{
  this->dataPtr->cmdMax = _c;
}

/////////////////////////////////////////////////
void PID::SetCmdMin(const double _c)
{
  this->dataPtr->cmdMin = _c;
}

/////////////////////////////////////////////////
void PID::SetCmdOffset(const double _c)
{
  this->dataPtr->cmdOffset = _c;
}

/////////////////////////////////////////////////
void PID::Reset()
{
  this->dataPtr->pErrLast = 0.0;
  this->dataPtr->pErr = 0.0;
  this->dataPtr->iErr = 0.0;
  this->dataPtr->dErr = 0.0;
  this->dataPtr->cmd = 0.0;
}

/////////////////////////////////////////////////
double PID::Update(const double _error,
                   const std::chrono::duration<double> &_dt)
{
  if (_dt == std::chrono::duration<double>(0) ||
      gz::math::isnan(_error) || std::isinf(_error))
  {
    return 0.0;
  }

  double pTerm, dTerm;
  this->dataPtr->pErr = _error;

  // Calculate proportional contribution to command
  pTerm = this->dataPtr->pGain * this->dataPtr->pErr;

  // Calculate the integral error
  this->dataPtr->iErr = this->dataPtr->iErr +
    this->dataPtr->iGain * _dt.count() * this->dataPtr->pErr;

  // Check the integral limits
  // If enabled, this will limit iErr so that the limit is meaningful
  // in the output
  if (this->dataPtr->iMax >= this->dataPtr->iMin)
    this->dataPtr->iErr = clamp(this->dataPtr->iErr,
                                this->dataPtr->iMin,
                                this->dataPtr->iMax);

  // Calculate the derivative error
  if (_dt != std::chrono::duration<double>(0))
  {
    this->dataPtr->dErr =
      (this->dataPtr->pErr - this->dataPtr->pErrLast) / _dt.count();
    this->dataPtr->pErrLast = this->dataPtr->pErr;
  }

  // Calculate derivative contribution to command
  dTerm = this->dataPtr->dGain * this->dataPtr->dErr;
  this->dataPtr->cmd =
    this->dataPtr->cmdOffset - pTerm - this->dataPtr->iErr - dTerm;

  // Check the command limits
  if (this->dataPtr->cmdMax >= this->dataPtr->cmdMin)
    this->dataPtr->cmd =
      clamp(this->dataPtr->cmd, this->dataPtr->cmdMin, this->dataPtr->cmdMax);

  return this->dataPtr->cmd;
}

/////////////////////////////////////////////////
void PID::SetCmd(const double _cmd)
{
  this->dataPtr->cmd = _cmd;
}

/////////////////////////////////////////////////
double PID::Cmd() const
{
  return this->dataPtr->cmd;
}

/////////////////////////////////////////////////
void PID::Errors(double &_pe, double &_ie, double &_de) const
{
  _pe = this->dataPtr->pErr;
  _ie = this->dataPtr->iErr;
  _de = this->dataPtr->dErr;
}

/////////////////////////////////////////////////
double PID::PGain() const
{
  return this->dataPtr->pGain;
}

/////////////////////////////////////////////////
double PID::IGain() const
{
  return this->dataPtr->iGain;
}

/////////////////////////////////////////////////
double PID::DGain() const
{
  return this->dataPtr->dGain;
}

/////////////////////////////////////////////////
double PID::IMax() const
{
  return this->dataPtr->iMax;
}

/////////////////////////////////////////////////
double PID::IMin() const
{
  return this->dataPtr->iMin;
}

/////////////////////////////////////////////////
double PID::CmdMax() const
{
  return this->dataPtr->cmdMax;
}

/////////////////////////////////////////////////
double PID::CmdMin() const
{
  return this->dataPtr->cmdMin;
}

/////////////////////////////////////////////////
double PID::CmdOffset() const
{
  return this->dataPtr->cmdOffset;
}

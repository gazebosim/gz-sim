/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "ignition/math/Helpers.hh"
#include "ignition/math/SpeedLimiter.hh"

using namespace ignition;
using namespace math;

/// \brief Private SpeedLimiter data class.
class ignition::math::SpeedLimiterPrivate
{
  /// \brief Minimum velocity limit.
  public: double minVelocity{-std::numeric_limits<double>::infinity()};

  /// \brief Maximum velocity limit.
  public: double maxVelocity{std::numeric_limits<double>::infinity()};

  /// \brief Minimum acceleration limit.
  public: double minAcceleration{-std::numeric_limits<double>::infinity()};

  /// \brief Maximum acceleration limit.
  public: double maxAcceleration{std::numeric_limits<double>::infinity()};

  /// \brief Minimum jerk limit.
  public: double minJerk{-std::numeric_limits<double>::infinity()};

  /// \brief Maximum jerk limit.
  public: double maxJerk{std::numeric_limits<double>::infinity()};
};

//////////////////////////////////////////////////
SpeedLimiter::SpeedLimiter()
  : dataPtr(std::make_unique<SpeedLimiterPrivate>())
{
}

//////////////////////////////////////////////////
SpeedLimiter::~SpeedLimiter() = default;

//////////////////////////////////////////////////
void SpeedLimiter::SetMinVelocity(double _lim)
{
  this->dataPtr->minVelocity = _lim;
}

//////////////////////////////////////////////////
double SpeedLimiter::MinVelocity() const
{
  return this->dataPtr->minVelocity;
}

//////////////////////////////////////////////////
void SpeedLimiter::SetMaxVelocity(double _lim)
{
  this->dataPtr->maxVelocity = _lim;
}

//////////////////////////////////////////////////
double SpeedLimiter::MaxVelocity() const
{
  return this->dataPtr->maxVelocity;
}

//////////////////////////////////////////////////
void SpeedLimiter::SetMinAcceleration(double _lim)
{
  this->dataPtr->minAcceleration = _lim;
}

//////////////////////////////////////////////////
double SpeedLimiter::MinAcceleration() const
{
  return this->dataPtr->minAcceleration;
}

//////////////////////////////////////////////////
void SpeedLimiter::SetMaxAcceleration(double _lim)
{
  this->dataPtr->maxAcceleration = _lim;
}

//////////////////////////////////////////////////
double SpeedLimiter::MaxAcceleration() const
{
  return this->dataPtr->maxAcceleration;
}

//////////////////////////////////////////////////
void SpeedLimiter::SetMinJerk(double _lim)
{
  this->dataPtr->minJerk = _lim;
}

//////////////////////////////////////////////////
double SpeedLimiter::MinJerk() const
{
  return this->dataPtr->minJerk;
}

//////////////////////////////////////////////////
void SpeedLimiter::SetMaxJerk(double _lim)
{
  this->dataPtr->maxJerk = _lim;
}

//////////////////////////////////////////////////
double SpeedLimiter::MaxJerk() const
{
  return this->dataPtr->maxJerk;
}

//////////////////////////////////////////////////
double SpeedLimiter::Limit(double &_vel, double _prevVel, double _prevPrevVel,
    std::chrono::steady_clock::duration _dt) const
{
  const double vUnclamped = _vel;

  this->LimitJerk(_vel, _prevVel, _prevPrevVel, _dt);
  this->LimitAcceleration(_vel, _prevVel, _dt);
  this->LimitVelocity(_vel);

  return _vel - vUnclamped;
}

//////////////////////////////////////////////////
double SpeedLimiter::LimitVelocity(double &_vel) const
{
  const double vUnclamped = _vel;

  _vel = ignition::math::clamp(
    _vel, this->dataPtr->minVelocity, this->dataPtr->maxVelocity);

  return _vel - vUnclamped;
}

//////////////////////////////////////////////////
double SpeedLimiter::LimitAcceleration(double &_vel, double _prevVel,
    std::chrono::steady_clock::duration _dt) const
{
  const double vUnclamped = _vel;

  const double dtSec = std::chrono::duration<double>(_dt).count();

  if (equal(dtSec, 0.0))
    return 0.0;

  const double accUnclamped = (_vel - _prevVel) / dtSec;

  const double accClamped = ignition::math::clamp(accUnclamped,
      this->dataPtr->minAcceleration, this->dataPtr->maxAcceleration);

  _vel = _prevVel + accClamped * dtSec;

  return _vel - vUnclamped;
}

//////////////////////////////////////////////////
double SpeedLimiter::LimitJerk(double &_vel, double _prevVel,
    double _prevPrevVel, std::chrono::steady_clock::duration _dt) const
{
  const double vUnclamped = _vel;

  const double dtSec = std::chrono::duration<double>(_dt).count();

  if (equal(dtSec, 0.0))
    return 0.0;

  const double accUnclamped  = (_vel  - _prevVel) / dtSec;
  const double accPrev = (_prevVel - _prevPrevVel) / dtSec;
  const double jerkUnclamped = (accUnclamped - accPrev) / dtSec;

  const double jerkClamped = ignition::math::clamp(jerkUnclamped,
      this->dataPtr->minJerk, this->dataPtr->maxJerk);

  const double accClamped = accPrev + jerkClamped * dtSec;

  _vel = _prevVel + accClamped * dtSec;

  return _vel - vUnclamped;
}

/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <cmath>
#include "ignition/math/OdometryFourWheel.hh"
#include "ignition/math/RollingMean.hh"

using namespace ignition;
using namespace math;

// The implementation was borrowed from: https://github.com/ros-controls/ros_controllers/blob/melodic-devel/four_wheel_steering_controller/src/odometry.cpp

class ignition::math::OdometryFourWheelPrivate
{
  /// \brief Integrates the velocities (linear on x and y and angular)
  /// \param[in] _linearX Linear velocity along x of the robot frame.
  /// \param[in] _linearY  Linear velocity along y of the robot frame.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateXY(double _linearX, double _linearY, double _angular);

  /// \brief Integrates the velocities (linear and angular) using 2nd order
  /// Runge-Kutta.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateRungeKutta2(double _linear, double _angular);

  /// \brief Integrates the velocities (linear and angular) using exact
  /// method.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateExact(double _linear, double _angular);

  /// \brief Current timestamp.
  public: clock::time_point lastUpdateTime;

  /// \brief Current x position in meters.
  public: double x{0.0};

  /// \brief Current y position in meters.
  public: double y{0.0};

  /// \brief Current heading in radians.
  public: double heading{0.0};

  /// \brief Current velocity in meter/second.
  public: double linear{0.0};

  /// \brief Current velocity along X axis in meter/second.
  // public: double linearX{0.0};

  /// \brief Current velocity along Y axis meter/second.
  // public: double linearY{0.0};

  /// \brief Current angular velocity in radians/second.
  public: double angular{0.0};

  /// \brief Seperation between left and right steering joints in meters.
  // public: double steeringTrack{0.0};

  /// \brief  Offset between the steering and wheel joints in meters.
  // public: double wheelSteeringYOffset{0.0};

  /// \brief Left wheel radius in meters.
  public: double leftWheelRadius{0.0};

  /// \brief Right wheel radius in meters.
  public: double rightWheelRadius{0.0};

  /// \brief Wheel separation in meters.
  public: double wheelSeparation{0.0};

  /// \brief Previous left wheel position/state in radians.
  public: double leftWheelOldPos{0.0};

  /// \brief Previous right wheel position/state in radians.
  public: double rightWheelOldPos{0.0};

  /// \brief Rolling mean accumulators for the linear velocity
  public: RollingMean linearMean;

  /// \brief Rolling mean accumulators for the angular velocity
  public: RollingMean angularMean;

  // public: RollingMean linearJerk;
  // public: RollingMean frontSteerVel;
  // public: RollingMean rearSteerVel;
  public: double linearVelPrev;
  public: double linearAccelPrev;
  public: double frontSteerVelPrev;
  public: double rearSteerVelPrev;
};

//////////////////////////////////////////////////
OdometryFourWheel::OdometryFourWheel(size_t _windowSize)
  : dataPtr(new OdometryFourWheelPrivate)
{
  this->dataPtr->linearAcceleration.SetWindowSize(_windowSize);
  this->dataPtr->linearJerk.SetWindowSize(_windowSize);
  this->dataPtr->frontSteerVel.SetWindowSize(_windowSize);
  this->dataPtr->rearSteerVel.SetWindowSize(_windowSize);
}

//////////////////////////////////////////////////
void OdometryFourWheel::Init(const clock::time_point &_time)
{
  // Reset accumulators and timestamp.
  this->dataPtr->linearAcceleration.Clear();
  this->dataPtr->linearJerk.Clear();
  this->dataPtr->frontSteerVel.Clear();
  this->dataPtr->rearSteerVel.Clear();

  this->dataPtr->lastUpdateTime = _time;
}

//////////////////////////////////////////////////
bool OdometryFourWheel::Update(double _frontLeftSpeed, double _frontRightSpeed,
                      double _rearLeftSpeed, double _rearRightSpeed,
                      double _frontSteering, double _rearSteering,
                      const clock::time_point &_time)
{
  // Get current wheel joint positions:
  const double leftWheelCurPos = _leftPos * this->dataPtr->leftWheelRadius;
  const double rightWheelCurPos = _rightPos * this->dataPtr->rightWheelRadius

  // Estimate velocity of wheels using old and current position:
  const double leftWheelEstVel = leftWheelCurPos -
  this->dataPtr->leftWheelOldPos;
  const double rightWheelEstVel = rightWheelCurPos -
    this->dataPtr->rightWheelOldPos;

  // Update old position with current
  this->dataPtr->leftWheelOldPos = leftWheelCurPos;
  this->dataPtr->rightWheelOldPos = rightWheelCurPos;

  // Compute linear and angular diff
  const double linear = (rightWheelEstVel + leftWheelEstVel) * 0.5;
  const double angular = (rightWheelEstVel - leftWheelEstVel) /
    this->dataPtr->wheelSeparation;

    // integrate_fun_(linear, angular);
/*
  const double frontTmp = std::cos(_frontSteering) *
    (std::tan(_frontSteering) - std::tan(_rearSteering)) /
    this->dataPtr->wheelBase;

  const double frontLeftTmp = frontTmp /
    sqrt(1 - this->dataPtr->steeringTrack *
        frontTmp * std::cos(_frontSteering) +
        pow(this->dataPtr->steeringTrack * frontTmp/2, 2));

  const double frontRightTmp = frontTmp /
    sqrt(1 + this->dataPtr->steeringTrack * frontTmp * std::cos(_frontSteering)
        + pow(this->dataPtr->steeringTrack * frontTmp/2, 2));

  const double flSpeedTmp = _frontLeftSpeed *
    (1 / (1 - this->dataPtr->wheelSteeringYOffset * frontLeftTmp));

  const double frSpeedTmp = _frontRightSpeed *
    (1 / (1 - this->dataPtr->wheelSteeringYOffset * frontRightTmp));

  const double frontLinearSpeed = this->dataPtr->wheelRadius *
    copysign(1.0, flSpeedTmp + frSpeedTmp) *
    sqrt((pow(_frontLeftSpeed, 2) + pow(_frontRightSpeed, 2)) /
         (2 + pow(this->dataPtr->steeringTrack * frontTmp, 2) / 2.0));

  const double rearTmp = std::cos(_rearSteering) *
    (std::tan(_frontSteering) - std::tan(_rearSteering)) /
    this->dataPtr->wheelBase;

  const double rearLeftTmp = rearTmp /
    sqrt(1 - this->dataPtr->steeringTrack * rearTmp * std::cos(_rearSteering) +
         pow(this->dataPtr->steeringTrack * rearTmp / 2, 2));

  const double rearRightTmp = rearTmp /
    sqrt(1 + this->dataPtr->steeringTrack * rearTmp * std::cos(_rearSteering) +
         pow(this->dataPtr->steeringTrack * rearTmp / 2, 2));

  const double rlSpeedTmp = _rearLeftSpeed *
    (1 / (1 - this->dataPtr->wheelSteeringYOffset * rearLeftTmp));

  const double rrSpeedTmp = _rearRightSpeed *
    (1 / (1 - this->dataPtr->wheelSteeringYOffset * rearRightTmp));

  const double rearLinearSpeed = this->dataPtr->wheelRadius *
    std::copysign(1.0, rlSpeedTmp + rrSpeedTmp) *
    std::sqrt((std::pow(rlSpeedTmp, 2) + std::pow(rrSpeedTmp, 2)) /
              (2 + std::pow(this->dataPtr->steeringTrack * rearTmp, 2) / 2.0));

  this->dataPtr->angular = (frontLinearSpeed * frontTmp +
      rearLinearSpeed * rearTmp) / 2.0;

  this->dataPtr->linearX = (frontLinearSpeed * std::cos(_frontSteering) +
      rearLinearSpeed * std::cos(_rearSteering)) / 2.0;

  this->dataPtr->linearY = (frontLinearSpeed * std::sin(_frontSteering) -
      this->dataPtr->wheelBase * this->dataPtr->angular / 2.0 +
      rearLinearSpeed * std::sin(_rearSteering) + this->dataPtr->wheelBase *
      this->dataPtr->angular / 2.0) / 2.0;

  this->dataPtr->linear = std::copysign(1.0, rearLinearSpeed) *
    std::sqrt(std::pow(this->dataPtr->linearX, 2) +
              std::pow(this->dataPtr->linearY, 2));
              */

  /// Compute x, y and heading using velocity
  const std::chrono::duration<double> dt =
    _time - this->dataPtr->lastUpdateTime;

  // Check if interval is too small to integrate
  if (dt.count() < 0.0001)
    return false;

  this->dataPtr->lastUpdateTime = _time;

  // Estimate speeds using a rolling mean to filter them out:
  this->dataPtr->linearAcc.Push(linear / dt.count());
  this->dataPtr->angularAcc.Push(angular / dt.count());

  this->dataPtr->linear = this->dataPtr->linearAcc.Mean();
  this->dataPtr->angular = this->dataPtr->angularAcc.Mean();

  /*
  // Integrate odometry
  this->dataPtr->IntegrateXY(this->dataPtr->linearX * dt.count(),
      this->dataPtr->linearY * dt.count(), this->dataPtr->angular * dt.count());

  this->dataPtr->linearAcceleration.Push(
      (this->dataPtr->linearVelPrev - this->dataPtr->linear) / dt.count());
  this->dataPtr->linearVelPrev = this->dataPtr->linear;

  this->dataPtr->linearJerk.Push(
      (this->dataPtr->linearAccelPrev -
       this->dataPtr->linearAcceleration.Mean()) / dt.count());
  this->dataPtr->linearAccelPrev = this->dataPtr->linearAcceleration.Mean();

  this->dataPtr->frontSteerVel.Push(
      (this->dataPtr->frontSteerVelPrev - _frontSteering) / dt.count());
  this->dataPtr->frontSteerVelPrev = _frontSteering;

  this->dataPtr->rearSteerVel.Push(
      (this->dataPtr->rearSteerVelPrev - _rearSteering) / dt.count());
  this->dataPtr->rearSteerVelPrev = _rearSteering;
  */

  return true;
}

//////////////////////////////////////////////////
void OdometryFourWheel::SetWheelParams(double _steeringTrack,
    double _wheelSteeringYOffset, double _wheelRadius, double _wheelBase)
{
  this->dataPtr->steeringTrack = _steeringTrack;
  this->dataPtr->wheelSteeringYOffset = _wheelSteeringYOffset;
  this->dataPtr->wheelRadius = _wheelRadius;
  this->dataPtr->wheelBase = _wheelBase;
}

//////////////////////////////////////////////////
void OdometryFourWheel::SetVelocityRollingWindowSize(size_t _size)
{
  this->dataPtr->linearAcceleration.SetWindowSize(_size);
  this->dataPtr->linearJerk.SetWindowSize(_size);
  this->dataPtr->frontSteerVel.SetWindowSize(_size);
  this->dataPtr->rearSteerVel.SetWindowSize(_size);
}

//////////////////////////////////////////////////
double OdometryFourWheel::Heading() const
{
  return this->dataPtr->heading;
}

//////////////////////////////////////////////////
double OdometryFourWheel::X() const
{
  return this->dataPtr->x;
}

//////////////////////////////////////////////////
double OdometryFourWheel::Y() const
{
  return this->dataPtr->y;
}

//////////////////////////////////////////////////
double OdometryFourWheel::LinearVelocity() const
{
  return this->dataPtr->linear;
}

//////////////////////////////////////////////////
double OdometryFourWheel::LinearVelocityX() const
{
  return this->dataPtr->linearX;
}

//////////////////////////////////////////////////
double OdometryFourWheel::LinearVelocityY() const
{
  return this->dataPtr->linearY;
}

//////////////////////////////////////////////////
double OdometryFourWheel::AngularVelocity() const
{
  return this->dataPtr->angular;
}

//////////////////////////////////////////////////
double OdometryFourWheel::LinearAcceleration() const
{
  return this->dataPtr->linearAcceleration.Mean();
}

//////////////////////////////////////////////////
double OdometryFourWheel::LinearJerk() const
{
  return this->dataPtr->linearJerk.Mean();
}

//////////////////////////////////////////////////
double OdometryFourWheel::FrontSteerVelocity() const
{
  return this->dataPtr->frontSteerVel.Mean();
}

//////////////////////////////////////////////////
double OdometryFourWheel::RearSteerVelocity() const
{
  return this->dataPtr->rearSteerVel.Mean();
}

//////////////////////////////////////////////////
void OdometryFourWheelPrivate::IntegrateXY(
    double _linearX, double _linearY, double _angular)
{
  const double deltaX =
    _linearX * std::cos(this->heading) - _linearY * std::sin(this->heading);
  const double deltaY =
    _linearX * std::sin(this->heading) + _linearY * std::cos(this->heading);

  this->x += deltaX;
  this->y += deltaY;
  this->heading += _angular;
}

//////////////////////////////////////////////////
void OdometryFourWheelPrivate::IntegrateRungeKutta2(
    double _linear, double _angular)
{
  const double direction = this->heading + _angular * 0.5;

  // Runge-Kutta 2nd order integration:
  this->x += _linear * std::cos(direction);
  this->y += _linear * std::sin(direction);
  this->heading += _angular;
}

//////////////////////////////////////////////////
void OdometryFourWheelPrivate::IntegrateExact(double _linear, double _angular)
{
  if (std::fabs(_angular) < 1e-6)
  {
    this->IntegrateRungeKutta2(_linear, _angular);
  }
  else
  {
    // Exact integration (should solve problems when angular is zero):
    const double headingOld = this->heading;
    const double ratio = _linear/_angular;
    this->heading += _angular;
    this->x +=  ratio * (std::sin(this->heading) - std::sin(headingOld));
    this->y += -ratio * (std::cos(this->heading) - std::cos(headingOld));
  }
}

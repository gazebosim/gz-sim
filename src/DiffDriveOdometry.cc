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
#include "gz/math/DiffDriveOdometry.hh"
#include "gz/math/RollingMean.hh"

using namespace gz;
using namespace math;

// The implementation was borrowed from: https://github.com/ros-controls/ros_controllers/blob/melodic-devel/diff_drive_controller/src/odometry.cpp

class gz::math::DiffDriveOdometry::Implementation
{
  /// \brief Integrates the velocities (linear and angular) using 2nd order
  /// Runge-Kutta.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateRungeKutta2(double _linear,
                                    double _angular);

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
  public: Angle heading;

  /// \brief Current velocity in meter/second.
  public: double linearVel{0.0};

  /// \brief Current angular velocity in radians/second.
  public: Angle angularVel;

  /// \brief Left wheel radius in meters.
  public: double leftWheelRadius{0.0};

  /// \brief Right wheel radius in meters.
  public: double rightWheelRadius{0.0};

  /// \brief Wheel separation in meters.
  public: double wheelSeparation{1.0};

  /// \brief Previous left wheel position/state in radians.
  public: double leftWheelOldPos{0.0};

  /// \brief Previous right wheel position/state in radians.
  public: double rightWheelOldPos{0.0};

  /// \brief Rolling mean accumulators for the linear velocity
  public: RollingMean linearMean {0};

  /// \brief Rolling mean accumulators for the angular velocity
  public: RollingMean angularMean {0};

  /// \brief Initialized flag.
  public: bool initialized{false};
};

//////////////////////////////////////////////////
DiffDriveOdometry::DiffDriveOdometry(size_t _windowSize)
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  this->SetVelocityRollingWindowSize(_windowSize);
}

//////////////////////////////////////////////////
void DiffDriveOdometry::Init(const clock::time_point &_time)
{
  // Reset accumulators and timestamp.
  this->dataPtr->linearMean.Clear();
  this->dataPtr->angularMean.Clear();
  this->dataPtr->x = 0.0;
  this->dataPtr->y = 0.0;
  this->dataPtr->heading = 0.0;
  this->dataPtr->linearVel = 0.0;
  this->dataPtr->angularVel = 0.0;
  this->dataPtr->leftWheelOldPos = 0.0;
  this->dataPtr->rightWheelOldPos = 0.0;

  this->dataPtr->lastUpdateTime = _time;
  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
bool DiffDriveOdometry::Initialized() const
{
  return this->dataPtr->initialized;
}

//////////////////////////////////////////////////
bool DiffDriveOdometry::Update(const Angle &_leftPos, const Angle &_rightPos,
                      const clock::time_point &_time)
{
  // Compute x, y and heading using velocity
  const std::chrono::duration<double> dt =
    _time - this->dataPtr->lastUpdateTime;

  // Get current wheel joint positions:
  const double leftWheelCurPos = *_leftPos * this->dataPtr->leftWheelRadius;
  const double rightWheelCurPos = *_rightPos * this->dataPtr->rightWheelRadius;

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

  this->dataPtr->IntegrateExact(linear, angular);

  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (equal(0.0, dt.count()))
    return false;

  this->dataPtr->lastUpdateTime = _time;

  // Estimate speeds using a rolling mean to filter them out:
  this->dataPtr->linearMean.Push(linear / dt.count());
  this->dataPtr->angularMean.Push(angular / dt.count());

  this->dataPtr->linearVel = this->dataPtr->linearMean.Mean();
  this->dataPtr->angularVel = this->dataPtr->angularMean.Mean();

  return true;
}

//////////////////////////////////////////////////
void DiffDriveOdometry::SetWheelParams(double _wheelSeparation,
    double _leftWheelRadius, double _rightWheelRadius)
{
  this->dataPtr->wheelSeparation = _wheelSeparation;
  this->dataPtr->leftWheelRadius = _leftWheelRadius;
  this->dataPtr->rightWheelRadius = _rightWheelRadius;
}

//////////////////////////////////////////////////
void DiffDriveOdometry::SetVelocityRollingWindowSize(size_t _size)
{
  this->dataPtr->linearMean.SetWindowSize(_size);
  this->dataPtr->angularMean.SetWindowSize(_size);
}

//////////////////////////////////////////////////
const Angle &DiffDriveOdometry::Heading() const
{
  return this->dataPtr->heading;
}

//////////////////////////////////////////////////
double DiffDriveOdometry::X() const
{
  return this->dataPtr->x;
}

//////////////////////////////////////////////////
double DiffDriveOdometry::Y() const
{
  return this->dataPtr->y;
}

//////////////////////////////////////////////////
double DiffDriveOdometry::LinearVelocity() const
{
  return this->dataPtr->linearVel;
}

//////////////////////////////////////////////////
const Angle &DiffDriveOdometry::AngularVelocity() const
{
  return this->dataPtr->angularVel;
}

//////////////////////////////////////////////////
void DiffDriveOdometry::Implementation::IntegrateRungeKutta2(
    double _linear, double _angular)
{
  const double direction = *this->heading + _angular * 0.5;

  // Runge-Kutta 2nd order integration:
  this->x += _linear * std::cos(direction);
  this->y += _linear * std::sin(direction);
  this->heading += _angular;
}

//////////////////////////////////////////////////
void DiffDriveOdometry::Implementation::IntegrateExact(double _linear,
                                double _angular)
{
  if (std::fabs(_angular) < 1e-6)
  {
    this->IntegrateRungeKutta2(_linear, _angular);
  }
  else
  {
    // Exact integration (should solve problems when angular is zero):
    const double headingOld = *this->heading;
    const double ratio = _linear / _angular;
    this->heading += _angular;
    this->x += ratio * (std::sin(*this->heading) - std::sin(headingOld));
    this->y += -ratio * (std::cos(*this->heading) - std::cos(headingOld));
  }
}

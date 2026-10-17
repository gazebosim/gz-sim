/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "PlanarMotion.hh"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

using namespace gz;
using namespace sim::systems::simplified_physics;

namespace
{
/// \brief The integral of a body-frame twist, in long double and with no
/// series, as a reference for IntegrateArc.
PlanarDelta Reference(double _yaw, double _vx, double _vy, double _wz,
    double _dt)
{
  const long double theta = static_cast<long double>(_wz) * _dt;
  const long double sinc = std::sin(theta) / theta;
  const long double half = std::sin(0.5L * theta);
  const long double oneMinusCos = 2.0L * half * half / theta;
  const long double lx = _dt * (_vx * sinc - _vy * oneMinusCos);
  const long double ly = _dt * (_vy * sinc + _vx * oneMinusCos);
  PlanarDelta delta;
  delta.x = static_cast<double>(std::cos(static_cast<long double>(_yaw)) * lx -
      std::sin(static_cast<long double>(_yaw)) * ly);
  delta.y = static_cast<double>(std::sin(static_cast<long double>(_yaw)) * lx +
      std::cos(static_cast<long double>(_yaw)) * ly);
  delta.yaw = static_cast<double>(theta);
  return delta;
}
}

/////////////////////////////////////////////////
TEST(PlanarMotion, StraightLine)
{
  auto delta = IntegrateArc(0.0, 1.0, 0.0, 0.0, 0.5);
  EXPECT_DOUBLE_EQ(0.5, delta.x);
  EXPECT_DOUBLE_EQ(0.0, delta.y);
  EXPECT_DOUBLE_EQ(0.0, delta.yaw);
  EXPECT_FALSE(delta.IsZero());

  // Heading along y.
  delta = IntegrateArc(GZ_PI_2, 2.0, 0.0, 0.0, 0.25);
  EXPECT_NEAR(0.0, delta.x, 1e-15);
  EXPECT_NEAR(0.5, delta.y, 1e-15);
  EXPECT_DOUBLE_EQ(0.0, delta.yaw);

  // Backwards.
  delta = IntegrateArc(0.0, -1.0, 0.0, 0.0, 0.5);
  EXPECT_DOUBLE_EQ(-0.5, delta.x);

  // Sideways, to the left of the heading.
  delta = IntegrateArc(0.0, 0.0, 1.0, 0.0, 0.5);
  EXPECT_DOUBLE_EQ(0.0, delta.x);
  EXPECT_DOUBLE_EQ(0.5, delta.y);

  // Sideways with the heading along y: left is along -x.
  delta = IntegrateArc(GZ_PI_2, 0.0, 1.0, 0.0, 0.5);
  EXPECT_NEAR(-0.5, delta.x, 1e-15);
  EXPECT_NEAR(0.0, delta.y, 1e-15);
}

/////////////////////////////////////////////////
TEST(PlanarMotion, PureRotation)
{
  auto delta = IntegrateArc(0.3, 0.0, 0.0, 2.0, 0.25);
  EXPECT_DOUBLE_EQ(0.0, delta.x);
  EXPECT_DOUBLE_EQ(0.0, delta.y);
  EXPECT_DOUBLE_EQ(0.5, delta.yaw);
  EXPECT_FALSE(delta.IsZero());

  delta = IntegrateArc(0.3, 0.0, 0.0, -2.0, 0.25);
  EXPECT_DOUBLE_EQ(-0.5, delta.yaw);

  // The origin of the body stays put, even for a large turn.
  delta = IntegrateArc(0.0, 0.0, 0.0, 100.0, 1.0);
  EXPECT_DOUBLE_EQ(0.0, delta.x);
  EXPECT_DOUBLE_EQ(0.0, delta.y);
  EXPECT_DOUBLE_EQ(100.0, delta.yaw);
}

/////////////////////////////////////////////////
TEST(PlanarMotion, Arc)
{
  // A quarter turn at unit speed and unit rate: a circle of radius 1 whose
  // center is at (0, 1).
  auto delta = IntegrateArc(0.0, 1.0, 0.0, 1.0, GZ_PI_2);
  EXPECT_NEAR(1.0, delta.x, 1e-15);
  EXPECT_NEAR(1.0, delta.y, 1e-15);
  EXPECT_NEAR(GZ_PI_2, delta.yaw, 1e-15);

  // A right turn mirrors it.
  delta = IntegrateArc(0.0, 1.0, 0.0, -1.0, GZ_PI_2);
  EXPECT_NEAR(1.0, delta.x, 1e-15);
  EXPECT_NEAR(-1.0, delta.y, 1e-15);
  EXPECT_NEAR(-GZ_PI_2, delta.yaw, 1e-15);

  // A full circle returns to the start.
  delta = IntegrateArc(0.7, 1.0, 0.0, 1.0, 2.0 * GZ_PI);
  EXPECT_NEAR(0.0, delta.x, 1e-14);
  EXPECT_NEAR(0.0, delta.y, 1e-14);
  EXPECT_NEAR(2.0 * GZ_PI, delta.yaw, 1e-14);

  // The chord of a half circle of radius 2 is its diameter, perpendicular to
  // the heading.
  delta = IntegrateArc(0.0, 2.0, 0.0, 1.0, GZ_PI);
  EXPECT_NEAR(0.0, delta.x, 1e-14);
  EXPECT_NEAR(4.0, delta.y, 1e-14);

  // Lateral velocity turns with the body.
  delta = IntegrateArc(0.0, 0.0, 1.0, 1.0, GZ_PI_2);
  EXPECT_NEAR(-1.0, delta.x, 1e-15);
  EXPECT_NEAR(1.0, delta.y, 1e-15);
}

/////////////////////////////////////////////////
TEST(PlanarMotion, MatchesReference)
{
  // Rates around the switch to the series, and well inside it.
  for (const double theta : {1e-12, 1e-8, 1e-5, 9.9e-5, 1.1e-4, 1e-3, 0.1,
      1.0, 3.0})
  {
    for (const double sign : {1.0, -1.0})
    {
      const double dt = 0.01;
      const double wz = sign * theta / dt;
      const auto got = IntegrateArc(0.4, 1.5, -0.5, wz, dt);
      const auto want = Reference(0.4, 1.5, -0.5, wz, dt);
      EXPECT_NEAR(want.x, got.x, 1e-15) << theta;
      EXPECT_NEAR(want.y, got.y, 1e-15) << theta;
      EXPECT_NEAR(want.yaw, got.yaw, 1e-15 + 1e-15 * std::abs(want.yaw))
          << theta;
    }
  }
}

/////////////////////////////////////////////////
TEST(PlanarMotion, ContinuousAtZeroRate)
{
  // No jump between a tiny rate and none.
  const auto straight = IntegrateArc(0.2, 1.0, 0.3, 0.0, 0.01);
  const auto tiny = IntegrateArc(0.2, 1.0, 0.3, 1e-12, 0.01);
  EXPECT_NEAR(straight.x, tiny.x, 1e-15);
  EXPECT_NEAR(straight.y, tiny.y, 1e-15);
}

/////////////////////////////////////////////////
TEST(PlanarMotion, IndependentOfStepSize)
{
  // The same command over one second, in steps of different sizes. The
  // heading is updated between steps, as the system does.
  const double vx = 0.8;
  const double vy = 0.2;
  const double wz = 1.3;
  const double yaw0 = 0.25;

  const auto whole = IntegrateArc(yaw0, vx, vy, wz, 1.0);

  for (const int steps : {2, 10, 100, 1000})
  {
    const double dt = 1.0 / steps;
    double x = 0.0;
    double y = 0.0;
    double yaw = yaw0;
    for (int i = 0; i < steps; ++i)
    {
      const auto delta = IntegrateArc(yaw, vx, vy, wz, dt);
      x += delta.x;
      y += delta.y;
      yaw += delta.yaw;
    }
    EXPECT_NEAR(whole.x, x, 1e-12) << steps;
    EXPECT_NEAR(whole.y, y, 1e-12) << steps;
    EXPECT_NEAR(yaw0 + whole.yaw, yaw, 1e-12) << steps;
  }
}

/////////////////////////////////////////////////
TEST(PlanarMotion, InvalidInput)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_TRUE(IntegrateArc(0.0, 1.0, 1.0, 1.0, 0.0).IsZero());
  EXPECT_TRUE(IntegrateArc(0.0, 1.0, 1.0, 1.0, -0.1).IsZero());
  EXPECT_TRUE(IntegrateArc(0.0, 1.0, 1.0, 1.0, nan).IsZero());
  EXPECT_TRUE(IntegrateArc(0.0, 1.0, 1.0, 1.0, inf).IsZero());
  EXPECT_TRUE(IntegrateArc(nan, 1.0, 1.0, 1.0, 0.1).IsZero());
  EXPECT_TRUE(IntegrateArc(0.0, nan, 1.0, 1.0, 0.1).IsZero());
  EXPECT_TRUE(IntegrateArc(0.0, 1.0, inf, 1.0, 0.1).IsZero());
  EXPECT_TRUE(IntegrateArc(0.0, 1.0, 1.0, nan, 0.1).IsZero());

  // A zero twist is a zero displacement.
  EXPECT_TRUE(IntegrateArc(1.0, 0.0, 0.0, 0.0, 0.1).IsZero());

  // A very small step still moves.
  EXPECT_FALSE(IntegrateArc(0.0, 1.0, 0.0, 0.0, 1e-12).IsZero());
}

/////////////////////////////////////////////////
TEST(PlanarMotion, FrameAtOrigin)
{
  const math::Pose3d origin(1, 2, 0.5, 0, 0, GZ_PI_2);
  const math::Vector3d velocity(0.0, 1.0, 0.0);

  const auto frame = PlanarFrameAt(origin, velocity, 0.0, math::Pose3d::Zero);
  EXPECT_EQ(origin, frame.pose);
  EXPECT_EQ(velocity, frame.linearVelocity);
  EXPECT_EQ(math::Vector3d::Zero, frame.angularVelocity);
  EXPECT_EQ(math::Vector3d::Zero, frame.linearAcceleration);
  EXPECT_EQ(math::Vector3d::Zero, frame.angularAcceleration);
}

/////////////////////////////////////////////////
TEST(PlanarMotion, FrameAtOffset)
{
  // A body heading along +y that turns left at 2 rad/s. A point 1 m ahead of
  // the origin is at (1, 3). It moves with the origin's velocity plus the
  // turn: w x r = (0, 0, 2) x (0, 1, 0) = (-2, 0, 0).
  const math::Pose3d origin(1, 2, 0.5, 0, 0, GZ_PI_2);
  const math::Vector3d velocity(0.0, 1.0, 0.0);
  const double wz = 2.0;

  auto frame = PlanarFrameAt(origin, velocity, wz,
      math::Pose3d(1, 0, 0, 0, 0, 0));
  EXPECT_TRUE(frame.pose.Pos().Equal(math::Vector3d(1, 3, 0.5), 1e-12));
  EXPECT_TRUE(frame.linearVelocity.Equal(math::Vector3d(-2, 1, 0), 1e-12));
  EXPECT_EQ(math::Vector3d(0, 0, wz), frame.angularVelocity);
  EXPECT_EQ(math::Vector3d::Zero, frame.angularAcceleration);

  // a = w x v = (0, 0, 2) x (-2, 1, 0) = (-2, -4, 0).
  EXPECT_TRUE(frame.linearAcceleration.Equal(math::Vector3d(-2, -4, 0), 1e-12));

  // The offset rotates with the body: the point has the body's heading.
  EXPECT_TRUE(frame.pose.Rot().Equal(origin.Rot(), 1e-12));

  // A frame of a frame equals the composed offset.
  const auto link = PlanarFrameAt(origin, velocity, wz,
      math::Pose3d(0.5, 0, 0, 0, 0, 0));
  const auto sensor = PlanarFrameAt(link.pose, link.linearVelocity,
      link.angularVelocity.Z(), math::Pose3d(0.5, 0, 0, 0, 0, 0));
  EXPECT_TRUE(sensor.pose.Pos().Equal(frame.pose.Pos(), 1e-12));
  EXPECT_TRUE(sensor.linearVelocity.Equal(frame.linearVelocity, 1e-12));
  EXPECT_TRUE(sensor.linearAcceleration.Equal(frame.linearAcceleration, 1e-12));
}

/////////////////////////////////////////////////
TEST(PlanarMotion, TwistFromDeltaInvertsTheArc)
{
  const double yaws[] = {0.0, 0.7, -2.5, 3.1};
  const double speeds[][3] = {
      {1.0, 0.0, 0.0}, {-0.5, 0.0, 0.0}, {0.8, 0.0, 0.4}, {0.3, -0.2, -1.1},
      {0.0, 0.0, 2.0}, {1.2, 0.1, 1e-6}, {0.4, 0.0, 30.0}};
  for (const double yaw : yaws)
  {
    for (const auto &v : speeds)
    {
      for (const double dt : {0.001, 0.01, 0.1})
      {
        const auto delta = IntegrateArc(yaw, v[0], v[1], v[2], dt);
        const auto twist = TwistFromDelta(yaw, delta, dt);
        EXPECT_NEAR(v[0], twist.vx, 1e-9) << yaw << " " << dt;
        EXPECT_NEAR(v[1], twist.vy, 1e-9) << yaw << " " << dt;
        EXPECT_NEAR(v[2], twist.wz, 1e-9) << yaw << " " << dt;
      }
    }
  }
}

/////////////////////////////////////////////////
TEST(PlanarMotion, TwistFromDeltaOfNothing)
{
  const auto zero = TwistFromDelta(1.0, PlanarDelta(), 0.001);
  EXPECT_EQ(0.0, zero.vx);
  EXPECT_EQ(0.0, zero.vy);
  EXPECT_EQ(0.0, zero.wz);

  PlanarDelta delta;
  delta.x = 0.1;
  EXPECT_EQ(0.0, TwistFromDelta(0.0, delta, 0.0).vx);
  EXPECT_EQ(0.0, TwistFromDelta(0.0, delta, -1.0).vx);
  delta.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(0.0, TwistFromDelta(0.0, delta, 0.001).vx);
}

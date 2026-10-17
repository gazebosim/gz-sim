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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "WheelKinematics.hh"

using namespace gz;
using namespace sim::systems::simplified_physics;

namespace
{
constexpr double kRadius{0.3};
constexpr double kHalfTrack{0.625};

/// \brief Two wheels on an axle 0.4 m ahead of the model origin, rolling
/// along +x. Index 0 is on the left (+y), index 1 on the right.
std::vector<WheelGeometry> DiffDriveWheels()
{
  WheelGeometry left;
  left.contact.Set(0.4, kHalfTrack);
  left.radius = kRadius;
  WheelGeometry right;
  right.contact.Set(0.4, -kHalfTrack);
  right.radius = kRadius;
  return {left, right};
}

/// \brief The same, with a second pair of wheels 0.8 m behind the first.
std::vector<WheelGeometry> SkidSteerWheels()
{
  auto wheels = DiffDriveWheels();
  for (int i = 0; i < 2; ++i)
  {
    auto wheel = wheels[i];
    wheel.contact.X(-0.4);
    wheels.push_back(wheel);
  }
  return wheels;
}
}

/////////////////////////////////////////////////
TEST(WheelKinematics, DiffDriveClosedForm)
{
  WheelDrive drive(DiffDriveWheels());
  ASSERT_EQ(2u, drive.Size());
  EXPECT_DOUBLE_EQ(2 * kHalfTrack, drive.Track());

  const double rates[2] = {3.0, 5.0};
  const auto twist = drive.TwistFromRates(rates, 0x3);

  // v = r (wL + wR) / 2 and wz = r (wR - wL) / b.
  const double v = kRadius * (rates[0] + rates[1]) / 2;
  const double wz = kRadius * (rates[1] - rates[0]) / (2 * kHalfTrack);
  EXPECT_NEAR(v, twist.vx, 1e-7);
  EXPECT_NEAR(wz, twist.wz, 1e-7);

  // The wheels do not slide sideways: the axle midpoint, 0.4 m ahead of the
  // origin, has no lateral velocity, so the origin has -0.4 wz.
  EXPECT_NEAR(-0.4 * wz, twist.vy, 1e-7);
}

/////////////////////////////////////////////////
TEST(WheelKinematics, StraightLineAndSpinInPlace)
{
  WheelDrive drive(DiffDriveWheels());

  const double straight[2] = {2.0, 2.0};
  auto twist = drive.TwistFromRates(straight, 0x3);
  EXPECT_NEAR(2.0 * kRadius, twist.vx, 1e-7);
  EXPECT_NEAR(0.0, twist.vy, 1e-7);
  EXPECT_NEAR(0.0, twist.wz, 1e-7);

  const double spin[2] = {-2.0, 2.0};
  twist = drive.TwistFromRates(spin, 0x3);
  EXPECT_NEAR(0.0, twist.vx, 1e-7);
  EXPECT_NEAR(2.0 * 2.0 * kRadius / (2 * kHalfTrack), twist.wz, 1e-7);
}

/////////////////////////////////////////////////
TEST(WheelKinematics, RatesFromTwistInvertsTwistFromRates)
{
  for (const auto &wheels : {DiffDriveWheels(), SkidSteerWheels()})
  {
    WheelDrive drive(wheels);
    std::vector<double> rates(wheels.size());
    for (std::size_t i = 0; i < wheels.size(); ++i)
      rates[i] = wheels[i].contact.Y() > 0 ? 1.5 : 4.0;

    const auto twist = drive.TwistFromRates(rates.data(),
        (std::uint64_t{1} << wheels.size()) - 1);
    for (std::size_t i = 0; i < wheels.size(); ++i)
    {
      // Exact for one wheel per side, and within the sideways weight for
      // the skid-steer set.
      EXPECT_NEAR(rates[i], drive.RateFromTwist(i, twist), 1e-3)
          << "wheel " << i << " of " << wheels.size();
    }
  }
}

/////////////////////////////////////////////////
TEST(WheelKinematics, SkidSteerTurns)
{
  // Wheels at x = +-0.4 on both sides. Strict no-slip rows would pin the
  // twist to zero. With the small weight the vehicle still turns.
  WheelDrive drive(SkidSteerWheels());
  const double rates[4] = {-2.0, 2.0, -2.0, 2.0};
  const auto twist = drive.TwistFromRates(rates, 0xf);
  EXPECT_NEAR(2.0 * 2.0 * kRadius / (2 * kHalfTrack), twist.wz, 1e-3);
  EXPECT_NEAR(0.0, twist.vx, 1e-6);
  EXPECT_NEAR(0.0, twist.vy, 1e-6);
}

/////////////////////////////////////////////////
TEST(WheelKinematics, DrivenSubsetsAndCache)
{
  WheelDrive drive(SkidSteerWheels());
  // Wheel rates for the front pair (0, 1) and the rear pair (2, 3).
  const double rates[4] = {1.0, 3.0, 5.0, 5.0};

  const auto front = drive.TwistFromRates(rates, 0x3);
  const auto rear = drive.TwistFromRates(rates, 0xc);
  const auto frontAgain = drive.TwistFromRates(rates, 0x3);

  EXPECT_NEAR(kRadius * 2.0, front.vx, 1e-7);
  EXPECT_NEAR(kRadius * 5.0, rear.vx, 1e-7);
  EXPECT_NEAR(0.0, rear.wz, 1e-7);
  // A cached set gives the same answer as the first time.
  EXPECT_DOUBLE_EQ(front.vx, frontAgain.vx);
  EXPECT_DOUBLE_EQ(front.vy, frontAgain.vy);
  EXPECT_DOUBLE_EQ(front.wz, frontAgain.wz);

  // Rates of wheels outside the set are not read.
  const double junk[4] = {1.0, 3.0, std::nan(""), std::nan("")};
  const auto only = drive.TwistFromRates(junk, 0x3);
  EXPECT_DOUBLE_EQ(front.vx, only.vx);
  EXPECT_DOUBLE_EQ(front.wz, only.wz);
}

/////////////////////////////////////////////////
TEST(WheelKinematics, NothingDriven)
{
  WheelDrive drive(DiffDriveWheels());
  const double rates[2] = {1.0, 1.0};
  const auto twist = drive.TwistFromRates(rates, 0);
  EXPECT_EQ(0.0, twist.vx);
  EXPECT_EQ(0.0, twist.vy);
  EXPECT_EQ(0.0, twist.wz);

  WheelDrive empty({});
  EXPECT_EQ(0u, empty.Size());
  EXPECT_EQ(0.0, empty.Track());
  EXPECT_EQ(0.0, empty.TwistFromRates(rates, 0x3).vx);
}

/////////////////////////////////////////////////
TEST(WheelKinematics, TrackIgnoresTheDiagonal)
{
  // Four wheels at (+-0.3, +-0.2) rolling along x: the track is 0.4 m.
  std::vector<WheelGeometry> wheels;
  for (const double x : {0.3, -0.3})
  {
    for (const double y : {0.2, -0.2})
    {
      WheelGeometry wheel;
      wheel.contact.Set(x, y);
      wheel.rolling.Set(1.0, 0.0);
      wheel.radius = kRadius;
      wheels.push_back(wheel);
    }
  }
  EXPECT_NEAR(0.4, WheelDrive(wheels).Track(), 1e-12);
}

/////////////////////////////////////////////////
TEST(WheelKinematics, SingleWheelHasASolution)
{
  WheelDrive drive(DiffDriveWheels());
  const double rates[2] = {2.0, 0.0};
  const auto twist = drive.TwistFromRates(rates, 0x1);
  EXPECT_TRUE(std::isfinite(twist.vx));
  EXPECT_TRUE(std::isfinite(twist.vy));
  EXPECT_TRUE(std::isfinite(twist.wz));
  // The one driven wheel rolls at its rate whatever the motion is.
  EXPECT_NEAR(2.0, drive.RateFromTwist(0, twist), 1e-6);
}

/////////////////////////////////////////////////
TEST(WheelKinematics, ReversedAxisRollsTheOtherWay)
{
  auto wheels = DiffDriveWheels();
  wheels[0].rolling.Set(-1.0, 0.0);
  wheels[1].rolling.Set(-1.0, 0.0);
  WheelDrive drive(wheels);

  const double rates[2] = {2.0, 2.0};
  EXPECT_NEAR(-2.0 * kRadius, drive.TwistFromRates(rates, 0x3).vx, 1e-7);
}

/////////////////////////////////////////////////
TEST(WheelKinematics, MoreThanTheLimitIsCut)
{
  std::vector<WheelGeometry> many(WheelDrive::kMaxWheels + 6);
  WheelDrive drive(many);
  EXPECT_EQ(WheelDrive::kMaxWheels, drive.Size());
}

/////////////////////////////////////////////////
TEST(WheelKinematics, AdvanceJointWithoutLimits)
{
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(1.5, AdvanceJoint(1.0, 0.5, 1.0, -inf, inf));
  EXPECT_DOUBLE_EQ(0.5, AdvanceJoint(1.0, -1.0, 0.5, -1.79769e+308,
      1.79769e+308));
  EXPECT_DOUBLE_EQ(-3.0, AdvanceJoint(-2.0, -1.0, 1.0,
      std::numeric_limits<double>::quiet_NaN(), inf));
}

/////////////////////////////////////////////////
TEST(WheelKinematics, AdvanceJointStopsAtLimits)
{
  EXPECT_DOUBLE_EQ(1.0, AdvanceJoint(0.9, 5.0, 1.0, -1.0, 1.0));
  EXPECT_DOUBLE_EQ(-1.0, AdvanceJoint(-0.9, -5.0, 1.0, -1.0, 1.0));
  EXPECT_DOUBLE_EQ(0.2, AdvanceJoint(0.0, 0.1, 2.0, -1.0, 1.0));
  // A position outside the limits is pulled in.
  EXPECT_DOUBLE_EQ(1.0, AdvanceJoint(2.0, 0.0, 0.0, -1.0, 1.0));
  // Swapped limits are ignored.
  EXPECT_DOUBLE_EQ(0.5, AdvanceJoint(0.0, 0.5, 1.0, 1.0, -1.0));
}

/////////////////////////////////////////////////
TEST(WheelKinematics, AdvanceJointIgnoresBadInput)
{
  const double nan = std::nan("");
  EXPECT_DOUBLE_EQ(0.5, AdvanceJoint(0.5, nan, 1.0, -1.0, 1.0));
  EXPECT_DOUBLE_EQ(0.5, AdvanceJoint(0.5, 1.0, nan, -1.0, 1.0));
  EXPECT_DOUBLE_EQ(0.5, AdvanceJoint(0.5, 1.0, -1.0, -1.0, 1.0));
}

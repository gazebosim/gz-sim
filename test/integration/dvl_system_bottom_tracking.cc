/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */

#include <gtest/gtest.h>

#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <gz/msgs/dvl_velocity_tracking.pb.h>
#include <gz/msgs/dvl_tracking_target.pb.h>

#include "helpers/Subscription.hh"
#include "helpers/TestFixture.hh"
#include "test_config.hh"

using namespace gz;
using namespace std::literals::chrono_literals;

using DVLBeamState = msgs::DVLBeamState;
using DVLTrackingTarget = msgs::DVLTrackingTarget;
using DVLVelocityTracking = msgs::DVLVelocityTracking;

static constexpr double beamInclination{GZ_PI / 6.};
static constexpr math::Vector3d sensorPositionInSFMFrame{0., 0.6, -0.16};

// Account for slight roll and limited resolution
static constexpr double kRangeTolerance{0.2};

//////////////////////////////////////////////////
TEST(DVLTest, GZ_UTILS_TEST_DISABLED_ON_MAC(BottomTracking))
{
  const std::string worldFile = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test",
      "worlds", "flat_seabed.sdf");
  TestFixtureWithModel fixture(worldFile, "tethys");

  constexpr double seaBedDepth{20.};
  // Assume zero roll and arbitrary resolution
  const double expectedBeamRange =
      (seaBedDepth + sensorPositionInSFMFrame.Z()) /
      std::cos(beamInclination);

  transport::Node node;
  Subscription<msgs::DVLVelocityTracking> velocitySubscription;
  velocitySubscription.Subscribe(node, "/dvl/velocity", 1);

  // Step a few iterations for simulation to setup itself
  fixture.Step(2s);

  ASSERT_TRUE(velocitySubscription.WaitForMessages(1, 10s));
  {
    const msgs::DVLVelocityTracking message =
        velocitySubscription.ReadLastMessage();
    ASSERT_TRUE(message.has_target());
    const msgs::DVLTrackingTarget & target = message.target();
    EXPECT_EQ(target.type(), msgs::DVLTrackingTarget::DVL_TARGET_BOTTOM);
    EXPECT_NEAR(target.range().mean(), expectedBeamRange, kRangeTolerance);
    for (int i = 0; i < message.beams_size(); ++i)
    {
      const msgs::DVLBeamState & beam = message.beams(i);
      EXPECT_EQ(beam.id(), i + 1);
      EXPECT_TRUE(beam.locked()) << "Beam #" << beam.id() << " not locked";
      EXPECT_NEAR(beam.range().mean(), expectedBeamRange, kRangeTolerance)
          << "Beam #" << beam.id() << " range is off";
    }
    ASSERT_TRUE(message.has_velocity());
    const math::Vector3d linearVelocityEstimate =
        msgs::Convert(message.velocity().mean());
    constexpr double kVelocityTolerance{1e-2};  // account for noise
    EXPECT_NEAR(0., linearVelocityEstimate.X(), kVelocityTolerance);
    EXPECT_NEAR(0., linearVelocityEstimate.Y(), kVelocityTolerance);
    EXPECT_NEAR(0., linearVelocityEstimate.Z(), kVelocityTolerance);
  }

  // Move the AUV in a straight line
  // Manipualtor sets linear velocity in body frame
  fixture.Manipulator().SetLinearVelocity(math::Vector3d::UnitX);

  // Have the AUV describe a circle
  // todo(anyone) Having a non-zero angular velocity produces inaccurate
  // velocity estimates. Investigate whether it is a test issue or gz-sensors
  // dvl implementation issue
  // fixture.Manipulator().SetAngularVelocity(math::Vector3d::UnitZ);

  // Step simulation for some time for DVL estimates to estabilize
  fixture.Step(50s);

  ASSERT_TRUE(velocitySubscription.WaitForMessages(50, 10s));

  {
    const msgs::DVLVelocityTracking message =
        velocitySubscription.ReadLastMessage();
    ASSERT_TRUE(message.has_target());
    const msgs::DVLTrackingTarget & target = message.target();
    EXPECT_EQ(target.type(), msgs::DVLTrackingTarget::DVL_TARGET_BOTTOM);
    EXPECT_NEAR(target.range().mean(), expectedBeamRange, kRangeTolerance);
    for (int i = 0; i < message.beams_size(); ++i)
    {
      const msgs::DVLBeamState & beam = message.beams(i);
      EXPECT_EQ(beam.id(), i + 1);
      EXPECT_TRUE(beam.locked()) << "Beam #" << beam.id() << " not locked";
      EXPECT_NEAR(beam.range().mean(), expectedBeamRange, kRangeTolerance)
          << "Beam #" << beam.id() << " range is off";
    }
    ASSERT_TRUE(message.has_velocity());
    const math::Vector3d linearVelocityEstimate =
        msgs::Convert(message.velocity().mean());

    const auto &linearVelocities =
        fixture.Observer().LinearVelocities();
    const auto &angularVelocities =
        fixture.Observer().AngularVelocities();
    const auto &poses = fixture.Observer().Poses();

    // Linear velocities w.r.t. to underwater currents
    // are reported in a sensor affixed, SFM frame.

    // linear velocity output from fixture Observer is in world frame
    // convert to body frame
    math::Vector3d linearVelocityBodyFrame =
        poses.back().Rot().RotateVectorReverse(
            linearVelocities.back());

    // get linear velocity w.r.t reference frame
    // sensor rotation from body
    auto sensorRot = math::Quaterniond(math::Vector3d(0, 0, GZ_PI));
    // reference_frame
    auto referenceRot = math::Quaterniond(math::Vector3d(0, 0, -GZ_PI/2.0));
    auto bodyToRef = sensorRot * referenceRot;
    math::Vector3d expectedLinearVelocityEstimate =
      bodyToRef.RotateVectorReverse(linearVelocityBodyFrame);

    constexpr double kVelocityTolerance{1e-2};  // account for noise
    EXPECT_NEAR(expectedLinearVelocityEstimate.X(),
                linearVelocityEstimate.X(), kVelocityTolerance);
    EXPECT_NEAR(expectedLinearVelocityEstimate.Y(),
                linearVelocityEstimate.Y(), kVelocityTolerance);
    EXPECT_NEAR(expectedLinearVelocityEstimate.Z(),
                linearVelocityEstimate.Z(), kVelocityTolerance);
  }
}
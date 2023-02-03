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

static constexpr math::Vector3d sensorPositionInSFMFrame{0., 0.6, -0.16};

//////////////////////////////////////////////////
TEST(DVLTest, WaterMassTracking)
{
  const std::string worldFile = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test",
      "worlds", "underwater_currents.sdf");
  TestFixtureWithModel fixture(worldFile, "tethys");

  constexpr math::Vector3d waterCurrentVelocity{-1., 0.5, 0.};

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
    EXPECT_EQ(target.type(), msgs::DVLTrackingTarget::DVL_TARGET_WATER_MASS);
    for (int i = 0; i < message.beams_size(); ++i)
    {
      const msgs::DVLBeamState & beam = message.beams(i);
      EXPECT_EQ(beam.id(), i + 1);
      EXPECT_TRUE(beam.locked())
          << "Beam #" << beam.id() << " not locked";
    }
    ASSERT_TRUE(message.has_velocity());
    const math::Vector3d linearVelocityEstimate =
        msgs::Convert(message.velocity().mean());
    constexpr double kVelocityTolerance{1e-1};  // account for noise
    EXPECT_NEAR(linearVelocityEstimate.X(),
                -waterCurrentVelocity.X(), kVelocityTolerance);
    EXPECT_NEAR(linearVelocityEstimate.Y(),
                -waterCurrentVelocity.Y(), kVelocityTolerance);
    EXPECT_NEAR(linearVelocityEstimate.Z(),
                -waterCurrentVelocity.Z(), kVelocityTolerance);
  }

  // Have the AUV describe a circle
  fixture.Manipulator().SetLinearVelocity(math::Vector3d::UnitX);
  fixture.Manipulator().SetAngularVelocity(math::Vector3d::UnitZ);

  // Step simulation for some time for DVL estimates to estabilize
  fixture.Step(50s);

  ASSERT_TRUE(velocitySubscription.WaitForMessages(50, 10s));

  {
    const msgs::DVLVelocityTracking message =
        velocitySubscription.ReadLastMessage();
    ASSERT_TRUE(message.has_target());
    const msgs::DVLTrackingTarget & target = message.target();
    EXPECT_EQ(target.type(), msgs::DVLTrackingTarget::DVL_TARGET_WATER_MASS);
    for (int i = 0; i < message.beams_size(); ++i)
    {
      const msgs::DVLBeamState & beam = message.beams(i);
      EXPECT_EQ(beam.id(), i + 1);
      EXPECT_TRUE(beam.locked())
          << "Beam #" << beam.id() << " not locked";
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
    const math::Vector3d expectedLinearVelocityEstimate =
        poses.back().Rot().RotateVectorReverse(
            linearVelocities.back() - waterCurrentVelocity +
            angularVelocities.back().Cross(
                poses.back().Rot().RotateVector(sensorPositionInSFMFrame)));
    constexpr double kVelocityTolerance{1e-1};  // account for noise
    EXPECT_NEAR(expectedLinearVelocityEstimate.X(),
                linearVelocityEstimate.X(), kVelocityTolerance);
    EXPECT_NEAR(expectedLinearVelocityEstimate.Y(),
                linearVelocityEstimate.Y(), kVelocityTolerance);
    EXPECT_NEAR(expectedLinearVelocityEstimate.Z(),
                linearVelocityEstimate.Z(), kVelocityTolerance);
  }
}

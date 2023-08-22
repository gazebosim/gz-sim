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
TEST(DVLTest, GZ_UTILS_TEST_DISABLED_ON_MAC(NoTracking))
{
  const std::string worldFile = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test",
      "worlds", "bottomless_pit.sdf");
  TestFixtureWithModel fixture(worldFile, "tethys");

  transport::Node node;
  Subscription<msgs::DVLVelocityTracking> velocitySubscription;
  velocitySubscription.Subscribe(node, "/dvl/velocity", 1);

  fixture.Step(10s);

  ASSERT_TRUE(velocitySubscription.WaitForMessages(9, 10s));

  const msgs::DVLVelocityTracking message =
      velocitySubscription.ReadLastMessage();
  EXPECT_FALSE(message.has_target());
  EXPECT_FALSE(message.has_velocity());
  for (int i = 0; i < message.beams_size(); ++i)
  {
    EXPECT_FALSE(message.beams(i).locked())
        << "Beam #" << message.beams(i).id() << " is locked";
  }
}

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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <gz/msgs/dvl_velocity_tracking.pb.h>
#include <gz/msgs/dvl_tracking_target.pb.h>

#include <gz/sim/EventManager.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/sim/TestFixture.hh>

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

//////////////////////////////////////////////////
// The DVL system asks for the frames that it needs, so it does not make the
// Sensors system render on every iteration.
TEST(DVLTest, GZ_UTILS_TEST_DISABLED_ON_MAC(RendersOnlyWhenDue))
{
  const std::string worldFile = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test",
      "worlds", "bottomless_pit.sdf");
  sim::TestFixture fixture(worldFile);

  std::atomic<uint64_t> frames{0u};
  common::ConnectionPtr postRenderConn;
  common::ConnectionPtr onDemandConn;
  fixture.OnConfigure(
      [&](const sim::Entity &, const std::shared_ptr<const sdf::Element> &,
          sim::EntityComponentManager &, sim::EventManager &_eventMgr)
      {
        postRenderConn = _eventMgr.Connect<sim::events::PostRender>(
            [&frames] { ++frames; });
        // Count the frames without asking for any
        onDemandConn =
            _eventMgr.Connect<sim::events::OnDemandRenderConnections>(
            [](unsigned int &_count) { ++_count; });
      }).Finalize();

  transport::Node node;
  Subscription<msgs::DVLVelocityTracking> velocitySubscription;
  velocitySubscription.Subscribe(node, "/dvl/velocity", 1);

  // 10 s in steps of 20 ms, with the DVL updating at 1 Hz
  constexpr uint64_t kIterations{500u};
  ASSERT_TRUE(fixture.Server()->Run(true, kIterations, false));
  ASSERT_TRUE(velocitySubscription.WaitForMessages(9, 10s));

  EXPECT_LT(frames, kIterations / 10u)
      << "The DVL system makes the Sensors system render on every iteration";
}

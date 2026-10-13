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

#include <gz/msgs/odometry.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Regression test for https://github.com/gazebosim/gz-sim/issues/3830
/// A unit quaternion q and its negation -q represent the same rotation. When
/// the published world pose flips between these two representations,
/// naively differencing consecutive orientations produces a spurious
/// near-360-degree rotation for that tick, causing a large spike in the
/// published angular velocity even though the model is rotating at a
/// constant rate.
class OdometryQuaternionContinuityTest
  : public InternalFixture<::testing::TestWithParam<int>>
{
};

/////////////////////////////////////////////////
TEST_P(OdometryQuaternionContinuityTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(AngularVelocityHasNoSpikes))
{
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(gz::common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "odometry_publisher.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  std::vector<math::Vector3d> odomAngVels;
  std::function<void(const msgs::Odometry &)> odomCb =
    [&](const msgs::Odometry &_msg)
    {
      odomAngVels.push_back(msgs::Convert(_msg.twist().angular()));
    };
  transport::Node node;
  node.Subscribe("/model/vehicle/odometry", odomCb);

  // Command a constant yaw rate. A rotation held long enough to sweep
  // through several full turns will cross the quaternion sign-flip point
  // multiple times, which is what triggers the bug.
  test::Relay velocityRamp;
  math::Vector3d angVelCmd(0.0, 0.0, 1.0);
  velocityRamp.OnPreUpdate(
      [&](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
      {
        auto en = _ecm.EntityByComponents(
          components::Model(),
          components::Name("vehicle"));
        EXPECT_NE(kNullEntity, en);

        auto angVelCmdComp =
          _ecm.Component<components::AngularVelocityCmd>(en);
        if (!angVelCmdComp)
        {
          _ecm.CreateComponent(en,
              components::AngularVelocityCmd(angVelCmd));
        }
        else
        {
          angVelCmdComp->Data() = angVelCmd;
        }
      });
  server.AddSystem(velocityRamp.systemPtr);

  // Run long enough to complete several full rotations at 1 rad/s.
  server.Run(true, 6000, false);

  int sleep = 0;
  int maxSleep = 30;
  for (; odomAngVels.size() < 200 && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(maxSleep, sleep);
  ASSERT_FALSE(odomAngVels.empty());

  // Every reported yaw rate should stay close to the commanded 1 rad/s.
  // Before the fix, a quaternion sign flip produces an angular velocity
  // spike close to 2*pi divided by one publish period, orders of magnitude
  // above the commanded rate.
  for (const auto &angVel : odomAngVels)
  {
    EXPECT_NEAR(angVel.Z(), angVelCmd.Z(), 0.5)
        << "Detected an angular velocity spike, likely from a "
        << "non-continuous orientation quaternion.";
  }
}

INSTANTIATE_TEST_SUITE_P(ServerRepeat, OdometryQuaternionContinuityTest,
    ::testing::Range(1, 2));

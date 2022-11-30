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

#include <gtest/gtest.h>

#include <gz/msgs/double.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/JointForceCmd.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define TOL 1e-4

using namespace gz;
using namespace sim;

/// \brief Test fixture for ApplyJointForce system
class ApplyJointForceTestFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Tests that the ApplyJointForce accepts joint velocity commands
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(ApplyJointForceTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointVelocityCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/apply_joint_force.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Slow update period to increase the likelihood of published data arriving
  // before iterations are completed
  server.SetUpdatePeriod(1ms);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> jointForceCmd;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));
        auto forceComp = _ecm.Component<components::JointForceCmd>(joint);
        if (forceComp)
        {
          jointForceCmd.push_back(forceComp->Data()[0]);
        }
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_EQ(initIters, jointForceCmd.size());
  for (const auto &jointForce : jointForceCmd)
  {
    EXPECT_NEAR(0, jointForce, TOL);
  }

  jointForceCmd.clear();

  // Publish command and check that the joint force is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_force_test/joint/j1/cmd_force");

  const double testJointForce{0.001};
  msgs::Double msg;
  msg.set_data(testJointForce);

  pub.Publish(msg);
  // Wait for the message to be published
  const std::size_t maxIters = 1000;
  for (std::size_t i = 0; i < maxIters; ++i)
  {
    server.Run(true, 1, false);
    if (std::abs(jointForceCmd.back() - testJointForce) < 1e-6)
    {
      break;
    }
  }
  EXPECT_DOUBLE_EQ(jointForceCmd.back(), testJointForce);
}

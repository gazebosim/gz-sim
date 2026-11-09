/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include <optional>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

class SpacecraftTest : public InternalFixture<::testing::Test>
{
  protected: std::unique_ptr<Server> StartServer(const std::string &_filePath)
  {
    ServerConfig serverConfig;
    const auto sdfFile = std::string(PROJECT_SOURCE_PATH) + _filePath;
    serverConfig.SetSdfFile(sdfFile);

    auto server = std::make_unique<Server>(serverConfig);
    EXPECT_FALSE(server->Running());
    EXPECT_FALSE(*server->Running(0));

    using namespace std::chrono_literals;
    server->SetUpdatePeriod(1ns);
    return server;
  }
};

/////////////////////////////////////////////////
// Test that a thruster duty cycle command is applied
TEST_F(SpacecraftTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(InputTest))
{
  // Start server
  auto server = this->StartServer("/examples/worlds/spacecraft.sdf");

  transport::Node node;
  auto cmdDutyCyclePublisher =
      node.Advertise<msgs::Actuators>("/dart/command/duty_cycle");

  const std::size_t iterTestStart{100};
  const std::size_t nIters{500};

  server->Run(true, iterTestStart, false);

  // Command a thruster duty cycle
  const double cmdDutyCycle{0};
  msgs::Actuators msg;
#if GOOGLE_PROTOBUF_VERSION >= 7035000
  msg.mutable_normalized()->resize(12, cmdDutyCycle);
#else
  msg.mutable_normalized()->Resize(12, cmdDutyCycle);
#endif
  msg.mutable_normalized()->Set(0, 1.0);
  cmdDutyCyclePublisher.Publish(msg);

  server->Run(true, nIters, false);

  // Check for movement in the right direction
  auto ecm = server->EcmScope();
  ASSERT_TRUE(ecm.Valid());
  const Entity id = ecm->EntityByComponents(
      components::Model(), components::Name("dart"));
  ASSERT_NE(kNullEntity, id);
  const auto *poseComp = ecm->Component<components::Pose>(id);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_GT(poseComp->Data().Pos().Y(), 0.0);
}

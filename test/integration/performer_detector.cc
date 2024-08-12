/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "test_config.hh"

#include "helpers/Relay.hh"
#include "helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

class PerformerDetectorTest : public InternalFixture<::testing::Test>
{
  protected: std::unique_ptr<Server> StartServer(const std::string &_filePath,
                                      bool _useLevels = false)
  {
    ServerConfig serverConfig;
    const auto sdfFile = std::string(PROJECT_SOURCE_PATH) + _filePath;
    serverConfig.SetSdfFile(sdfFile);
    serverConfig.SetUseLevels(_useLevels);

    auto server = std::make_unique<Server>(serverConfig);
    EXPECT_FALSE(server->Running());
    EXPECT_FALSE(*server->Running(0));

    using namespace std::chrono_literals;
    server->SetUpdatePeriod(1ns);
    return server;
  }

  protected: std::mutex poseMsgsMutex;
  protected: std::vector<msgs::Pose> poseMsgs;
};

/////////////////////////////////////////////////
// Test that commanded motor speed is applied
// See: https://github.com/gazebosim/gz-sim/issues/1175
// See: https://github.com/gazebosim/gz-sim/issues/630
TEST_F(PerformerDetectorTest,
       GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(MovingPerformer))
{
  auto server = this->StartServer("/test/worlds/performer_detector.sdf");

  transport::Node node;
  auto cmdVelPub = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");

  std::string expectedCount = "1";
  auto detectorCb = std::function<void(const msgs::Pose &)>(
      [this, &expectedCount](const auto &_msg)
      {
        std::lock_guard<std::mutex> lock(this->poseMsgsMutex);
        this->poseMsgs.push_back(_msg);

        std::string detectorName;
        for (int i = 0; i < _msg.header().data_size(); ++i)
        {
          if (_msg.header().data(i).key() == "frame_id")
            detectorName = _msg.header().data(i).value(0);
        }

        bool hasUniqueKey = false;
        bool hasDuplicateKey = false;
        bool hasCount = false;
        for (int i = 0; i < _msg.header().data_size(); ++i)
        {
          EXPECT_NE(_msg.header().data(i).key(), "no_value");
          EXPECT_NE(_msg.header().data(i).value(0), "no_key");
          EXPECT_NE(_msg.header().data(i).value(0), "first_value");
          if (_msg.header().data(i).key() == "unique_key")
          {
            EXPECT_EQ(_msg.header().data(i).value(0), "unique_value");
            hasUniqueKey  = true;
          }
          else if (_msg.header().data(i).key() == "duplicate_key")
          {
            EXPECT_EQ(_msg.header().data(i).value(0), "second_value");
            hasDuplicateKey  = true;
          }
          else if (_msg.header().data(i).key() == "count")
          {
            EXPECT_EQ(_msg.header().data(i).value(0), expectedCount);
            hasCount = true;
          }
        }
        if (detectorName == "detector1")
        {
          EXPECT_EQ(5, _msg.header().data_size());
          EXPECT_TRUE(hasDuplicateKey);
          EXPECT_TRUE(hasUniqueKey);
          EXPECT_TRUE(hasCount);
        }
        else
        {
          EXPECT_EQ(3, _msg.header().data_size());
          EXPECT_FALSE(hasDuplicateKey);
          EXPECT_FALSE(hasUniqueKey);
          EXPECT_TRUE(hasCount);
          // Change the expected count after 'detector2' is triggered.
          expectedCount = "0";
        }
      });

  node.Subscribe("/performer_detector", detectorCb);

  server->Run(true, 1, false);
  msgs::Twist cmdVelMsg;
  cmdVelMsg.mutable_linear()->set_x(2.0);
  cmdVelPub.Publish(cmdVelMsg);

  const std::size_t nIters{6000};
  server->Run(true, nIters, false);

  // Wait for messages to arrive in poseMsgs or a timeout is reached
  const auto timeOut = 5s;
  auto tInit = std::chrono::steady_clock::now();
  auto tNow = tInit;
  while (tNow - tInit < timeOut)
  {
    std::this_thread::sleep_for(100ms);

    std::lock_guard<std::mutex> lock(this->poseMsgsMutex);
    if (this->poseMsgs.size() == 4)
      break;

    tNow = std::chrono::steady_clock::now();
  }

  ASSERT_EQ(4u, this->poseMsgs.size());
  EXPECT_EQ("detector1", this->poseMsgs[0].header().data(0).value(0));
  EXPECT_EQ("1", this->poseMsgs[0].header().data(1).value(0));
  EXPECT_EQ("detector2", this->poseMsgs[1].header().data(0).value(0));
  EXPECT_EQ("1", this->poseMsgs[1].header().data(1).value(0));
  EXPECT_EQ("detector1", this->poseMsgs[2].header().data(0).value(0));
  EXPECT_EQ("0", this->poseMsgs[2].header().data(1).value(0));
  EXPECT_EQ("detector2", this->poseMsgs[3].header().data(0).value(0));
  EXPECT_EQ("0", this->poseMsgs[3].header().data(1).value(0));

  // The performer's bounding box is 2x2. It starts at a position of {0, 2} and
  // moves straight in the +x direction. The performer enters the detector's
  // region when the its bounding box interesects with the detector's region.
  // The reported position is relative to the detector.

  // detector1's XY position is {4, 0} with a region of 4x4. Accounting for the
  // performer's own bounding box, the interval of interesection becomes:
  //   x:[4 - 2 - 1, 4 + 2 + 1], y: [0 - 2 - 1, 0 + 2 + 1]
  // = x:[1, 7], y:[-3, 3]
  // The position of the performer is {1, 2} when it enters detector1's region
  // and {7, 2} when it leaves the region
  // The reported position is relative to the detector.
  EXPECT_NEAR(-3.0, this->poseMsgs[0].position().x(), 1e-2);
  EXPECT_NEAR(2.0, this->poseMsgs[0].position().y(), 1e-2);
  EXPECT_NEAR(3.0, this->poseMsgs[2].position().x(), 1e-2);
  EXPECT_NEAR(2.0, this->poseMsgs[2].position().y(), 1e-2);

  // detector2's XY position is {5, 3} with a region of 3x2.5. Accounting for
  // the performer's own bounding box, the interval of interesection becomes:
  //   x:[5 - 1.5 - 1, 5 + 1.5 + 1], y: [3 - 1.25 - 1, 3 + 1.25 + 1]
  // = x:[2.5, 7.5], y:[1.75, 5.25]
  // The position of the performer is {2.5, 2} when it enters detector2's region
  // and {7.5, 2} when it leaves the region
  // The reported position is relative to the detector.
  EXPECT_NEAR(-2.5, this->poseMsgs[1].position().x(), 1e-2);
  EXPECT_NEAR(-1, this->poseMsgs[1].position().y(), 1e-2);
  EXPECT_NEAR(2.5, this->poseMsgs[3].position().x(), 1e-2);
  EXPECT_NEAR(-1, this->poseMsgs[3].position().y(), 1e-2);
}

/////////////////////////////////////////////////
// Test that Performer detector handles the case where the associated model is
// removed, for example, by the level manager
TEST_F(PerformerDetectorTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(HandlesRemovedParentModel))
{
  auto server = this->StartServer("/test/worlds/performer_detector.sdf", true);

  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
  {
    Entity vehicle = _ecm.EntityByComponents(
        components::Model(), components::Name("vehicle_blue"));
    ASSERT_FALSE(kNullEntity == vehicle);

    if (_info.iterations == 2)
    {
      // Move vehicle out of level1
      _ecm.CreateComponent(vehicle,
          components::WorldPoseCmd(math::Pose3d({-100, 0, 0}, {})));
    }
    else if (_info.iterations == 4)
    {
      auto pose = _ecm.Component<components::Pose>(vehicle);
      EXPECT_NEAR(-100.0, pose->Data().Pos().X(), 1e-3);
      ASSERT_TRUE(nullptr == _ecm.Component<components::WorldPoseCmd>(vehicle));

      // Move vehicle back into level1 and in the detectors' region
      _ecm.CreateComponent(vehicle,
          components::WorldPoseCmd(math::Pose3d({5, 2, 0.325}, {})));
    }
    else if (_info.iterations == 5)
    {
      auto pose = _ecm.Component<components::Pose>(vehicle);
      EXPECT_NEAR(5, pose->Data().Pos().X(), 1e-3);
    }
  });

  server->AddSystem(testSystem.systemPtr);

  transport::Node node;
  auto cmdVelPub = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");

  auto detectorCb = std::function<void(const msgs::Pose &)>(
      [this](const auto &_msg)
      {
        std::lock_guard<std::mutex> lock(this->poseMsgsMutex);
        this->poseMsgs.push_back(_msg);
      });

  node.Subscribe("/performer_detector", detectorCb);

  server->Run(true, 10, false);

  // Wait for messages to arrive in poseMsgs or a timeout is reached
  const auto timeOut = 5s;
  auto tInit = std::chrono::steady_clock::now();
  auto tNow = tInit;
  while (tNow - tInit < timeOut)
  {
    std::this_thread::sleep_for(100ms);

    std::lock_guard<std::mutex> lock(this->poseMsgsMutex);
    if (this->poseMsgs.size() >= 1)
      break;

    tNow = std::chrono::steady_clock::now();
  }
  EXPECT_EQ(2u, this->poseMsgs.size());
}

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
#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

#define tol 10e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test PosePublisher system
class PosePublisherTest : public ::testing::TestWithParam<int>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

class Relay
{
  public: Relay()
  {
    auto plugin = loader.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem = static_cast<MockSystem *>(
        systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: Relay &OnPreUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnPostUpdate(MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: SystemPluginPtr systemPtr;

  private: SystemLoader loader;
  private: MockSystem *mockSystem;
};

std::mutex mutex;
std::vector<msgs::Pose> poseMsgs;

/////////////////////////////////////////////////
void poseCb(const msgs::Pose &_msg)
{
  mutex.lock();
  poseMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
TEST_F(PosePublisherTest, PublishCmd)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/pose_publisher.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that records the model's link poses
  Relay testSystem;

  // entity names
  std::string modelName = "double_pendulum_with_base";
  std::string baseName = "base";
  std::string lowerLinkName = "lower_link";
  std::string upperLinkName = "upper_link";

  // Test system for recording entity poses
  std::list<math::Pose3d> poses;
  std::list<math::Pose3d> basePoses;
  std::list<math::Pose3d> lowerLinkPoses;
  std::list<math::Pose3d> upperLinkPoses;
  std::list<common::Time> timestamps;
  testSystem.OnPostUpdate(
      [&modelName, &baseName, &lowerLinkName, &upperLinkName,
      &poses, &basePoses, &lowerLinkPoses, &upperLinkPoses, &timestamps](
      const gazebo::UpdateInfo &_info,
      const gazebo::EntityComponentManager &_ecm)
    {
      // get our double pendulum model
      auto id = _ecm.EntityByComponents(
        components::Model(),
        components::Name(modelName));
      EXPECT_NE(kNullEntity, id);

      auto poseComp = _ecm.Component<components::Pose>(id);
      ASSERT_NE(nullptr, poseComp);
      poses.push_back(poseComp->Data());

      Model model(id);

      // get base link pose
      auto base = model.LinkByName(_ecm, baseName);
      EXPECT_NE(kNullEntity, base);
      auto basePoseComp = _ecm.Component<components::Pose>(base);
      ASSERT_NE(nullptr, basePoseComp);
      basePoses.push_back(basePoseComp->Data());

      // get lower link pose
      auto lowerLink = model.LinkByName(_ecm, lowerLinkName);
      EXPECT_NE(kNullEntity, lowerLink);
      auto lowerLinkPoseComp = _ecm.Component<components::Pose>(lowerLink);
      ASSERT_NE(nullptr, lowerLinkPoseComp);
      lowerLinkPoses.push_back(lowerLinkPoseComp->Data());

      // get upper link pose
      auto upperLink = model.LinkByName(_ecm, upperLinkName);
      EXPECT_NE(kNullEntity, upperLink);
      auto upperLinkPoseComp = _ecm.Component<components::Pose>(upperLink);
      ASSERT_NE(nullptr, upperLinkPoseComp);
      upperLinkPoses.push_back(upperLinkPoseComp->Data());

      // timestamps
      auto simTimeSecNsec =
          ignition::math::durationToSecNsec(_info.simTime);
       timestamps.push_back(
           common::Time(simTimeSecNsec.first, simTimeSecNsec.second));
    });
  server.AddSystem(testSystem.systemPtr);

  // subscribe to the pose publisher
  transport::Node node;
  node.Subscribe(std::string("/model/double_pendulum_with_base/pose"),
      &poseCb);

  // Run server
  unsigned int iters = 1000u;
  server.Run(true, iters, false);

  // check that entity poses are generated and recorded
  EXPECT_EQ(iters, poses.size());
  EXPECT_EQ(iters, basePoses.size());
  EXPECT_EQ(iters, lowerLinkPoses.size());
  EXPECT_EQ(iters, upperLinkPoses.size());
  EXPECT_EQ(iters, timestamps.size());

  // Wait for all message to be received
  bool received = false;
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mutex.lock();
    received = (poseMsgs.size() ==
        (basePoses.size() + lowerLinkPoses.size() + upperLinkPoses.size()));
    mutex.unlock();

    if (received)
      break;
  }
  EXPECT_TRUE(received);

  int tCounter = 0;
  int numLinks = 3;
  mutex.lock();

  // sort the pose msgs according to timestamp
  std::sort(poseMsgs.begin(), poseMsgs.end(), [](
      const ignition::msgs::Pose &_l, const ignition::msgs::Pose &_r)
  {
    common::Time lt(_l.header().stamp().sec(), _l.header().stamp().nsec());
    common::Time rt(_r.header().stamp().sec(), _r.header().stamp().nsec());
    return lt < rt;
  });

  // verify pose msgs against recorded ones
  for (const auto &msg : poseMsgs)
  {
    EXPECT_TRUE(!msg.name().empty());

    // verify header
    EXPECT_EQ(2, msg.header().data_size());
    EXPECT_EQ("frame_id", msg.header().data(0).key());
    EXPECT_EQ(1, msg.header().data(0).value_size());
    EXPECT_EQ("child_frame_id", msg.header().data(1).key());
    EXPECT_EQ(1, msg.header().data(1).value_size());

    std::string frame = msg.header().data(0).value(0);
    EXPECT_EQ(modelName, frame);

    std::string childFrame = msg.header().data(1).value(0);
    EXPECT_EQ(childFrame, msg.name());

    // verify timestamp
    common::Time time(msg.header().stamp().sec(), msg.header().stamp().nsec());
    EXPECT_EQ(timestamps.front(), time);
    // assume msgs arrive in order and there is a pose msg for every link
    // so we only remove a timestamp from list after checking against all links
    // for that iteration
    if (++tCounter % numLinks == 0)
      timestamps.pop_front();

    // verify pose
    math::Pose3d expectedPose;
    auto p = msgs::Convert(msg);
    if (msg.name() == baseName)
    {
      expectedPose = basePoses.front();
      basePoses.pop_front();
    }
    else if (msg.name() == lowerLinkName)
    {
      expectedPose = lowerLinkPoses.front();
      lowerLinkPoses.pop_front();
    }
    else if (msg.name() == upperLinkName)
    {
      expectedPose = upperLinkPoses.front();
      upperLinkPoses.pop_front();
    }
    else
    {
      FAIL() << "Unknown link found: " << msg.name();
    }
    EXPECT_EQ(expectedPose, p);
  }
  mutex.unlock();
}

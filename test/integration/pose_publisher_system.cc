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

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define tol 10e-4

using namespace gz;
using namespace sim;

/// \brief Test PosePublisher system
class PosePublisherTest : public InternalFixture<::testing::TestWithParam<int>>
{
};

std::mutex mutex;
std::vector<msgs::Pose> poseMsgs;
std::vector<msgs::Pose> staticPoseMsgs;
std::vector<msgs::Pose_V> poseVMsgs;
std::vector<msgs::Pose_V> staticPoseVMsgs;

/////////////////////////////////////////////////
void poseCb(const msgs::Pose &_msg)
{
  mutex.lock();
  poseMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
void staticPoseCb(const msgs::Pose &_msg)
{
  mutex.lock();
  staticPoseMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
void poseVCb(const msgs::Pose_V &_msg)
{
  mutex.lock();
  poseVMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
void staticPoseVCb(const msgs::Pose_V &_msg)
{
  mutex.lock();
  staticPoseVMsgs.push_back(_msg);
  mutex.unlock();
}

std::string addDelimiter(const std::vector<std::string> &_name,
                         const std::string &_delim)
{
  if (_name.empty())
    return "";

  std::string out = _name.front();
  for (auto it = _name.begin() + 1; it != _name.end(); ++it)
  {
    out += _delim + *it;
  }
  return out;
}

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(PosePublisherTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(PublishCmd))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/pose_publisher.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that records the model's link poses
  test::Relay testSystem;

  // The delimiter is hard coded in PosePublisher. If it changes there, it needs
  // to be changed here as well
  const std::string delim = "::";
  // entity names
  std::string modelName  = "double_pendulum_with_base";
  std::string scopedModelName  = modelName;
  std::string baseName = "base";
  std::string scopedBaseName = addDelimiter({scopedModelName, baseName}, delim);
  std::string lowerLinkName = "lower_link";
  std::string scopedLowerLinkName =
      addDelimiter({scopedModelName, lowerLinkName}, delim);
  std::string upperLinkName = "upper_link";
  std::string scopedUpperLinkName =
      addDelimiter({scopedModelName, upperLinkName}, delim);
  std::string sensorName = "imu_sensor";
  std::string scopedSensorName =
      addDelimiter({scopedUpperLinkName, sensorName}, delim);

  // Test system for recording entity poses
  std::list<math::Pose3d> poses;
  std::list<math::Pose3d> basePoses;
  std::list<math::Pose3d> lowerLinkPoses;
  std::list<math::Pose3d> upperLinkPoses;
  std::list<math::Pose3d> sensorPoses;
  std::list<std::chrono::steady_clock::time_point> timestamps;
  testSystem.OnPostUpdate(
      [&modelName, &baseName, &lowerLinkName, &upperLinkName, &sensorName,
      &poses, &basePoses, &lowerLinkPoses, &upperLinkPoses, &sensorPoses,
      &timestamps](
      const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
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

      // get sensor pose
      auto sensors = _ecm.ChildrenByComponents(upperLink, components::Sensor(),
                                              components::Name(sensorName));
      ASSERT_FALSE(sensors.empty());
      auto sensor = sensors.front();
      EXPECT_NE(kNullEntity, sensor);
      auto sensorPoseComp = _ecm.Component<components::Pose>(sensor);
      ASSERT_NE(nullptr, sensorPoseComp);
      sensorPoses.push_back(sensorPoseComp->Data());

      // timestamps
      auto simTimeSecNsec =
          math::durationToSecNsec(_info.simTime);
       timestamps.push_back(
           math::secNsecToTimePoint(
             simTimeSecNsec.first,
             simTimeSecNsec.second));
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
  EXPECT_EQ(iters, sensorPoses.size());
  EXPECT_EQ(iters, timestamps.size());

  // Wait for all message to be received
  bool received = false;
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mutex.lock();
    received =
        (poseMsgs.size() == (basePoses.size() + lowerLinkPoses.size() +
                             upperLinkPoses.size() + sensorPoses.size()));
    mutex.unlock();

    if (received)
      break;
  }
  EXPECT_TRUE(received);

  int tCounter = 0;
  int numLinks = 3;
  int numSensors = 1;
  mutex.lock();

  // sort the pose msgs according to timestamp
  std::sort(poseMsgs.begin(), poseMsgs.end(), [](
      const msgs::Pose &_l, const msgs::Pose &_r)
  {
    std::chrono::steady_clock::time_point lt =
      math::secNsecToTimePoint(
        _l.header().stamp().sec(),
        _l.header().stamp().nsec());
    std::chrono::steady_clock::time_point rt =
      math::secNsecToTimePoint(
        _r.header().stamp().sec(),
        _r.header().stamp().nsec());
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
    if (msg.name() == scopedSensorName)
    {
      // Handle the sensor as a special case
      EXPECT_EQ(scopedUpperLinkName, frame);
    }
    else
      EXPECT_EQ(scopedModelName, frame);

    std::string childFrame = msg.header().data(1).value(0);
    EXPECT_EQ(childFrame, msg.name());

    // verify timestamp
    std::chrono::steady_clock::time_point time =
      math::secNsecToTimePoint(
        msg.header().stamp().sec(),
        msg.header().stamp().nsec());
    EXPECT_EQ(timestamps.front(), time);
    // assume msgs arrive in order and there is a pose msg for every link
    // so we only remove a timestamp from list after checking against all links
    // for that iteration
    if (++tCounter % (numLinks + numSensors) == 0)
      timestamps.pop_front();

    // verify pose
    math::Pose3d expectedPose;
    auto p = msgs::Convert(msg);
    if (msg.name() == scopedBaseName)
    {
      expectedPose = basePoses.front();
      basePoses.pop_front();
    }
    else if (msg.name() == scopedLowerLinkName)
    {
      expectedPose = lowerLinkPoses.front();
      lowerLinkPoses.pop_front();
    }
    else if (msg.name() == scopedUpperLinkName)
    {
      expectedPose = upperLinkPoses.front();
      upperLinkPoses.pop_front();
    }
    else if (msg.name() == scopedSensorName)
    {
      expectedPose = sensorPoses.front();
      sensorPoses.pop_front();
    }
    else
    {
      FAIL() << "Unknown link found: " << msg.name();
    }
    EXPECT_EQ(expectedPose, p);
  }
  mutex.unlock();
}

/////////////////////////////////////////////////
// See: https://github.com/gazebosim/gz-sim/issues/630
TEST_F(PosePublisherTest,
       GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(UpdateFrequency))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/pose_publisher.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that records the model's link poses
  test::Relay testSystem;

  {
    std::lock_guard<std::mutex> lock(mutex);
    poseMsgs.clear();
  }

  // subscribe to the pose publisher
  transport::Node node;
  node.Subscribe(std::string("/model/test_update_freq/pose"), &poseCb);

  // Run server
  unsigned int iters = 1000u;
  server.Run(true, iters, false);

  std::size_t nExpMessages = 100;
  // Wait for 100 messages to be received
  bool received = false;
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    {
      std::lock_guard<std::mutex> lock(mutex);
      received = (poseMsgs.size() >= nExpMessages);
    }

    if (received)
      break;
  }

  ASSERT_TRUE(received);

  {
    std::lock_guard<std::mutex> lock(mutex);
    // Calculate average frequency. The calculation is not very precise because
    // we only collect a small sample size but it should be enough to tell the
    // difference between 100 Hz and 1000 Hz
    std::chrono::duration<double> firstStamp =
        std::chrono::seconds(poseMsgs.front().header().stamp().sec()) +
        std::chrono::nanoseconds(poseMsgs.front().header().stamp().nsec());
    std::chrono::duration<double> lastStamp =
        std::chrono::seconds(poseMsgs.back().header().stamp().sec()) +
        std::chrono::nanoseconds(poseMsgs.back().header().stamp().nsec());

    double diff = (lastStamp - firstStamp).count();
    ASSERT_GT(diff, 0);
    double freq = poseMsgs.size()/diff;
    EXPECT_NEAR(100.0, freq, 10.0);
  }
}

/////////////////////////////////////////////////
TEST_F(PosePublisherTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(StaticPosePublisher))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/pose_publisher.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that records the model's link poses
  test::Relay testSystem;

  {
    std::lock_guard<std::mutex> lock(mutex);
    poseVMsgs.clear();
    staticPoseVMsgs.clear();
  }

  // The delimiter is hard coded in PosePublisher. If it changes there, it needs
  // to be changed here as well
  const std::string delim = "::";
  // entity names
  std::string modelName  = "double_pendulum_static_pose_publisher";
  std::string scopedModelName  = modelName;
  std::string baseName = "base";
  std::string scopedBaseName = addDelimiter({scopedModelName, baseName}, delim);
  std::string lowerLinkName = "lower_link";
  std::string scopedLowerLinkName =
      addDelimiter({scopedModelName, lowerLinkName}, delim);
  std::string upperLinkName = "upper_link";
  std::string scopedUpperLinkName =
      addDelimiter({scopedModelName, upperLinkName}, delim);
  std::string sensorName = "imu_sensor";
  std::string scopedSensorName =
      addDelimiter({scopedUpperLinkName, sensorName}, delim);

  // Test system for recording entity poses
  std::list<math::Pose3d> poses;
  std::list<math::Pose3d> basePoses;
  std::list<math::Pose3d> lowerLinkPoses;
  std::list<math::Pose3d> upperLinkPoses;
  std::list<math::Pose3d> sensorPoses;
  std::list<std::chrono::steady_clock::time_point> timestamps;
  std::list<std::chrono::steady_clock::time_point> staticPoseTimestamps;

  testSystem.OnPostUpdate(
      [&modelName, &baseName, &lowerLinkName, &upperLinkName, &sensorName,
      &poses, &basePoses, &lowerLinkPoses, &upperLinkPoses, &sensorPoses,
      &timestamps, &staticPoseTimestamps](
      const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
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

      // get sensor pose
      auto sensors = _ecm.ChildrenByComponents(upperLink, components::Sensor(),
                                              components::Name(sensorName));
      ASSERT_FALSE(sensors.empty());
      auto sensor = sensors.front();
      EXPECT_NE(kNullEntity, sensor);
      auto sensorPoseComp = _ecm.Component<components::Pose>(sensor);
      ASSERT_NE(nullptr, sensorPoseComp);
      sensorPoses.push_back(sensorPoseComp->Data());

      // timestamps
      auto simTimeSecNsec =
          math::durationToSecNsec(_info.simTime);
      timestamps.push_back(
          math::secNsecToTimePoint(
            simTimeSecNsec.first, simTimeSecNsec.second));
      staticPoseTimestamps.push_back(
          math::secNsecToTimePoint(
            simTimeSecNsec.first, simTimeSecNsec.second));
    });
  server.AddSystem(testSystem.systemPtr);

  // subscribe to the pose publisher
  transport::Node node;
  node.Subscribe(
      std::string("/model/double_pendulum_static_pose_publisher/pose"),
      &poseVCb);
  node.Subscribe(
      std::string("/model/double_pendulum_static_pose_publisher/pose_static"),
      &staticPoseVCb);

  // Run server
  unsigned int iters = 100u;
  server.Run(true, iters, false);

  // check that entity poses are generated and recorded
  EXPECT_EQ(iters, poses.size());
  EXPECT_EQ(iters, basePoses.size());
  EXPECT_EQ(iters, lowerLinkPoses.size());
  EXPECT_EQ(iters, upperLinkPoses.size());
  EXPECT_EQ(iters, sensorPoses.size());
  EXPECT_EQ(iters, timestamps.size());

  // Wait for all message to be received
  bool received = false;
  bool staticReceived = false;
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mutex.lock();
    received = (poseVMsgs.size() == iters);
    staticReceived = (staticPoseVMsgs.size() == iters);
    mutex.unlock();

    if (received && staticReceived)
      break;
  }
  EXPECT_TRUE(received);
  EXPECT_TRUE(staticReceived);

  mutex.lock();

  // verify pose msgs against recorded ones
  for (const auto &poseVMsg : poseVMsgs)
  {
    for (const auto &msg : poseVMsg.pose())
    {
      EXPECT_TRUE(!msg.name().empty());

      // verify header
      EXPECT_EQ(2, msg.header().data_size());
      EXPECT_EQ("frame_id", msg.header().data(0).key());
      EXPECT_EQ(1, msg.header().data(0).value_size());
      EXPECT_EQ("child_frame_id", msg.header().data(1).key());
      EXPECT_EQ(1, msg.header().data(1).value_size());

      std::string frame = msg.header().data(0).value(0);
      EXPECT_EQ(scopedModelName, frame);

      std::string childFrame = msg.header().data(1).value(0);
      EXPECT_EQ(childFrame, msg.name());

      // verify timestamp
      std::chrono::steady_clock::time_point time =
        math::secNsecToTimePoint(msg.header().stamp().sec(),
        msg.header().stamp().nsec());
      EXPECT_EQ(timestamps.front(), time);

      // verify pose
      math::Pose3d expectedPose;
      auto p = msgs::Convert(msg);
      if (msg.name() == scopedLowerLinkName)
      {
        expectedPose = lowerLinkPoses.front();
        lowerLinkPoses.pop_front();
      }
      else if (msg.name() == scopedUpperLinkName)
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
    timestamps.pop_front();
  }
  mutex.unlock();

  mutex.lock();
  // verify static pose msgs against recorded ones
  for (const auto &staticPoseVMsg : staticPoseVMsgs)
  {
    for (const auto &msg : staticPoseVMsg.pose())
    {
      EXPECT_TRUE(!msg.name().empty());

      // verify header
      EXPECT_EQ(2, msg.header().data_size());
      EXPECT_EQ("frame_id", msg.header().data(0).key());
      EXPECT_EQ(1, msg.header().data(0).value_size());
      EXPECT_EQ("child_frame_id", msg.header().data(1).key());
      EXPECT_EQ(1, msg.header().data(1).value_size());
      std::string frame = msg.header().data(0).value(0);
      if (msg.name() == scopedSensorName)
      {
        // Handle the sensor as a special case
        EXPECT_EQ(scopedUpperLinkName, frame);
      }
      else
        EXPECT_EQ(scopedModelName, frame);

      std::string childFrame = msg.header().data(1).value(0);
      EXPECT_EQ(childFrame, msg.name());

      // verify timestamp
      std::chrono::steady_clock::time_point time =
        math::secNsecToTimePoint(msg.header().stamp().sec(),
          msg.header().stamp().nsec());
      EXPECT_EQ(staticPoseTimestamps.front(), time);

      // verify pose
      math::Pose3d expectedPose;
      auto p = msgs::Convert(msg);
      if (msg.name() == scopedBaseName)
      {
        expectedPose = basePoses.front();
        basePoses.pop_front();
      }
      else if (msg.name() == scopedSensorName)
      {
        expectedPose = sensorPoses.front();
        sensorPoses.pop_front();
      }
      else
      {
        FAIL() << "Unknown link found: " << msg.name();
      }
      EXPECT_EQ(expectedPose, p);
    }
    staticPoseTimestamps.pop_front();
  }
  mutex.unlock();
}

/////////////////////////////////////////////////
// See: https://github.com/gazebosim/gz-sim/issues/630
TEST_F(PosePublisherTest,
       GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(StaticPoseUpdateFrequency))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/pose_publisher.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that records the model's link poses
  test::Relay testSystem;

  {
    std::lock_guard<std::mutex> lock(mutex);
    staticPoseMsgs.clear();
  }

  // subscribe to the pose publisher
  transport::Node node;
  node.Subscribe(std::string("/model/test_static_pose_update_freq/pose_static"),
      &staticPoseCb);

  // Run server
  unsigned int iters = 1000u;
  server.Run(true, iters, false);

  std::size_t nExpMessages = 100;
  // Wait for 100 messages to be received
  bool received = false;
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    {
      std::lock_guard<std::mutex> lock(mutex);
      received = (staticPoseMsgs.size() >= nExpMessages);
    }

    if (received)
      break;
  }

  ASSERT_TRUE(received);

  {
    std::lock_guard<std::mutex> lock(mutex);
    // Calculate average frequency. The calculation is not very precise because
    // we only collect a small sample size but it should be enough to tell the
    // difference between 100 Hz and 1000 Hz
    std::chrono::duration<double> firstStamp =
        std::chrono::seconds(staticPoseMsgs.front().header().stamp().sec()) +
        std::chrono::nanoseconds(
        staticPoseMsgs.front().header().stamp().nsec());
    std::chrono::duration<double> lastStamp =
        std::chrono::seconds(staticPoseMsgs.back().header().stamp().sec()) +
        std::chrono::nanoseconds(staticPoseMsgs.back().header().stamp().nsec());

    double diff = (lastStamp - firstStamp).count();
    ASSERT_GT(diff, 0);
    double freq = staticPoseMsgs.size()/diff;
    EXPECT_NEAR(100.0, freq, 10.0);
  }
}

/////////////////////////////////////////////////
TEST_F(PosePublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(NestedModelLoadPlugin))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/nested_model.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  poseMsgs.clear();

  // test that the pose publisher plugin can be loaded by a nested model
  // by subscribe to the its topic
  transport::Node node;
  node.Subscribe(std::string("/model/model_00/model/model_01/pose"), &poseCb);

  // Run server
  unsigned int iters = 1000u;
  server.Run(true, iters, false);

  // Wait for messages to be received
  int sleep = 0;
  while (poseMsgs.empty() && sleep++ < 30)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(!poseMsgs.empty());

  // only the pose of the nested model should be published and no other entity
  std::string expectedEntityName = "model_00::model_01";
  math::Pose3d expectedEntityPose(1, 0, 0, 0, 0, 0);
  for (auto &msg : poseMsgs)
  {
    ASSERT_LT(1, msg.header().data_size());
    ASSERT_LT(0, msg.header().data(1).value_size());
    std::string childFrameId = msg.header().data(1).value(0);
    EXPECT_EQ(expectedEntityName, childFrameId);
    auto p = msgs::Convert(poseMsgs[0]);
    EXPECT_EQ(expectedEntityPose, p);
  }
}

/////////////////////////////////////////////////
TEST_F(PosePublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ModelPoseOnly))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/pose_publisher.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  poseMsgs.clear();

  // subscribe to the pose publisher
  transport::Node node;
  node.Subscribe(std::string("/model/test_publish_only_model_pose/pose"),
                 &poseCb);

  // Run server
  unsigned int iters = 1000u;
  server.Run(true, iters, false);

  // Wait for messages to be received
  int sleep = 0;
  while (poseMsgs.empty() && sleep++ < 30)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(!poseMsgs.empty());

  // only the pose of the model should be published and no other entity
  std::string expectedEntityName = "test_publish_only_model_pose";
  math::Pose3d expectedEntityPose(5, 5, 0, 0, 0, 0);
  for (auto &msg : poseMsgs)
  {
    ASSERT_LT(1, msg.header().data_size());
    ASSERT_LT(0, msg.header().data(1).value_size());
    std::string childFrameId = msg.header().data(1).value(0);
    EXPECT_EQ(expectedEntityName, childFrameId);
    auto p = msgs::Convert(poseMsgs[0]);
    EXPECT_EQ(expectedEntityPose, p);
  }
}

/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/PoseCmd.hh>
#include "gz/sim/components/PhysicsEnginePlugin.hh"
#include <gz/sim/Model.hh>
#include <gz/physics/ContactProperties.hh>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/FeaturePolicy.hh>
#include <gz/physics/InstallationDirectories.hh>
#include <gz/physics/config.hh>
#include <gz/plugin/Loader.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/Util.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define tol 10e-4

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

#define EXPECT_ANGLE_NEAR(a1, a2, tol) \
  EXPECT_LT(std::abs(math::Angle((a1) - (a2)).Normalized().Radian()), (tol)) \
    << (a1) << " vs. " << (a2)

// Verify that a model's world pose is near a specified pose.
void verifyPose(const math::Pose3d& pose1, const math::Pose3d& pose2)
{
  EXPECT_NEAR(pose1.Pos().X(), pose2.Pos().X(), 1e-1);
  EXPECT_NEAR(pose1.Pos().Y(), pose2.Pos().Y(), 1e-1);
  EXPECT_NEAR(pose1.Pos().Z(), pose2.Pos().Z(), 1e-2);
  EXPECT_ANGLE_NEAR(pose1.Rot().Roll(), pose2.Rot().Roll(), 1e-2);
  EXPECT_ANGLE_NEAR(pose1.Rot().Pitch(), pose2.Rot().Pitch(), 1e-2);
  EXPECT_ANGLE_NEAR(pose1.Rot().Yaw(), pose2.Rot().Yaw(), 1e-1);
}

/// \brief Helper function to wait until a predicate is true or a timeout occurs
/// \tparam Pred Predicate function of type bool()
/// \param[in] _timeoutMs Timeout in milliseconds
template <typename Pred>
bool waitUntil(int _timeoutMs, Pred _pred)
{
  using namespace std::chrono;
  auto tStart = steady_clock::now();
  auto sleepDur = milliseconds(std::min(100, _timeoutMs));
  auto waitDuration = milliseconds(_timeoutMs);
  while (duration_cast<milliseconds>(steady_clock::now() - tStart) <
         waitDuration)
  {
    if (_pred())
    {
      return true;
    }
    std::this_thread::sleep_for(sleepDur);
  }
  return false;
}

/// \brief Test TrackedVehicle system. This test drives a tracked robot over a
/// course of obstacles and verifies that it is able to climb on/over them.
class TrackedVehicleTest : public InternalFixture<::testing::Test>
{
  public: void SkipTestIfNotSupported(const EntityComponentManager &_ecm,
                                      bool &_shouldSkip)
  {
#if __APPLE__
    // until https://github.com/gazebosim/gz-sim/issues/806 is fixed
    _shouldSkip = true;
#else
    _shouldSkip = false;
    auto pluginLib =
        _ecm.ComponentData<components::PhysicsEnginePlugin>(worldEntity(_ecm));
    ASSERT_TRUE(pluginLib.has_value())
        << "PhysicsEnginePlugin component not found";

    // Find physics plugin (copied from the Physics system with some
    // modifications)
    common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv("GZ_SIM_PHYSICS_ENGINE_PATH");
    systemPaths.AddPluginPaths(gz::physics::getEngineInstallDir());

    auto pathToLib = systemPaths.FindSharedLibrary(*pluginLib);
    ASSERT_FALSE(pathToLib.empty())
        << "Failed to find plugin [" << *pluginLib << "]";

    // Load engine plugin
    plugin::Loader pluginLoader;
    auto plugins = pluginLoader.LoadLib(pathToLib);
    ASSERT_FALSE(plugins.empty())
        << "Unable to load the [" << pathToLib << "] library";

    // Check that we do have a valid physics engine. Otherwise, this should be a
    // failure not a skip.
    auto classNames = pluginLoader.PluginsImplementing<
        physics::ForwardStep::Implementation<
            physics::FeaturePolicy3d>>();
    ASSERT_FALSE(classNames.empty())
        << "No physics plugins found in library [" << pathToLib << "]";

    // Check if there are any plugins implementing
    // SetContactPropertiesCallbackFeature. If not, skip the test.
    auto contactProperties = pluginLoader.PluginsImplementing<
        physics::SetContactPropertiesCallbackFeature::Implementation<
            physics::FeaturePolicy3d>>();
    if (contactProperties.empty())
    {
      _shouldSkip = true;
    }
#endif
  }

  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _cmdVelTopic Command velocity topic.
  /// \param[in] _odomTopic Odometry topic.
  protected: void TestPublishCmd(const std::string &_sdfFile,
                                 const std::string &_cmdVelTopic,
                                 const std::string &_odomTopic)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    // Create a system that records the vehicle poses
    test::Relay ecmGetterSystem;
    EntityComponentManager* ecm {nullptr};
    ecmGetterSystem.OnPreUpdate([&ecm](const UpdateInfo &,
      EntityComponentManager &_ecm)
      {
        if (ecm == nullptr)
          ecm = &_ecm;
      });
    server.AddSystem(ecmGetterSystem.systemPtr);
    // Get ECM
    server.Run(true, 1, false);

    ASSERT_NE(nullptr, ecm);
    bool shouldSkipTest = false;
    this->SkipTestIfNotSupported(*ecm, shouldSkipTest);
    if (shouldSkipTest)
    {
      // Skip test if the ContactProperties feature is not available
      GTEST_SKIP() << "Skipping test because physics engine does not support "
        "SetContactPropertiesCallbackFeature";
    }

    test::Relay testSystem;
    Entity modelEntity {kNullEntity};
    std::vector<math::Pose3d> poses;
    testSystem.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        modelEntity = _ecm.EntityByComponents(
          components::Model(),
          components::Name("simple_tracked"));
        EXPECT_NE(kNullEntity, modelEntity);

        auto poseComp = _ecm.Component<components::Pose>(modelEntity);
        ASSERT_NE(nullptr, poseComp);

        poses.push_back(poseComp->Data());
      });
    server.AddSystem(testSystem.systemPtr);

    // Run server and check that vehicle didn't move
    server.Run(true, 1000, false);

    EXPECT_EQ(1000u, poses.size());

    for (size_t i = 101;  i < poses.size(); ++i)
    {
      SCOPED_TRACE(i);
      verifyPose(poses[100], poses[i]);
    }

    poses.clear();

    // Get odometry messages
    double period{1.0 / 50.0};
    double lastMsgTime{1.0};
    std::vector<math::Pose3d> odomPoses;
    std::function<void(const msgs::Odometry &)> odomCb =
      [&](const msgs::Odometry &_msg)
      {
        ASSERT_TRUE(_msg.has_header());
        ASSERT_TRUE(_msg.header().has_stamp());

        double msgTime =
            static_cast<double>(_msg.header().stamp().sec()) +
            static_cast<double>(_msg.header().stamp().nsec()) * 1e-9;

        EXPECT_DOUBLE_EQ(msgTime, lastMsgTime + period);
        lastMsgTime = msgTime;

        odomPoses.push_back(msgs::Convert(_msg.pose()));
      };

    // Publish command and check that vehicle moved
    transport::Node node;
    auto pub = node.Advertise<msgs::Twist>(_cmdVelTopic);
    node.Subscribe(_odomTopic, odomCb);

    msgs::Twist msg;
    msg.mutable_linear()->set_x(1.0);

    pub.Publish(msg);

    server.Run(true, 1000, false);

    // Poses for 1s
    ASSERT_EQ(1000u, poses.size());

    int sleep = 0;
    int maxSleep = 30;
    for (; odomPoses.size() < 50 && sleep < maxSleep; ++sleep)
    {
      std::this_thread::sleep_for(100ms);
    }
    ASSERT_NE(maxSleep, sleep);

    // Odom for 3s
    ASSERT_FALSE(odomPoses.empty());
    EXPECT_EQ(50u, odomPoses.size());

    EXPECT_LT(poses[0].Pos().X(), poses[999].Pos().X());
    EXPECT_NEAR(poses[0].Pos().Y(), poses[999].Pos().Y(), tol);
    EXPECT_NEAR(poses[0].Pos().Z(), poses[999].Pos().Z(), tol);
    EXPECT_ANGLE_NEAR(poses[0].Rot().X(), poses[999].Rot().X(), tol);
    EXPECT_ANGLE_NEAR(poses[0].Rot().Y(), poses[999].Rot().Y(), tol);
    EXPECT_ANGLE_NEAR(poses[0].Rot().Z(), poses[999].Rot().Z(), tol);

    // The robot starts at (3,0,0), so odom will have this shift.
    EXPECT_NEAR(poses[0].Pos().X(), odomPoses[0].Pos().X() + 3.0, 3e-2);
    EXPECT_NEAR(poses[0].Pos().Y(), odomPoses[0].Pos().Y(), 1e-2);
    EXPECT_NEAR(poses.back().Pos().X(), odomPoses.back().Pos().X() + 3, 1e-1);
    EXPECT_NEAR(poses.back().Pos().Y(), odomPoses.back().Pos().Y(), 1e-2);

    // Max velocities/accelerations expectations.
    // Moving time.
    double t = 1.0;
    double d = poses[999].Pos().Distance(poses[0].Pos());
    double v = d / t;
    EXPECT_LT(v, 1);

    poses.clear();

    Model model(modelEntity);

    // Move the robot somewhere to free space without obstacles.
    model.SetWorldPoseCmd(*ecm, math::Pose3d(10, 10, 0.1, 0, 0, 0));

    // Let the models settle down.
    server.Run(true, 300, false);

    // Test straight driving - 1 sec driving, should move 1 meter forward.

    const auto startPose = poses.back();

    const double linearSpeed = 1.0;
    msgs::Set(msg.mutable_linear(), math::Vector3d(linearSpeed, 0, 0));
    msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, 0));
    pub.Publish(msg);
    server.Run(true, 1000, false);

    EXPECT_NEAR(poses.back().Pos().X(), startPose.Pos().X() + linearSpeed, 0.1);
    EXPECT_NEAR(poses.back().Pos().Y(), startPose.Pos().Y(), 1e-1);
    EXPECT_NEAR(poses.back().Pos().Z(), startPose.Pos().Z(), 1e-2);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), startPose.Rot().Roll(), 1e-2);
    EXPECT_ANGLE_NEAR(
      poses.back().Rot().Pitch(), startPose.Rot().Pitch(), 1e-2);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Yaw(), startPose.Rot().Yaw(), 1e-1);

    // Test rotation in place - 1 sec rotation, should turn 0.25 rad.

    const auto middlePose = poses.back();

    // Take care when changing this value - if too high, it could get restricted
    // by the max speed of the tracks.
    const double rotationSpeed = 0.25;
    msgs::Set(msg.mutable_linear(), math::Vector3d(0, 0, 0));
    msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, rotationSpeed));
    pub.Publish(msg);
    server.Run(true, 1000, false);

    EXPECT_NEAR(poses.back().Pos().X(), middlePose.Pos().X(), 1e-1);
    EXPECT_NEAR(poses.back().Pos().Y(), middlePose.Pos().Y(), 1e-1);
    EXPECT_NEAR(poses.back().Pos().Z(), middlePose.Pos().Z(), 1e-2);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), middlePose.Rot().Roll(), 1e-2);
    EXPECT_ANGLE_NEAR(
      poses.back().Rot().Pitch(), middlePose.Rot().Pitch(), 1e-2);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Yaw(),
                      middlePose.Rot().Yaw() + rotationSpeed, 1e-1);

    // Test following a circular path.

    const auto lastPose = poses.back();

    msgs::Set(msg.mutable_linear(), math::Vector3d(0.5, 0, 0));
    msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, 0.2));
    pub.Publish(msg);
    server.Run(true, 1000, false);

    EXPECT_NEAR(poses.back().Pos().X(), lastPose.Pos().X() + 0.4, 1e-1);
    EXPECT_NEAR(poses.back().Pos().Y(), lastPose.Pos().Y() + 0.15, 1e-1);
    EXPECT_NEAR(poses.back().Pos().Z(), lastPose.Pos().Z(), 1e-2);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), lastPose.Rot().Roll(), 1e-2);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Pitch(), lastPose.Rot().Pitch(), 1e-2);
    EXPECT_ANGLE_NEAR(
      poses.back().Rot().Yaw(), lastPose.Rot().Yaw() + 0.2, 1e-1);

    // Test driving on staircase - should climb to its middle part.

    const auto beforeStairsPose = math::Pose3d(
      3, 0, 0.1,
      0, 0, 0);
    model.SetWorldPoseCmd(*ecm, beforeStairsPose);

    // Let the model settle down.
    server.Run(true, 300, false);

    msgs::Set(msg.mutable_linear(), math::Vector3d(linearSpeed, 0, 0));
    msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, 0));
    pub.Publish(msg);
    server.Run(true, 3500, false);

    EXPECT_NEAR(poses.back().Pos().X(), beforeStairsPose.X() + 3.4, 0.15);
    EXPECT_LE(poses.back().Pos().Y(), 0.7);
    EXPECT_GT(poses.back().Pos().Z(), 0.6);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), 0.0, 1e-1);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Pitch(), -0.4, 1e-1);
    EXPECT_ANGLE_NEAR(
      poses.back().Rot().Yaw(), beforeStairsPose.Rot().Yaw(), 1e-1);

    // Test driving over a cylinder

    const auto beforeCylinderPose = math::Pose3d(
      1, 0, 0.1,
      0, 0, -math::Angle::Pi.Radian());
    model.SetWorldPoseCmd(*ecm, beforeCylinderPose);

    // Let the model settle down.
    server.Run(true, 300, false);

    msgs::Set(msg.mutable_linear(), math::Vector3d(linearSpeed, 0, 0));
    msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, 0));
    pub.Publish(msg);
    server.Run(true, 2000, false);

    // The cylinder is at (0, 0, 0), we start at (0, 1, 0), and want to pass
    // at least a bit behind the cylinder (0, -1, 0). The driving is a bit wild,
    // so we don't care much about the end Y position and yaw.
    EXPECT_LT(poses.back().Pos().X(), -0.99);  // The driving is wild
    EXPECT_NEAR(poses.back().Pos().Y(), 0, 0.5);
    EXPECT_NEAR(poses.back().Pos().Z(), 0.0, 1e-1);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), 0.0, 1e-1);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Pitch(), 0.0, 1e-1);
    // The driving is wild
    EXPECT_ANGLE_NEAR(
      poses.back().Rot().Yaw(), beforeCylinderPose.Rot().Yaw(), 0.5);

    // Test driving over an obstacle that requires flippers. Without them, the
    // robot would get stuck in front of the obstacle.

    const auto beforeBoxPose = math::Pose3d(
      1, 2, 0.1,
      0, 0, -math::Angle::Pi.Radian());
    model.SetWorldPoseCmd(*ecm, beforeBoxPose);

    // Let the model settle down.
    server.Run(true, 300, false);

    // we go backwards because we have the CoG in the back
    msgs::Set(msg.mutable_linear(), math::Vector3d(-linearSpeed, 0, 0));
    msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, 0));
    pub.Publish(msg);
    server.Run(true, 4000, false);

    // The box is at (2, 2, 0), we start at (1, 2, 0), and want to pass
    // at least a bit behind the box (3.5, 2, 0). The driving is a bit wild.
    EXPECT_GT(poses.back().Pos().X(), 3.5);
    EXPECT_NEAR(poses.back().Pos().Y(), 2, 0.1);  // The driving is wild
    EXPECT_NEAR(poses.back().Pos().Z(), 0.0, 1e-1);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), 0.0, 1e-1);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Pitch(), 0.0, 1e-1);
    // The driving is wild
    EXPECT_ANGLE_NEAR(
      poses.back().Rot().Yaw(), beforeBoxPose.Rot().Yaw(), 0.25);
    // And we go back, which is a somewhat easier way

    msgs::Set(msg.mutable_linear(), math::Vector3d(linearSpeed, 0, 0));
    msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, 0));
    pub.Publish(msg);
    server.Run(true, 4000, false);

    // We start at (3.5, 2, 0), we go back, and it should be a bit faster than
    // the previous traversal, so we should end up beyond the starting point.
    EXPECT_LT(poses.back().Pos().X(), 1);
    EXPECT_NEAR(poses.back().Pos().Y(), 2, 0.1);  // The driving is wild
    EXPECT_NEAR(poses.back().Pos().Z(), 0.0, 1e-1);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), 0.0, 1e-1);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Pitch(), 0.0, 1e-1);
    // The driving is wild
    EXPECT_ANGLE_NEAR(
      poses.back().Rot().Yaw(), beforeBoxPose.Rot().Yaw(), 0.25);

    // Test that disabling the contact surface customization makes the vehicle
    // immobile

    ecm->Each<components::Collision>(
      [&](const Entity & _entity,
          const components::Collision */*_collision*/) -> bool
      {
        ecm->SetComponentData<components::EnableContactSurfaceCustomization>(
          _entity, false);
        return true;
      });

    model.SetWorldPoseCmd(*ecm, beforeCylinderPose);

    // Let the model settle down.
    server.Run(true, 300, false);

    msgs::Set(msg.mutable_linear(), math::Vector3d(linearSpeed, 0, 0));
    msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, 0));
    pub.Publish(msg);
    server.Run(true, 500, false);

    // Verify that the vehicle did not move
    poses.back().SetZ(beforeCylinderPose.Pos().Z());  // ignore Z offset
    verifyPose(poses.back(), beforeCylinderPose);
  }

  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _cmdVelTopic Command velocity topic.
  protected: void TestConveyor(const std::string &_sdfFile,
                               const std::string &_cmdVelTopic,
                               const std::string &_odometryTopic)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    // gz::transport node
    transport::Node node;

    // subscribe to odometry
    msgs::Odometry odometryMsg;
    unsigned int odomMsgCounter = 0;

    auto msgCb = std::function<void(const msgs::Odometry &)>(
    [&odometryMsg, &odomMsgCounter](const auto & msg)
    {
      odometryMsg = msg;
      odomMsgCounter++;
    });

    node.Subscribe(_odometryTopic, msgCb);

    // Create a system that records the vehicle poses
    test::Relay ecmGetterSystem;
    EntityComponentManager* ecm {nullptr};
    ecmGetterSystem.OnPreUpdate([&ecm](const UpdateInfo &,
      EntityComponentManager &_ecm)
      {
        if (ecm == nullptr)
          ecm = &_ecm;
      });
    server.AddSystem(ecmGetterSystem.systemPtr);
    // Get ECM
    server.Run(true, 1, false);

    ASSERT_NE(nullptr, ecm);
    bool shouldSkipTest = false;
    this->SkipTestIfNotSupported(*ecm, shouldSkipTest);
    if (shouldSkipTest)
    {
      // Skip test if the ContactProperties feature is not available
      GTEST_SKIP() << "Skipping test because physics engine does not support "
        "SetContactPropertiesCallbackFeature";
    }

    test::Relay testSystem;
    Entity boxEntity {kNullEntity};
    std::vector<math::Pose3d> poses;
    testSystem.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        boxEntity = _ecm.EntityByComponents(
          components::Model(),
          components::Name("box"));
        EXPECT_NE(kNullEntity, boxEntity);

        auto poseComp = _ecm.Component<components::Pose>(boxEntity);
        ASSERT_NE(nullptr, poseComp);

        poses.push_back(poseComp->Data());
      });
    server.AddSystem(testSystem.systemPtr);

    server.Run(true, 1000, false);

    // Wait for messages
    waitUntil(5000, [&]{return 50 == odomMsgCounter;});

    // Poses for 1s
    ASSERT_EQ(1000u, poses.size());
    ASSERT_EQ(50u, odomMsgCounter);

    // check that the box has not moved in X and Y directions (it will move in
    // Z as it falls down on the conveyor)
    EXPECT_NEAR(poses[0].Pos().X(), poses.back().Pos().X(), 1e-6);
    EXPECT_NEAR(poses[0].Pos().Y(), poses.back().Pos().Y(), 1e-6);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), 0, 1e-3);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Pitch(), 0, 1e-3);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Yaw(), 0, 1e-3);

    // check reported odometry pose and twist
    EXPECT_NEAR(0, odometryMsg.pose().position().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().position().y(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().position().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().w(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().linear().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().linear().y(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().linear().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().z(), 1e-6);

    poses.clear();
    odomMsgCounter = 0;

    // Publish command and check that vehicle moved
    auto pub = node.Advertise<msgs::Double>(_cmdVelTopic);

    // In this test, there is a long conveyor and a small box at its center.
    // The conveyor has max_velocity 0.5, max_acceleration 0.25 and command
    // timeout 2 seconds. So we expect the box will move slowly in the first
    // second, it will reach 0.5 meters in 2 seconds, and it will slow down
    // for the third second (deceleration is limited even for command timeout).

    msgs::Double msg;
    msg.set_data(1.0);

    pub.Publish(msg);

    server.Run(true, 1000, false);

    // Wait for messages
    waitUntil(5000, [&]{return 50 == odomMsgCounter;});

    // Poses for 1s
    ASSERT_EQ(1000u, poses.size());
    ASSERT_EQ(50u, odomMsgCounter);

    EXPECT_NEAR(0.125, poses.back().Pos().X(), 1e-1);
    EXPECT_NEAR(poses[0].Pos().Y(), poses.back().Pos().Y(), 1e-2);
    EXPECT_NEAR(poses[0].Pos().Z(), poses.back().Pos().Z(), 1e-2);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), 0, 1e-3);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Pitch(), 0, 1e-3);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Yaw(), 0, 1e-3);

    // check reported odometry pose and twist
    EXPECT_NEAR(0.125, odometryMsg.pose().position().x(), 1e-1);
    EXPECT_NEAR(0, odometryMsg.pose().position().y(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().position().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().w(), 1e-6);
    EXPECT_NEAR(0.25, odometryMsg.twist().linear().x(), 1e-1);
    EXPECT_NEAR(0, odometryMsg.twist().linear().y(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().linear().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().z(), 1e-6);

    server.Run(true, 1000, false);

    // Wait for messages
    waitUntil(5000, [&]{return 100 == odomMsgCounter;});

    // Poses for 2s
    ASSERT_EQ(2000u, poses.size());
    ASSERT_EQ(100u, odomMsgCounter);

    EXPECT_NEAR(0.5, poses.back().Pos().X(), 1e-1);
    EXPECT_NEAR(poses[0].Pos().Y(), poses.back().Pos().Y(), 1e-2);
    EXPECT_NEAR(poses[0].Pos().Z(), poses.back().Pos().Z(), 1e-2);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), 0, 1e-3);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Pitch(), 0, 1e-3);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Yaw(), 0, 1e-3);

    // check reported odometry pose and twist
    EXPECT_NEAR(0.5, odometryMsg.pose().position().x(), 1e-1);
    EXPECT_NEAR(0, odometryMsg.pose().position().y(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().position().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().w(), 1e-6);
    EXPECT_NEAR(0.5, odometryMsg.twist().linear().x(), 1e-1);
    EXPECT_NEAR(0, odometryMsg.twist().linear().y(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().linear().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().z(), 1e-6);

    server.Run(true, 1000, false);

    // Wait for messages
    waitUntil(5000, [&]{return 150 == odomMsgCounter;});

    // Poses for 3s
    ASSERT_EQ(3000u, poses.size());
    ASSERT_EQ(150u, odomMsgCounter);

    EXPECT_NEAR(0.875, poses.back().Pos().X(), 1e-1);
    EXPECT_NEAR(poses[0].Pos().Y(), poses.back().Pos().Y(), 1e-2);
    EXPECT_NEAR(poses[0].Pos().Z(), poses.back().Pos().Z(), 1e-2);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Roll(), 0, 1e-3);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Pitch(), 0, 1e-3);
    EXPECT_ANGLE_NEAR(poses.back().Rot().Yaw(), 0, 1e-3);

    // check reported odometry pose and twist
    EXPECT_NEAR(0.875, odometryMsg.pose().position().x(), 1e-1);
    EXPECT_NEAR(0, odometryMsg.pose().position().y(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().position().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.pose().orientation().w(), 1e-6);
    EXPECT_NEAR(0.25, odometryMsg.twist().linear().x(), 1e-1);
    EXPECT_NEAR(0, odometryMsg.twist().linear().y(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().linear().z(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().x(), 1e-6);
    EXPECT_NEAR(0, odometryMsg.twist().angular().z(), 1e-6);

    poses.clear();
  }
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(TrackedVehicleTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(PublishCmd))
{
  this->TestPublishCmd(
    std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/tracked_vehicle_simple.sdf",
    "/model/simple_tracked/cmd_vel",
    "/model/simple_tracked/odometry");
}

/////////////////////////////////////////////////
TEST_F(TrackedVehicleTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Conveyor))
{
  this->TestConveyor(
    std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/conveyor.sdf",
    "/model/conveyor/link/base_link/track_cmd_vel",
    "/model/conveyor/link/base_link/odometry");
}

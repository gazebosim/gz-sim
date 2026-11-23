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

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Model.hh"

#include "test_config.hh"
#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class BuoyancyEngineTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture::SetUp();
    this->pub = this->node.Advertise<gz::msgs::Double>(
      "/model/buoyant_box/buoyancy_engine/");
  }

  /// \brief Check the published volume and model height with no commands.
  /// \param[in] _model Name and topic namespace of the model to check.
  /// \param[in] _expectedVolume Expected bladder volume in m^3.
  protected: void CheckStartVolume(const std::string &_model,
      double _expectedVolume);

  /// \brief Node for communication
  public: gz::transport::Node node;

  /// \brief Publishes commands
  public: gz::transport::Node::Publisher pub;
};

/////////////////////////////////////////////////
void BuoyancyEngineTest::CheckStartVolume(const std::string &_model,
    double _expectedVolume)
{
  using namespace std::chrono_literals;
  constexpr std::size_t steps = 30;  // 3 s at the world's 100 ms step size.
  struct Samples
  {
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<double> volumes;
  };
  auto samples = std::make_shared<Samples>();
  std::function<void(const msgs::Double &)> onVolume =
      [samples](const msgs::Double &_msg)
      {
        std::lock_guard lock(samples->mutex);
        samples->volumes.push_back(_msg.data());
        samples->cv.notify_one();
      };
  ASSERT_TRUE(this->node.Subscribe(
      "/model/" + _model + "/buoyancy_engine/current_volume", onVolume));

  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "buoyancy_engine_start.sdf");
  serverConfig.SetSdfFile(sdfFile);
  Server server(serverConfig);
  server.SetUpdatePeriod(1ns);

  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
  {
    const Entity model = _ecm.EntityByComponents(
        components::Model(), components::Name(_model));
    const auto *pose = _ecm.Component<components::Pose>(model);
    ASSERT_NE(nullptr, pose);
    EXPECT_NEAR(0.0, pose->Data().Pos().Z(), 1e-6);
  });
  ASSERT_TRUE(server.AddSystem(testSystem.systemPtr).value_or(false));

  ASSERT_TRUE(server.Run(true, steps, false));
  std::unique_lock lock(samples->mutex);
  ASSERT_TRUE(samples->cv.wait_for(lock, 5s,
      [&] { return !samples->volumes.empty(); }));
  for (double volume : samples->volumes)
    EXPECT_NEAR(_expectedVolume, volume, 1e-9);
}

/////////////////////////////////////////////////
TEST_F(BuoyancyEngineTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(TestDefaultStartVolume))
{
  this->CheckStartVolume("default_engine", 3e-4);
}

/////////////////////////////////////////////////
TEST_F(BuoyancyEngineTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(TestStartVolumeClampedToMinimum))
{
  this->CheckStartVolume("raised_min_engine", 1e-3);
}

/////////////////////////////////////////////////
TEST_F(BuoyancyEngineTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(TestExplicitStartVolume))
{
  this->CheckStartVolume("explicit_engine", 6e-4);
}

/////////////////////////////////////////////////
TEST_F(BuoyancyEngineTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(TestDownward))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "buoyancy_engine.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::size_t iterations = 10000;

  test::Relay testSystem;
  std::vector<gz::math::Pose3d> poses;

  testSystem.OnPostUpdate([&](const sim::UpdateInfo &/*_info*/,
                             const sim::EntityComponentManager &_ecm)
  {
    // Check pose
    Entity buoyantBox = _ecm.EntityByComponents(
      components::Model(), components::Name("buoyant_box"));
    EXPECT_NE(kNullEntity, buoyantBox);

    auto submarineBuoyantPose = _ecm.Component<components::Pose>(buoyantBox);
    EXPECT_NE(submarineBuoyantPose, nullptr);
    if (submarineBuoyantPose == nullptr)
    {
      gzerr << "Unable to get pose" <<std::endl;
      return;
    }
    poses.push_back(submarineBuoyantPose->Data());
  });

  server.AddSystem(testSystem.systemPtr);
  gz::msgs::Double volume;
  volume.set_data(0);
  this->pub.Publish(volume);
  server.Run(true, iterations, false);

  EXPECT_LT(poses.rbegin()->Pos().Z(), poses.begin()->Pos().Z());
  EXPECT_NEAR(poses.rbegin()->Pos().X(), poses.begin()->Pos().X(), 1e-3);
  EXPECT_NEAR(poses.rbegin()->Pos().Y(), poses.begin()->Pos().Y(), 1e-3);
}

/////////////////////////////////////////////////
TEST_F(BuoyancyEngineTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(TestUpward))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "buoyancy_engine.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::size_t iterations = 10000;

  test::Relay testSystem;
  std::vector<gz::math::Pose3d> poses;

  testSystem.OnPostUpdate([&](const sim::UpdateInfo &/*_info*/,
                             const sim::EntityComponentManager &_ecm)
  {
    // Check pose
    Entity buoyantBox = _ecm.EntityByComponents(
      components::Model(), components::Name("buoyant_box"));
    EXPECT_NE(kNullEntity, buoyantBox);

    auto submarineBuoyantPose = _ecm.Component<components::Pose>(buoyantBox);
    EXPECT_NE(submarineBuoyantPose, nullptr);
    if (submarineBuoyantPose == nullptr)
    {
      gzerr << "Unable to get pose" <<std::endl;
      return;
    }
    poses.push_back(submarineBuoyantPose->Data());
  });

  server.AddSystem(testSystem.systemPtr);
  gz::msgs::Double volume;
  volume.set_data(10);
  this->pub.Publish(volume);
  server.Run(true, iterations, false);

  EXPECT_GT(poses.rbegin()->Pos().Z(), poses.begin()->Pos().Z());
  EXPECT_NEAR(poses.rbegin()->Pos().X(), poses.begin()->Pos().X(), 1e-3);
  EXPECT_NEAR(poses.rbegin()->Pos().Y(), poses.begin()->Pos().Y(), 1e-3);
}

/////////////////////////////////////////////////
TEST_F(BuoyancyEngineTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(TestUpwardSurface))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "buoyancy_engine.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::size_t iterations = 10000;

  test::Relay testSystem;
  std::vector<gz::math::Pose3d> poses;

  testSystem.OnPostUpdate([&](const sim::UpdateInfo &/*_info*/,
                             const sim::EntityComponentManager &_ecm)
  {
    // Check pose
    Entity buoyantBox = _ecm.EntityByComponents(
      components::Model(), components::Name("buoyant_box_w_surface"));
    EXPECT_NE(kNullEntity, buoyantBox);

    auto submarineBuoyantPose = _ecm.Component<components::Pose>(buoyantBox);
    EXPECT_NE(submarineBuoyantPose, nullptr);
    if (submarineBuoyantPose == nullptr)
    {
      gzerr << "Unable to get pose" <<std::endl;
      return;
    }
    poses.push_back(submarineBuoyantPose->Data());
    double tol = 0.1;
    EXPECT_LT(submarineBuoyantPose->Data().Z(), 0 + tol);
  });

  server.AddSystem(testSystem.systemPtr);
  gz::msgs::Double volume;
  volume.set_data(10);
  this->pub.Publish(volume);
  server.Run(true, iterations, false);

  EXPECT_GT(poses.rbegin()->Pos().Z(), poses.begin()->Pos().Z());
  EXPECT_NEAR(poses.rbegin()->Pos().X(), poses.begin()->Pos().X(), 1e-3);
  EXPECT_NEAR(poses.rbegin()->Pos().Y(), poses.begin()->Pos().Y(), 1e-3);
}

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

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/twist.pb.h>

#include <optional>
#include <regex>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Model.hh"
#include "test_config.hh"

#include "helpers/Relay.hh"
#include "helpers/UniqueTestDirectoryEnv.hh"
#include "helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class BreadcrumbsTest : public InternalFixture<::testing::Test>
{
  public: void LoadWorld(const std::string &_path, bool _useLevels = false)
  {
    this->serverConfig.SetResourceCache(test::UniqueTestDirectoryEnv::Path());
    this->serverConfig.SetSdfFile(
        common::joinPaths(PROJECT_SOURCE_PATH, _path));
    this->serverConfig.SetUseLevels(_useLevels);

    this->server = std::make_unique<Server>(this->serverConfig);
    EXPECT_FALSE(this->server->Running());
    EXPECT_FALSE(*this->server->Running(0));
    using namespace std::chrono_literals;
    this->server->SetUpdatePeriod(1ns);
  }

  public: ServerConfig serverConfig;
  public: std::unique_ptr<Server> server;
};

int kRemaining{-1};

/////////////////////////////////////////////////
void remainingCb(const msgs::Int32 &_msg)
{
  kRemaining = _msg.data();
}

/////////////////////////////////////////////////
// This test checks the .../deploy/remaining topic
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(BreadcrumbsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Remaining))
{
  // Start server
  this->LoadWorld(common::joinPaths("test", "worlds", "breadcrumbs.sdf"));
  kRemaining = 0;

  test::Relay testSystem;
  transport::Node node;
  auto deployB1 =
      node.Advertise<msgs::Empty>("/model/vehicle_blue/breadcrumbs/B1/deploy");
  node.Subscribe("/model/vehicle_blue/breadcrumbs/B1/deploy/remaining",
      &remainingCb);
  EXPECT_EQ(0, kRemaining);
  kRemaining = -1;

  deployB1.Publish(msgs::Empty());
  int sleep = 0;
  int maxSleep = 30;
  for (; kRemaining != 2 && sleep < maxSleep; ++sleep)
  {
    this->server->Run(true, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_EQ(2, kRemaining);
  kRemaining = -1;

  deployB1.Publish(msgs::Empty());
  sleep = 0;
  for (; kRemaining != 1 && sleep < maxSleep; ++sleep)
  {
    this->server->Run(true, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_EQ(1, kRemaining);
  kRemaining = -1;

  deployB1.Publish(msgs::Empty());
  sleep = 0;
  for (; kRemaining != 0 && sleep < maxSleep; ++sleep)
  {
    this->server->Run(true, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_EQ(0, kRemaining);
  kRemaining = -1;

  deployB1.Publish(msgs::Empty());
  sleep = 0;
  for (; kRemaining != 0 && sleep < maxSleep; ++sleep)
  {
    this->server->Run(true, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_EQ(0, kRemaining);
}

/////////////////////////////////////////////////
// The test checks breadcrumbs are deployed at the correct pose
TEST_F(BreadcrumbsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(DeployAtOffset))
{
  // Start server
  this->LoadWorld(common::joinPaths("test", "worlds", "breadcrumbs.sdf"));

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deployB1 =
      node.Advertise<msgs::Empty>("/model/vehicle_blue/breadcrumbs/B1/deploy");

  std::size_t iterTestStart = 1000;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
  {
    // Start moving the vehicle
    // After 1000 iterations, stop the vehicle, spawn a breadcrumb
    // Check the pose of the breadcrumb
    if (_info.iterations == iterTestStart)
    {
      msgs::Twist msg;
      msg.mutable_linear()->set_x(0.5);
      cmdVel.Publish(msg);
    }
    else if (_info.iterations == iterTestStart + 1000)
    {
      // Stop vehicle
      cmdVel.Publish(msgs::Twist());
    }
    else if (_info.iterations == iterTestStart + 1200)
    {
      // Deploy
      deployB1.Publish(msgs::Empty());
    }
    else if (_info.iterations == iterTestStart + 2000)
    {
      // Check pose
      Entity vehicleBlue = _ecm.EntityByComponents(
          components::Model(), components::Name("vehicle_blue"));
      ASSERT_NE(vehicleBlue, kNullEntity);

      auto poseVehicle = _ecm.Component<components::Pose>(vehicleBlue);
      ASSERT_NE(poseVehicle , nullptr);

      EXPECT_GT(poseVehicle->Data().Pos().X(), 0.4);

      // The first breadcrumb
      Entity b1 = _ecm.EntityByComponents(components::Model(),
                                          components::Name("B1_0"));
      ASSERT_NE(b1, kNullEntity);
      auto poseB1 = _ecm.Component<components::Pose>(b1);

      ASSERT_NE(poseB1, nullptr);

      auto poseDiff = poseVehicle->Data().Inverse() * poseB1->Data();
      EXPECT_NEAR(poseDiff.Pos().X(), -1.2, 1e-2);
    }
  });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, iterTestStart + 2001, false);
}

/////////////////////////////////////////////////
// The test checks max deployments
TEST_F(BreadcrumbsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(MaxDeployments))
{
  // Start server
  this->LoadWorld(common::joinPaths("test", "worlds", "breadcrumbs.sdf"));

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deployB1 =
      node.Advertise<msgs::Empty>("/model/vehicle_blue/breadcrumbs/B1/deploy");

  std::size_t iterTestStart = 1000;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
  {
    // Start moving the vehicle
    // Every 1000 iterations, deploy
    // After 5000 iterations, check the number of breadcrumbs deployed
    if (_info.iterations == iterTestStart)
    {
      msgs::Twist msg;
      msg.mutable_linear()->set_x(0.5);
      cmdVel.Publish(msg);
    }
    else if ((_info.iterations + iterTestStart + 1) % 1000 == 0)
    {
      // Deploy
      deployB1.Publish(msgs::Empty());
    }
    else if (_info.iterations == iterTestStart + 5000)
    {
      std::size_t breadCrumbCount{0};
      _ecm.Each<components::Model, components::Name>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name)
          {
            if (_name->Data().find("B1") == 0)
            {
              ++breadCrumbCount;
            }

            return true;
          });

      EXPECT_EQ(breadCrumbCount, 3u);
    }
  });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, iterTestStart + 5001, false);
}


/////////////////////////////////////////////////
// The test checks that including models from fuel works. Also checks custom
// topic
TEST_F(BreadcrumbsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(FuelDeploy))
{
  // Start server
  this->LoadWorld(common::joinPaths("test", "worlds", "breadcrumbs.sdf"));

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deploy = node.Advertise<msgs::Empty>("/fuel_deploy");

  const std::size_t iterTestStart = 1000;
  const std::size_t nIters = iterTestStart + 2500;
  const std::size_t maxDeployments = 5;
  std::size_t deployCount = 0;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
  {
    // Start moving the vehicle
    // Every 500 iterations, deploy
    // After 3500 iterations, check the number of breadcrumbs deployed
    if ((_info.iterations + iterTestStart + 1) % 500 == 0)
    {
      if (deployCount < maxDeployments)
      {
        deploy.Publish(msgs::Empty());
        ++deployCount;
      }
    }
    else if (_info.iterations == nIters)
    {
      std::size_t breadCrumbCount{0};
      _ecm.Each<components::Model, components::Name>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name)
          {
            if (_name->Data().find("B2") == 0)
            {
              ++breadCrumbCount;
            }

            return true;
          });

      EXPECT_EQ(breadCrumbCount, deployCount);
    }
  });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, nIters, false);
}

/////////////////////////////////////////////////
// The test checks that breadcrumbs can be performers
TEST_F(BreadcrumbsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Performer))
{
  // Start server
  this->LoadWorld(common::joinPaths("test", "worlds", "breadcrumbs.sdf"));

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deploy = node.Advertise<msgs::Empty>(
      "/model/vehicle_blue/breadcrumbs/B1_perf/deploy");

  const std::size_t iterTestStart = 1000;
  const std::size_t nIters = iterTestStart + 10000;

  std::optional<math::Pose3d> initialPose;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
  {
    // Deploy a performer breadcrumb on a tile that's on a level, and ensure
    // that it keeps the tile from being unloaded.

    // Deploy, move outside of the level, check
    if (_info.iterations == iterTestStart)
    {
      msgs::Twist msg;
      msg.mutable_linear()->set_x(2);
      cmdVel.Publish(msg);
      deploy.Publish(msgs::Empty());
    }
    else if (_info.iterations == iterTestStart + 1000)
    {
      // get the initial pose of the performer while the level is loaded
      _ecm.Each<components::Model, components::Name, components::Pose>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name, const components::Pose *_pose)
          {
            if (_name->Data().find("B1_perf") == 0)
            {
              initialPose = _pose->Data();
              return false;
            }
            return true;
          });
    }
    else if (_info.iterations == nIters)
    {
      math::Pose3d finalPose;
      // get the final pose of the performer
      _ecm.Each<components::Model, components::Name, components::Pose>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name, const components::Pose *_pose)
          {
            if (_name->Data().find("B1_perf") == 0)
            {
              finalPose = _pose->Data();
              return false;
            }

            return true;
          });
      ASSERT_TRUE(initialPose.has_value());
      gzdbg << "Init: " << initialPose->Pos() << " Final: "
                << finalPose.Pos() << std::endl;
      EXPECT_NEAR(initialPose->Pos().Z(), finalPose.Pos().Z(), 1e-3);
    }
  });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, nIters, false);
}

/////////////////////////////////////////////////
// Test that the volume of the performer is set when deploying a performer
// breadcrumb
TEST_F(BreadcrumbsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(PerformerSetVolume))
{
  // Start server
  this->LoadWorld(common::joinPaths("test", "worlds", "breadcrumbs.sdf"), true);

  test::Relay testSystem;
  transport::Node node;
  auto deploy = node.Advertise<msgs::Empty>(
      "/model/vehicle_blue/breadcrumbs/B1_perf_large_volume/deploy");

  const std::size_t iterTestStart = 1000;
  const std::size_t nIters = iterTestStart + 2000;

  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
  {
    // Deploy a performer breadcrumb on a tile that's on the default a level,
    // and check that it causes tile_1 to be loaded since the performer's volume
    // is large enough to overlap with tile_1.

    // Set pose to tile_2 , check if tile_1 is still loaded
    if (_info.iterations == iterTestStart)
    {
      msgs::Pose req;
      req.set_name("vehicle_blue");
      req.mutable_position()->set_x(30);
      msgs::Boolean rep;
      bool result;
      node.Request("/world/breadcrumbs/set_pose", req, 2000, rep, result);
      EXPECT_TRUE(result);
    }
    else if (_info.iterations == iterTestStart + 500)
    {
      // Check that tile_1 is unloaded
      Entity tileEntity = _ecm.EntityByComponents(components::Model(),
                                                  components::Name("tile_1"));
      EXPECT_EQ(kNullEntity, tileEntity);

      deploy.Publish(msgs::Empty());
    }
    else if (_info.iterations == nIters)
    {
      Entity tileEntity = _ecm.EntityByComponents(components::Model(),
                                                  components::Name("tile_1"));
      EXPECT_NE(kNullEntity, tileEntity);
    }
  });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, nIters, false);
}

/////////////////////////////////////////////////
// The test verifies breadcrumbs physics is disabled using disable_physics_time
TEST_F(BreadcrumbsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(DeployDisablePhysics))
{
  // Start server
  this->LoadWorld(common::joinPaths("test", "worlds", "breadcrumbs.sdf"));

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deployB2 =
      node.Advertise<msgs::Empty>("/model/vehicle_blue/breadcrumbs/B2/deploy");

  std::size_t iterTestStart = 1000;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
  {
    // Start moving the vehicle
    // After 1000 iterations, stop the vehicle, spawn a breadcrumb
    // Check the pose of the breadcrumb
    if (_info.iterations == iterTestStart)
    {
      msgs::Twist msg;
      msg.mutable_linear()->set_x(0.5);
      cmdVel.Publish(msg);
    }
    else if (_info.iterations == iterTestStart + 1000)
    {
      // Stop vehicle
      cmdVel.Publish(msgs::Twist());
    }
    else if (_info.iterations == iterTestStart + 1200)
    {
      // Deploy
      deployB2.Publish(msgs::Empty());
    }
    else if (_info.iterations == iterTestStart + 2000)
    {
      // Check pose
      Entity vehicleBlue = _ecm.EntityByComponents(
          components::Model(), components::Name("vehicle_blue"));
      ASSERT_NE(vehicleBlue, kNullEntity);

      auto poseVehicle = _ecm.Component<components::Pose>(vehicleBlue);
      ASSERT_NE(poseVehicle , nullptr);

      EXPECT_GT(poseVehicle->Data().Pos().X(), 0.4);

      // The first breadcrumb
      Entity b2 = _ecm.EntityByComponents(components::Model(),
                                          components::Name("B2_0"));
      ASSERT_NE(b2, kNullEntity);
      auto poseB2 = _ecm.Component<components::Pose>(b2);

      ASSERT_NE(poseB2, nullptr);

      // check pose of breadcrumb
      auto poseDiff = poseVehicle->Data().Inverse() * poseB2->Data();
      EXPECT_NEAR(-2.2, poseDiff.Pos().X(), 1e-2);
      EXPECT_NEAR(0.0, poseDiff.Pos().Y(), 1e-2);

      // Verify that the breadcrumb stopped falling after 0.5s.
      sdf::Root root;
      root.Load(this->serverConfig.SdfFile());
      const sdf::World *world = root.WorldByIndex(0);
      double gz = world->Gravity().Z();
      double z0 = 2.0;
      double t = 0.5;
      double z = z0 + gz/2*t*t;
      EXPECT_NEAR(z, poseDiff.Pos().Z(), 1e-2);
    }
  });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, iterTestStart + 2001, false);
}

/////////////////////////////////////////////////
// The test verifies that if allow_renaming is true, the Breadcrumb system
// renames spawned models if a model with the same name exists.
TEST_F(BreadcrumbsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(AllowRenaming))
{
  // Start server
  this->LoadWorld("test/worlds/breadcrumbs.sdf");

  transport::Node node;
  auto deployB1 =
      node.Advertise<msgs::Empty>("/model/vehicle_blue/breadcrumbs/B1/deploy");
  auto noRenameDeploy =
      node.Advertise<msgs::Empty>("/no_rename_deploy");
  auto renameDeploy =
      node.Advertise<msgs::Empty>("/rename_deploy");

  this->server->Run(true, 1, false);
  deployB1.Publish(msgs::Empty());
  this->server->Run(true, 100, false);
  EXPECT_TRUE(this->server->HasEntity("B1_0"));

  // Deploying via "/no_rename_deploy" will try to spawn B1_0, but since the
  // model already exists, the spawn should fail.
  auto curEntityCount = this->server->EntityCount().value();
  noRenameDeploy.Publish(msgs::Empty());
  this->server->Run(true, 100, false);
  EXPECT_EQ(curEntityCount, this->server->EntityCount().value());

  // Deploying via "/rename_deploy" will try to spawn B1_0, but since the
  // model already exists, it will spawn B1_0_1 instead.
  renameDeploy.Publish(msgs::Empty());
  this->server->Run(true, 100, false);
  EXPECT_TRUE(this->server->HasEntity("B1_0_1"));
}

/////////////////////////////////////////////////
/// Return a list of model entities whose names match the given regex
std::vector<Entity> ModelsByNameRegex(
    const EntityComponentManager &_ecm, const std::regex &_re)
{
  std::vector<Entity> entities;
  _ecm.Each<components::Model, components::Name>(
      [&](const Entity _entity, const components::Model *,
          const components::Name *_name)
      {
        if (std::regex_match(_name->Data(), _re))
        {
          entities.push_back(_entity);
        }
        return true;
      });

  return entities;
}

// The test checks that models containing Breadcrumbs can be unloaded and loaded
// safely
TEST_F(BreadcrumbsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LevelLoadUnload))
{
  // Start server
  this->LoadWorld(
      common::joinPaths("test", "worlds", "breadcrumbs_levels.sdf"), true);

  test::Relay testSystem;
  transport::Node node;
  auto deploy =
      node.Advertise<msgs::Empty>("/model/tile_1/breadcrumbs/B1/deploy");

  const std::size_t iterTestStart = 1000;
  const std::size_t nIters = iterTestStart + 10000;

  std::regex reTile1{"tile_1"};
  std::regex reBreadcrumb{"B1_.*"};
  testSystem.OnPostUpdate(
      [&](const UpdateInfo &_info,
          const EntityComponentManager &_ecm)
      {
        // Ensure that tile_1 is loaded at the start, deploy a breadcrumb
        if (_info.iterations == iterTestStart)
        {
          auto tiles = ModelsByNameRegex(_ecm, reTile1);
          EXPECT_EQ(1u, tiles.size());
          EXPECT_TRUE(deploy.Publish(msgs::Empty()));
        }
        // Check if the breadcrumb has been deployed
        else if (_info.iterations == iterTestStart + 1000)
        {
          auto breadcrumbs = ModelsByNameRegex(_ecm, reBreadcrumb);
          EXPECT_EQ(1u, breadcrumbs.size());
        }
        // Move the performer (sphere) outside the level. This should cause
        // tile_1 to be unloaded
        else if (_info.iterations == iterTestStart + 1001)
        {
          msgs::Pose req;
          req.set_name("sphere");
          req.mutable_position()->set_x(30);
          msgs::Boolean rep;
          bool result;
          node.Request("/world/breadcrumbs_levels/set_pose", req, 1000, rep,
                       result);
          EXPECT_TRUE(result);
        }
        // Check that tile_1 is unloaded, then try deploying breadcrumb. This
        // should fail because the model associated with the breadcrumb system
        // is unloaded.
        else if (_info.iterations == iterTestStart + 2000)
        {
          auto tiles = ModelsByNameRegex(_ecm, reTile1);
          EXPECT_EQ(0u, tiles.size());
          EXPECT_TRUE(deploy.Publish(msgs::Empty()));
        }
        // Check that no new breadcrumbs have been deployed
        else if (_info.iterations == iterTestStart + 3000)
        {
          auto breadcrumbs = ModelsByNameRegex(_ecm, reBreadcrumb);
          EXPECT_EQ(1u, breadcrumbs.size());
        }
        // Move the performer (sphere) back into the level. This should load
        // tile_1 and a new Breadcrumbs system.
        else if (_info.iterations == iterTestStart + 3001)
        {
          msgs::Pose req;
          req.set_name("sphere");
          req.mutable_position()->set_x(0);
          msgs::Boolean rep;
          bool result;
          node.Request("/world/breadcrumbs_levels/set_pose", req, 1000, rep,
                       result);
          EXPECT_TRUE(result);
        }
        // Check that tile_1 is loaded, then try deploying breadcrumb.
        else if (_info.iterations == iterTestStart + 4000)
        {
          auto tiles = ModelsByNameRegex(_ecm, reTile1);
          EXPECT_EQ(1u, tiles.size());
          EXPECT_TRUE(deploy.Publish(msgs::Empty()));
        }
        // Check that only one additional breadcrumb has been deployed even
        // though there are two Breadcrumbs systems associated with the model.
        // The original Breadcrumbs system should now be invalid because the
        // model gets a new Entity ID.
        else if (_info.iterations == iterTestStart + 5000)
        {
          auto breadcrumbs = ModelsByNameRegex(_ecm, reBreadcrumb);
          EXPECT_EQ(2u, breadcrumbs.size());
        }
      });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, nIters, false);
}

/////////////////////////////////////////////////
/// Main
int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  ::testing::AddGlobalTestEnvironment(
      new test::UniqueTestDirectoryEnv("breadcrumbs_test_cache"));
  return RUN_ALL_TESTS();
}

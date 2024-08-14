/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#include <string.h>
#include <vector>

#include <gz/msgs/double.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/World.hh"

#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class LighterThanAirDynamicsTest : public InternalFixture<::testing::Test>
{
  /// \brief Test a world file
  /// \param[in] _world Path to world file
  /// \param[in] _namespace Namespace for topic
  public: std::vector<math::Vector3d> TestWorld(const std::string &_world,
   const std::string &_namespace);

  public: std::vector<std::pair<math::Pose3d, math::Vector3d>>
    TestUnstableYaw(
    const std::string &_world, const std::string &_namespace);

  public: std::vector<std::pair<math::Pose3d, math::Vector3d>>
    TestUnstablePitch(
      const std::string &_world, const std::string &_namespace);

  private: std::vector<math::Pose3d> worldPoses;
  private: std::vector<math::Vector3d> bodyAngularVels;
  private: std::vector<std::pair<math::Pose3d, math::Vector3d>> state;
};


//////////////////////////////////////////////////
std::vector<std::pair<math::Pose3d, math::Vector3d>>
LighterThanAirDynamicsTest::TestUnstablePitch(
const std::string &_world, const std::string &_namespace)
{
  // Maximum verbosity for debugging
  common::Console::SetVerbosity(4);

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(_world);

  TestFixture fixture(serverConfig);

  Model model;
  Link body;

  worldPoses.clear();
  bodyAngularVels.clear();
  state.clear();

  fixture.
  OnConfigure(
    [&](const Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      EntityComponentManager &_ecm,
      EventManager &/*eventMgr*/)
    {
      World world(_worldEntity);

      auto modelEntity =  world.ModelByName(_ecm, _namespace);
      EXPECT_NE(modelEntity, kNullEntity);
      model = Model(modelEntity);

      auto bodyEntity = model.LinkByName(_ecm, _namespace + "_link");
      EXPECT_NE(bodyEntity, kNullEntity);

      body = Link(bodyEntity);
      body.EnableVelocityChecks(_ecm);

      auto WorldPose = body.WorldPose(_ecm);

      math::Matrix3d DCM(WorldPose.value().Rot());
      // Small disturbance to enduce munk moment
      math::Vector3d body_z_force(0, 0, 100);
      math::Vector3d world_frame_force = DCM * body_z_force;

      body.AddWorldForce(_ecm, world_frame_force);

    }).
  OnPostUpdate([&](const UpdateInfo &/*_info*/,
                            const EntityComponentManager &_ecm)
    {
      auto worldPose = body.WorldPose(_ecm);
      math::Vector3d bodyAngularVel = \
        (body.WorldPose(_ecm)).value().Rot().Inverse() * \
          body.WorldAngularVelocity(_ecm).value();
      ASSERT_TRUE(worldPose);
      worldPoses.push_back(worldPose.value());
      bodyAngularVels.push_back(bodyAngularVel);
      state.push_back(std::make_pair(worldPose.value(), bodyAngularVel));
    }).
  OnPreUpdate([&](const UpdateInfo &/*_info*/,
                            EntityComponentManager &_ecm)
    {
      auto WorldPose = body.WorldPose(_ecm);

      math::Matrix3d DCM(WorldPose.value().Rot());
      math::Vector3d body_x_force(100, 0, 0);

      math::Vector3d world_frame_force = DCM * body_x_force;

      body.AddWorldForce(_ecm, world_frame_force);
    }).
  Finalize();

  fixture.Server()->Run(true, 2000, false);
  EXPECT_EQ(2000u, worldPoses.size());

  EXPECT_NE(model.Entity(), kNullEntity);
  EXPECT_NE(body.Entity(), kNullEntity);

  return state;
}

//////////////////////////////////////////////////
std::vector<std::pair<math::Pose3d, math::Vector3d>>
  LighterThanAirDynamicsTest::TestUnstableYaw(
  const std::string &_world, const std::string &_namespace)
{
  // Maximum verbosity for debugging
  common::Console::SetVerbosity(4);

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(_world);

  TestFixture fixture(serverConfig);

  Model model;
  Link body;
  worldPoses.clear();
  bodyAngularVels.clear();
  state.clear();
  fixture.
  OnConfigure(
    [&](const Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      EntityComponentManager &_ecm,
      EventManager &/*eventMgr*/)
    {
      World world(_worldEntity);

      auto modelEntity =  world.ModelByName(_ecm, _namespace);
      EXPECT_NE(modelEntity, kNullEntity);
      model = Model(modelEntity);

      auto bodyEntity = model.LinkByName(_ecm, _namespace + "_link");
      EXPECT_NE(bodyEntity, kNullEntity);

      body = Link(bodyEntity);
      body.EnableVelocityChecks(_ecm);

      auto WorldPose = body.WorldPose(_ecm);

      math::Matrix3d DCM(WorldPose.value().Rot());
      // Small disturbance to enduce some lateral movement
      // to allow munk moment to grow
      math::Vector3d body_y_force(0, 10, 0);
      math::Vector3d world_frame_force = DCM * body_y_force;

      body.AddWorldForce(_ecm, world_frame_force);

    }).
  OnPostUpdate([&](const UpdateInfo &/*_info*/,
                            const EntityComponentManager &_ecm)
    {
      auto worldPose = body.WorldPose(_ecm);
      math::Vector3d bodyAngularVel =
        (body.WorldPose(_ecm)).value().Rot().Inverse() * \
        body.WorldAngularVelocity(_ecm).value();
      ASSERT_TRUE(worldPose);
      worldPoses.push_back(worldPose.value());
      bodyAngularVels.push_back(bodyAngularVel);
      state.push_back(std::make_pair(worldPose.value(), bodyAngularVel));

    }).
  OnPreUpdate([&](const UpdateInfo &/*_info*/,
                            EntityComponentManager &_ecm)
    {
      auto WorldPose = body.WorldPose(_ecm);

      math::Matrix3d DCM(WorldPose.value().Rot());
      math::Vector3d body_x_force(100, 0, 0);

      math::Vector3d world_frame_force = DCM * body_x_force;

      body.AddWorldForce(_ecm, world_frame_force);
    }).
  Finalize();

  fixture.Server()->Run(true, 2000, false);
  EXPECT_EQ(2000u, worldPoses.size());

  EXPECT_NE(model.Entity(), kNullEntity);
  EXPECT_NE(body.Entity(), kNullEntity);

  return state;
}

//////////////////////////////////////////////////
// Test if the plugin can be loaded succesfully and
// runs
std::vector<math::Vector3d> LighterThanAirDynamicsTest::TestWorld(
  const std::string &_world, const std::string &_namespace)
{
  // Maximum verbosity for debugging
  common::Console::SetVerbosity(4);

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(_world);

  TestFixture fixture(serverConfig);

  Model model;
  Link body;
  std::vector<math::Vector3d> bodyVels;
  fixture.
  OnConfigure(
    [&](const Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      EntityComponentManager &_ecm,
      EventManager &/*eventMgr*/)
    {
      World world(_worldEntity);

      auto modelEntity =  world.ModelByName(_ecm, _namespace);
      EXPECT_NE(modelEntity, kNullEntity);
      model = Model(modelEntity);

      auto bodyEntity = model.LinkByName(_ecm, _namespace + "_link");
      EXPECT_NE(bodyEntity, kNullEntity);

      body = Link(bodyEntity);
      body.EnableVelocityChecks(_ecm);

    }).
  OnPostUpdate([&](const UpdateInfo &/*_info*/,
                            const EntityComponentManager &_ecm)
    {
      auto bodyVel = body.WorldLinearVelocity(_ecm);
      ASSERT_TRUE(bodyVel);
      bodyVels.push_back(bodyVel.value());
    }).
  Finalize();

  fixture.Server()->Run(true, 2000, false);
  EXPECT_EQ(2000u, bodyVels.size());

  EXPECT_NE(model.Entity(), kNullEntity);
  EXPECT_NE(body.Entity(), kNullEntity);

  return bodyVels;
}

/////////////////////////////////////////////////
/// This test evaluates whether the lighter-than-air-dynamics plugin
/// is loaded successfully and is stable
TEST_F(LighterThanAirDynamicsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SpawnModel))
{
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "lighter_than_air_dynamics.sdf");

  this->TestWorld(world, "hull");

}

/////////////////////////////////////////////////
// Test whether the yaw of the hull will grow
// due to a a small disturbance in its lateral system
TEST_F(LighterThanAirDynamicsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(
  UnstableMunkMomentInYaw))
{
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "lighter_than_air_dynamics.sdf");

  auto states = this->TestUnstableYaw(world, "hull");
  auto states2 = this->TestUnstableYaw(world, "hull2");

  math::Vector3d ang_vel = states.back().second;
  math::Vector3d end_pos = states.back().first.Pos();
  math::Quaterniond end_rot = states.back().first.Rot();

  math::Vector3d ang_vel2 = states2.back().second;
  math::Vector3d end_pos2 = states2.back().first.Pos();
  math::Quaterniond end_rot2 = states2.back().first.Rot();

  //  Drag from hull should result in translating less than an object with no
  // aerodynamic drag
  EXPECT_LE(end_pos.X(), end_pos2.X());

  // Due to the munk moment, the yaw of the aircraft should grow
  EXPECT_GT(abs(end_rot.Euler().Z()), abs(end_rot2.Euler().Z()));

  // Due to the munk moment, the yawrate needs to increase
  EXPECT_GT(abs(ang_vel.Z()), abs(ang_vel2.Z()));

}

/////////////////////////////////////////////////
// Test whether the pitch of the hull will grow
// due to a a small disturbance in its longitudinal system
TEST_F(LighterThanAirDynamicsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(
  UnstableMunkMomentInPitch))
{
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "lighter_than_air_dynamics.sdf");

  auto states = this->TestUnstablePitch(world, "hull");
  auto states2 = this->TestUnstablePitch(world, "hull2");

  math::Vector3d ang_vel = states.back().second;
  math::Vector3d end_pos = states.back().first.Pos();
  math::Quaterniond end_rot = states.back().first.Rot();

  math::Vector3d ang_vel2 = states2.back().second;
  math::Vector3d end_pos2 = states2.back().first.Pos();
  math::Quaterniond end_rot2 = states2.back().first.Rot();

  //  Drag from hull should result in translating less than an object with no
  // aerodynamic drag
  EXPECT_LE(end_pos.X(), end_pos2.X());

  // Due to the munk moment, the yaw of the aircraft should grow
  EXPECT_GT(abs(end_rot.Euler().Y()), abs(end_rot2.Euler().Y()));

  // Due to the munk moment, the yawrate needs to increase
  EXPECT_GT(abs(ang_vel.Y()), abs(ang_vel2.Y()));

}

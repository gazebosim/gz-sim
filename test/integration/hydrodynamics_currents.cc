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

#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/World.hh"

#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Ocean currents reaching the hydrodynamics plugin: from the topic,
/// and from environmental data that varies in space and time.
///
/// The bodies here carry no plugin added mass: a step change of the current,
/// as a topic message or a table that turns, is a step in the relative
/// velocity, and the legacy added-mass term finite-differences that into a
/// force spike.
class HydrodynamicsCurrentsTest : public InternalFixture<::testing::Test>
{
  /// \brief Run a world and record the world linear velocity of a model's
  /// link, named <model>_link, every iteration.
  /// \param[in] _world Path to the world file
  /// \param[in] _model Name of the model to watch
  /// \param[in] _iterations Iterations to run
  /// \param[in] _between Called after _between.first iterations, before the
  /// rest run; for publishing mid-run. The pair's second is ignored when
  /// first is 0.
  /// \return The recorded velocities, one per iteration
  public: std::vector<math::Vector3d> Drift(const std::string &_world,
      const std::string &_model, unsigned int _iterations,
      std::pair<unsigned int, std::function<void()>> _between = {0, nullptr});
};

//////////////////////////////////////////////////
std::vector<math::Vector3d> HydrodynamicsCurrentsTest::Drift(
    const std::string &_world, const std::string &_model,
    unsigned int _iterations,
    std::pair<unsigned int, std::function<void()>> _between)
{
  common::Console::SetVerbosity(4);

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(_world);
  TestFixture fixture(serverConfig);

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
      auto modelEntity = world.ModelByName(_ecm, _model);
      ASSERT_NE(modelEntity, kNullEntity);
      auto bodyEntity = Model(modelEntity).LinkByName(_ecm, _model + "_link");
      ASSERT_NE(bodyEntity, kNullEntity);
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

  if (_between.first > 0)
  {
    fixture.Server()->Run(true, _between.first, false);
    _between.second();
    fixture.Server()->Run(true, _iterations - _between.first, false);
  }
  else
  {
    fixture.Server()->Run(true, _iterations, false);
  }
  EXPECT_EQ(_iterations, bodyVels.size());
  return bodyVels;
}

/////////////////////////////////////////////////
/// A current published on /ocean_current reaches a plugin with no table.
TEST_F(HydrodynamicsCurrentsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(CurrentFromTopic))
{
  auto world = common::joinPaths(std::string(PROJECT_BINARY_PATH),
      "test", "worlds", "hydrodynamics.sdf");

  transport::Node node;
  auto pub = node.Advertise<msgs::Vector3d>("/ocean_current");
  auto publish = [&]()
  {
    for (int i = 0; i < 50 && !pub.HasConnections(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(pub.HasConnections());
    EXPECT_TRUE(pub.Publish(msgs::Convert(math::Vector3d(0, 1, 0))));
    // Let the message land before stepping on.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  };

  auto vels = this->Drift(world, "sphere_topic", 1200, {200, publish});
  ASSERT_EQ(1200u, vels.size());

  // Still until the message.
  EXPECT_NEAR(vels[199].Length(), 0.0, 1e-6);
  // Then along +y.
  for (unsigned int i = 1190; i < 1200; ++i)
  {
    EXPECT_GT(vels[i].Y(), 0.3);
    EXPECT_NEAR(vels[i].X(), 0, 1e-6);
    EXPECT_NEAR(vels[i].Z(), 0, 1e-6);
  }
}

/////////////////////////////////////////////////
/// A table whose current grows with z: the higher sphere drifts faster.
TEST_F(HydrodynamicsCurrentsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(CurrentSheared))
{
  auto world = common::joinPaths(std::string(PROJECT_BINARY_PATH),
      "test", "worlds", "hydrodynamics_sheared.sdf");

  auto high = this->Drift(world, "sphere_high", 1000);
  auto low = this->Drift(world, "sphere_low", 1000);
  ASSERT_EQ(1000u, high.size());
  ASSERT_EQ(1000u, low.size());

  // The current is 0.75 m/s at z = 10 and 0.25 m/s at z = -10; after a
  // second neither sphere has caught up with it, but both move along +x
  // and the higher one faster.
  for (unsigned int i = 990; i < 1000; ++i)
  {
    EXPECT_GT(high[i].X(), 0.3);
    EXPECT_GT(low[i].X(), 0.05);
    EXPECT_GT(high[i].X(), 2 * low[i].X());
    EXPECT_NEAR(high[i].Y(), 0, 1e-6);
    EXPECT_NEAR(low[i].Y(), 0, 1e-6);
    EXPECT_NEAR(high[i].Z(), 0, 1e-6);
    EXPECT_NEAR(low[i].Z(), 0, 1e-6);
  }
}

/////////////////////////////////////////////////
/// A sphere outside the table's bounds looks up no current and stays put,
/// without a crash or a NaN.
TEST_F(HydrodynamicsCurrentsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(CurrentOutsideTable))
{
  auto world = common::joinPaths(std::string(PROJECT_BINARY_PATH),
      "test", "worlds", "hydrodynamics_sheared.sdf");

  auto vels = this->Drift(world, "sphere_outside", 300);
  ASSERT_EQ(300u, vels.size());
  for (const auto &vel : vels)
  {
    EXPECT_TRUE(vel.IsFinite());
    EXPECT_NEAR(vel.Length(), 0.0, 1e-6);
  }
}

/////////////////////////////////////////////////
/// A table that turns from +x to +y at t = 2 s turns the drift with it.
TEST_F(HydrodynamicsCurrentsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(CurrentTimeVarying))
{
  auto world = common::joinPaths(std::string(PROJECT_BINARY_PATH),
      "test", "worlds", "hydrodynamics_time.sdf");

  auto vels = this->Drift(world, "sphere_turning", 5000);
  ASSERT_EQ(5000u, vels.size());

  // Half a second in: mostly along +x.
  EXPECT_GT(vels[499].X(), 0.3);
  EXPECT_GT(vels[499].X(), vels[499].Y());
  // Five seconds in: along +y, and the x drift has decayed against the
  // water, which no longer moves that way.
  EXPECT_GT(vels[4999].Y(), 0.3);
  EXPECT_GT(vels[4999].Y(), vels[4999].X());
  EXPECT_LT(vels[4999].X(), vels[499].X());
  EXPECT_NEAR(vels[4999].Z(), 0, 1e-6);
}

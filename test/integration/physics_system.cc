/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <algorithm>
#include <vector>

#include <ignition/common/Console.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemManager.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace std::chrono_literals;
namespace components = ignition::gazebo::components;

class PhysicsSystemFixture : public ::testing::Test
{
  protected: void SetUp() override
  {
    // Augment the system plugin path.  In SetUp to avoid test order issues.
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
      (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

class Relay
{
  public: Relay()
  {
    auto plugin = sm.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();
    this->mockSystem = static_cast<gazebo::MockSystem *>(
        systemPtr->QueryInterface<gazebo::System>());
  }

  public: Relay & OnPreUpdate(gazebo::MockSystem::CallbackType cb)
  {
    this->mockSystem->preUpdateCallback = cb;
    return *this;
  }

  public: Relay & OnUpdate(gazebo::MockSystem::CallbackType cb)
  {
    this->mockSystem->updateCallback = cb;
    return *this;
  }

  public: Relay & OnPostUpdate(gazebo::MockSystem::CallbackType cb)
  {
    this->mockSystem->postUpdateCallback = cb;
    return *this;
  }

  public: ignition::gazebo::SystemPluginPtr systemPtr;

  private: gazebo::SystemManager sm;
  private: gazebo::MockSystem *mockSystem;
};


/////////////////////////////////////////////////
TEST_F(PhysicsSystemFixture, CreatePhysicsWorld)
{
  ignition::gazebo::ServerConfig serverConfig;

  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);

  server.SetUpdatePeriod(1ns);

  for (uint64_t i = 1; i < 10; ++i)
  {
    EXPECT_FALSE(server.Running());
    server.Run(true, 1, false);
    EXPECT_FALSE(server.Running());
  }
  // TODO(addisu) add useful EXPECT calls
}

/////////////////////////////////////////////////
TEST_F(PhysicsSystemFixture, FallingObject)
{
  ignition::gazebo::ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/falling.sdf";
  serverConfig.SetSdfFile(sdfFile);

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  const sdf::Model *model = world->ModelByIndex(0);

  gazebo::Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  const std::string modelName = "sphere";
  std::vector<ignition::math::Pose3d> spherePoses;

  // Create a system that records the poses of the sphere
  Relay testSystem;

  testSystem.OnPostUpdate(
    [modelName, &spherePoses](const gazebo::UpdateInfo &,
    const gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const ignition::gazebo::EntityId &, const components::Model *,
        const components::Name *_name, const components::Pose *_pose)->bool
        {
          if (_name->Data() == modelName) {
            spherePoses.push_back(_pose->Data());
          }
          return true;
        });
    });

  server.AddSystem(testSystem.systemPtr);
  const size_t iters = 10;
  server.Run(true, iters, false);

  // TODO(addisu): Get dt from simulation
  const double dt = 0.001;
  const double grav = world->Gravity().Z();
  const double zInit = model->Pose().Pos().Z();
  // The sphere should have fallen for (iters * dt) seconds.
  const double zExpected = zInit + 0.5 * grav * pow(iters * dt, 2);
  EXPECT_NEAR(spherePoses.back().Pos().Z(), zExpected, 2e-4);

  // run for 2 more seconds and check to see if the sphere has stopped
  server.Run(true, 2000, false);

  // The sphere should land on the box and stop.
  auto geometry = model->LinkByIndex(0)->CollisionByIndex(0)->Geom();
  auto sphere = geometry->SphereShape();
  ASSERT_TRUE(sphere != nullptr);

  // The box surface is at 0 so the z position of the sphere is the same as its
  // radius. The position of the model will be offset by the first links pose
  const double zStopped =
      sphere->Radius() - model->LinkByIndex(0)->Pose().Pos().Z();
  EXPECT_NEAR(spherePoses.back().Pos().Z(), zStopped, 5e-2);
}

/////////////////////////////////////////////////
// This tests whether links with fixed joints keep their relative transforms
// after physics. For that to work properly, the canonical link implementation
// must be correct.
TEST_F(PhysicsSystemFixture, CanonicalLink)
{
  ignition::gazebo::ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/canonical.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  gazebo::Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  const std::string modelName{"canonical"};
  std::vector<std::string> linksToCheck{"base_link", "link1", "link2"};

  const sdf::Model *model = world->ModelByIndex(0);

  std::unordered_map<std::string, ignition::math::Pose3d> expectedLinPoses;
  for (auto &linkName : linksToCheck)
    expectedLinPoses[linkName] = model->LinkByName(linkName)->Pose();
  ASSERT_EQ(3u, expectedLinPoses.size());

  // Create a system that records the poses of the links after physics
  Relay testSystem;

  std::unordered_map<std::string, ignition::math::Pose3d> postUpLinkPoses;
  testSystem.OnPostUpdate(
    [&modelName, &postUpLinkPoses](const gazebo::UpdateInfo &,
    const gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Link, components::Name, components::Pose,
                components::ParentEntity>(
        [&](const ignition::gazebo::EntityId &, const components::Link *,
        const components::Name *_name, const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
        {
          auto parentModel = _ecm.Component<components::Name>(_parent->Data());
          EXPECT_TRUE(nullptr != parentModel);
          if (parentModel->Data() == modelName)
          {
            postUpLinkPoses[_name->Data()] = _pose->Data();
          }
          return true;
        });
      return true;
    });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  EXPECT_EQ(3u, postUpLinkPoses.size());

  for (auto &link : linksToCheck)
  {
    ASSERT_TRUE(postUpLinkPoses.find(link) != postUpLinkPoses.end())
        << link << " not found";
    // We expect that after physics iterations, the relative poses of the links
    // to be the same as their initial relative poses since all the joints are
    // fixed joints
    EXPECT_EQ(expectedLinPoses[link], postUpLinkPoses[link]) << link;
  }
}

/////////////////////////////////////////////////
// Test physics integration with revolute joints
TEST_F(PhysicsSystemFixture, RevoluteJoint)
{
  ignition::gazebo::ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/revolute_joint.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  gazebo::Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  const std::string modelName{"revolute_demo"};
  const std::string rotatingLinkName{"link2"};

  Relay testSystem;

  std::vector<double> armDistances;

  // The test checks if the link connected to the joint swings from side to
  // side. To do this, we check that the maximum and minimum distances of the
  // swinging arm from it's parent model frame. The maximum distance is when the
  // arm is in its initial position. The minimum distance is when the arm is in
  // line with the support arm.
  testSystem.OnPostUpdate(
    [&rotatingLinkName, &armDistances](const gazebo::UpdateInfo &,
    const gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Link, components::Name, components::Pose>(
        [&](const ignition::gazebo::EntityId &, const components::Link *,
        const components::Name *_name, const components::Pose *_pose)->bool
        {
          if (rotatingLinkName == _name->Data())
          {
            auto pos = _pose->Data().Pos();
            // we ignore the x axis to simplify distance comparisons
            pos.X() = 0;
            armDistances.push_back(pos.Length());
          }
          return true;
        });
    });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1000, false);

  ASSERT_GT(armDistances.size(), 0u);

  const sdf::Model *model = world->ModelByIndex(1);

  auto getCylinderLength = [&model](const std::string &linkName)
  {
    auto collision = model->LinkByName(linkName)->CollisionByIndex(0);
    auto cylinder = collision->Geom()->CylinderShape();
    return cylinder->Length();
  };

  const double link1Length = getCylinderLength("link1");
  const double link2Length = getCylinderLength("link2");
  // divide by 2.0 because the position is the center of the link.
  const double expMinDist = link1Length - link2Length/2.0;
  auto link2InitialPos = model->LinkByName("link2")->Pose().Pos();
  // we ignore the x axis to simplify distance comparisons
  link2InitialPos.X() = 0;
  const double expMaxDist = link2InitialPos.Length();

  auto minmax = std::minmax_element(armDistances.begin(), armDistances.end());

  EXPECT_NEAR(expMinDist, *minmax.first, 1e-3);
  EXPECT_NEAR(expMaxDist, *minmax.second, 1e-3);
}

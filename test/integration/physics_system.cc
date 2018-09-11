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
#include <ignition/common/Console.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>

#include "ignition/gazebo/Server.hh"
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
using namespace ignition;
using namespace std::chrono_literals;
namespace components = ignition::gazebo::components;

class PhysicsSystemFixture : public ::testing::Test
{
  protected:
    void SetUp() override
    {
      // Augment the system plugin path.  In SetUp to avoid test order issues.
      setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
             (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
    }
};

// TODO(addisu) Create a Relay class that can be used by other tests.
class MockSystem : public gazebo::System
{
  public:
  using CallbackType = std::function<void(const gazebo::UpdateInfo &,
                                     const gazebo::EntityComponentManager &)>;

  public: void Init() override;
  public: void PostUpdate(const gazebo::UpdateInfo &_info,
                const gazebo::EntityComponentManager &_ecm) override;
  public: void RegisterCallback(const CallbackType &cb);

  public: CallbackType updateCb;
};

void MockSystem::Init()
{
}

void MockSystem::RegisterCallback(const CallbackType &cb)
{
  updateCb = cb;
}

void MockSystem::PostUpdate(const gazebo::UpdateInfo &_info,
                            const gazebo::EntityComponentManager &_ecm)
{
  updateCb(_info, _ecm);
}

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
    EXPECT_FALSE(*server.Running());
    server.Run(true, 1);
    EXPECT_FALSE(*server.Running());
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

  server.SetUpdatePeriod(1ns);

  const std::string linkName = "sphere_link";
  std::vector<ignition::math::Pose3d> spherePoses;

  // Create a system that records the poses of the sphere
  auto testSystem = std::make_shared<MockSystem>();
  testSystem->RegisterCallback(
      [linkName, &spherePoses](const gazebo::UpdateInfo &,
                               const gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Link, components::Name, components::Pose>(
            [&](const ignition::gazebo::EntityId &, const components::Link *,
                const components::Name *_name, const components::Pose *_pose)
            {
              if (_name->Data() == linkName)
              {
                spherePoses.push_back(_pose->Data());
              }
            });
      });

  server.AddSystem(testSystem);
  const size_t iters = 10;
  server.Run(true, iters);

  // TODO(addisu): Get dt from simulation
  const double dt = 0.001;
  // TODO(addisu): Using default grav in DART
  const double grav = 9.81;
  const double zInit = model->Pose().Pos().Z();
  // The sphere should have fallen for 10ms.
  const double zExpected = zInit - 0.5 * grav * pow(iters * dt, 2);
  // The tolerance is not very tight due to integration errors with a step size
  // of 0.001
  EXPECT_NEAR(spherePoses.back().Pos().Z(), zExpected, 1e-4);
}

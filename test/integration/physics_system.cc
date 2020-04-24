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
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointPositionReset.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityReset.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

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

  public: Relay & OnPreUpdate(gazebo::MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay & OnUpdate(gazebo::MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay & OnPostUpdate(gazebo::MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: ignition::gazebo::SystemPluginPtr systemPtr;

  private: gazebo::SystemLoader sm;
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
        [&](const ignition::gazebo::Entity &, const components::Model *,
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
  const double zInit = model->RawPose().Pos().Z();
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
      sphere->Radius() - model->LinkByIndex(0)->RawPose().Pos().Z();
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
    expectedLinPoses[linkName] = model->LinkByName(linkName)->RawPose();
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
        [&](const ignition::gazebo::Entity &, const components::Link *,
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
// Same as the CanonicalLink test, but with a non-default canonical link
TEST_F(PhysicsSystemFixture, NonDefaultCanonicalLink)
{
  ignition::gazebo::ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/nondefault_canonical.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  gazebo::Server server(serverConfig);

  server.SetUpdatePeriod(1ns);

  const std::string modelName{"nondefault_canonical"};

  // Create a system that records the pose of the model.
  Relay testSystem;

  std::vector<ignition::math::Pose3d> modelPoses;
  testSystem.OnPostUpdate(
    [&modelName, &modelPoses](const gazebo::UpdateInfo &,
    const gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const ignition::gazebo::Entity &, const components::Model *,
        const components::Name *_name, const components::Pose *_pose)->bool
        {
          if (_name->Data() == modelName)
          {
            modelPoses.push_back(_pose->Data());
          }
          return true;
        });
      return true;
    });

  server.AddSystem(testSystem.systemPtr);
  std::size_t nIters{2000};
  server.Run(true, nIters, false);

  EXPECT_EQ(nIters, modelPoses.size());

  // The model is attached to link2 (it's canonical link) so it will fall during
  // simulation. The new Z position of the model is an offset of -2 in the Z
  // direction from the center of link2.
  EXPECT_NEAR(-(2.0 - 0.2), modelPoses.back().Pos().Z(), 1e-2);
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
        [&](const ignition::gazebo::Entity &, const components::Link *,
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

  auto getCylinderLength = [&model](const std::string &_linkName)
  {
    auto collision = model->LinkByName(_linkName)->CollisionByIndex(0);
    auto cylinder = collision->Geom()->CylinderShape();
    return cylinder->Length();
  };

  const double link1Length = getCylinderLength("link1");
  const double link2Length = getCylinderLength("link2");
  // divide by 2.0 because the position is the center of the link.
  const double expMinDist = link1Length - link2Length/2.0;
  auto link2InitialPos = model->LinkByName("link2")->RawPose().Pos();
  // we ignore the x axis to simplify distance comparisons
  link2InitialPos.X() = 0;
  const double expMaxDist = link2InitialPos.Length();

  auto minmax = std::minmax_element(armDistances.begin(), armDistances.end());

  EXPECT_NEAR(expMinDist, *minmax.first, 1e-3);
  EXPECT_NEAR(expMaxDist, *minmax.second, 1e-3);
}

/////////////////////////////////////////////////
TEST_F(PhysicsSystemFixture, CreateRuntime)
{
  ignition::gazebo::ServerConfig serverConfig;
  gazebo::Server server(serverConfig);
  server.SetPaused(false);

  // Create a system just to get the ECM
  // TODO(louise) It would be much more convenient if the Server just returned
  // the ECM for us. This would save all the trouble which is causing us to
  // create `Relay` systems in the first place. Consider keeping the ECM in a
  // shared pointer owned by the SimulationRunner.
  EntityComponentManager *ecm{nullptr};
  Relay testSystem;
  testSystem.OnPreUpdate([&](const gazebo::UpdateInfo &,
                             gazebo::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });
  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  // Check we don't have a new model yet
  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("new_model")));

  // Get world
  auto worldEntity = ecm->EntityByComponents(components::World());
  EXPECT_NE(kNullEntity, worldEntity);

  // Spawn a new model
  auto modelEntity = ecm->CreateEntity();
  ecm->CreateComponent(modelEntity, components::Model());
  ecm->CreateComponent(modelEntity, components::Pose(math::Pose3d::Zero));
  ecm->CreateComponent(modelEntity, components::Name("new_model"));
  ecm->CreateComponent(modelEntity, components::Static(false));
  ecm->SetParentEntity(modelEntity, worldEntity);
  ecm->CreateComponent(modelEntity, components::ParentEntity(worldEntity));

  auto linkEntity = ecm->CreateEntity();
  ecm->CreateComponent(linkEntity, components::Link());
  ecm->CreateComponent(linkEntity, components::CanonicalLink());
  ecm->CreateComponent(linkEntity, components::Pose(math::Pose3d::Zero));
  ecm->CreateComponent(linkEntity, components::Name("link"));

  math::MassMatrix3d massMatrix;
  massMatrix.SetMass(1.0);
  massMatrix.SetInertiaMatrix(0.4, 0.4, 0.4, 0, 0, 0);
  math::Inertiald inertia;
  inertia.SetMassMatrix(massMatrix);
  ecm->CreateComponent(linkEntity, components::Inertial(inertia));

  ecm->SetParentEntity(linkEntity, modelEntity);
  ecm->CreateComponent(linkEntity, components::ParentEntity(modelEntity));

  // Check we have a new model
  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("new_model")));

  // Run server and check new model keeps falling due to gravity
  auto poseComp = ecm->Component<components::Pose>(modelEntity);
  ASSERT_NE(nullptr, poseComp);

  auto pose = poseComp->Data();
  EXPECT_EQ(math::Pose3d::Zero, pose);

  for (int i = 0; i < 10; ++i)
  {
    server.Run(true, 100, false);

    poseComp = ecm->Component<components::Pose>(modelEntity);
    ASSERT_NE(nullptr, poseComp);

    EXPECT_GT(pose.Pos().Z(), poseComp->Data().Pos().Z());
    pose = poseComp->Data();
  }
}

/////////////////////////////////////////////////
TEST_F(PhysicsSystemFixture, SetFrictionCoefficient)
{
  ignition::gazebo::ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/friction.sdf";
  serverConfig.SetSdfFile(sdfFile);

  gazebo::Server server(serverConfig);

  server.SetUpdatePeriod(1ns);

  std::map<std::string, double> boxParams{
      {"box1", 0.01}, {"box2", 0.1}, {"box3", 1.0}};
  std::map<std::string, std::vector<ignition::math::Pose3d>> poses;

  // Create a system that records the poses of the 3 boxes
  Relay testSystem;
  double dt = 0.0;

  testSystem.OnPostUpdate(
    [&boxParams, &poses, &dt](const gazebo::UpdateInfo &_info,
    const gazebo::EntityComponentManager &_ecm)
    {
      dt = _info.dt.count();
      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const ignition::gazebo::Entity &, const components::Model *,
        const components::Name *_name, const components::Pose *_pose)->bool
        {
          if (boxParams.find(_name->Data()) != boxParams.end()) {
            poses[_name->Data()].push_back(_pose->Data());
          }
          return true;
        });
    });

  server.AddSystem(testSystem.systemPtr);
  const size_t iters = 2000;
  server.Run(true, iters, false);

  using PairType = std::pair<double, double>;
  std::vector<PairType> yPosCoeffPairs;
  for (const auto& [name, coeff] : boxParams)
  {
    EXPECT_EQ(iters, poses[name].size());
    // Check that the box with the smallest friction coefficient has travelled
    // the most in the y direction. To do so, put the y values of the last poses
    // in a sorted list paired up with the friction coefficients and check that
    // the list is in an order such that the highest y value corresponds to the
    // lower coefficient
    yPosCoeffPairs.emplace_back(poses[name].back().Pos().Y(), coeff);
  }

  EXPECT_EQ(boxParams.size(), yPosCoeffPairs.size());
  // Sort by coefficient first in a descending order
  std::sort(yPosCoeffPairs.begin(), yPosCoeffPairs.end(),
            [](const PairType &_a, const PairType &_b)
            {
              return _a.second > _b.second;
            });

  // Check if the Y position is strictly increasing
  bool isIncreasingYPos =
      std::is_sorted(yPosCoeffPairs.begin(), yPosCoeffPairs.end(),
                     [](const PairType &_a, const PairType &_b)
                     {
                       // std::is_sorted works by iterating through the
                       // container until comp(_a, _b) is true, where comp is
                       // this function. If comp returns true before reaching
                       // the end of the container, the container is not sorted.
                       // In a strictly ascending order, _a should always be
                       // greater than _b so this comparison should always
                       // return false. The last term is added to avoid floating
                       // point tolerance issues.
                       return _a.first <= _b.first + 1e-4;
                     });

  std::ostringstream oss;
  for (const auto &[pos, coeff] : yPosCoeffPairs)
  {
    oss << "(" << pos << ", " << coeff << "), ";
  }
  EXPECT_TRUE(isIncreasingYPos) << oss.str();
}

/////////////////////////////////////////////////
/// Test that joint position reported by the physics system include all axes
TEST_F(PhysicsSystemFixture, MultiAxisJointPosition)
{
  ignition::gazebo::ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/demo_joint_types.sdf";
  serverConfig.SetSdfFile(sdfFile);

  gazebo::Server server(serverConfig);

  server.SetUpdatePeriod(0ns);

  Relay testSystem;
  // Create JointPosition components if they don't already exist
  testSystem.OnPreUpdate(
      [&](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint>(
            [&](const ignition::gazebo::Entity &_entity,
                components::Joint *) -> bool
            {
              auto posComp = _ecm.Component<components::JointPosition>(_entity);
              if (!posComp)
              {
                _ecm.CreateComponent(_entity, components::JointPosition());
              }
              return true;
            });
      });

  std::map<std::string, std::size_t> jointPosDof;

  testSystem.OnPostUpdate(
    [&](const gazebo::UpdateInfo &, const gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint, components::Name, components::JointPosition>(
        [&](const ignition::gazebo::Entity &,
            const components::Joint *,
            const components::Name *_name,
            const components::JointPosition *_jointPos) -> bool
        {
          jointPosDof[_name->Data()] = _jointPos->Data().size();
          return true;
        });
    });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  std::map<std::string, std::size_t> jointsToCheck {
    {"revolute_demo", 1},
    {"prismatic_demo", 1},
    {"ball_demo", 3},
    {"screw_demo", 1},
    {"universal_demo", 2},
    {"fixed_demo", 0},
    // Gearbox not supported yet
    // {"gearbox_demo", 1},
    // Revolute2 not supported yet
    // {"revolute2_demo", 2},
  };

  for (const auto &[jName, jDof] : jointsToCheck)
  {
    ASSERT_TRUE(jointPosDof.find(jName) != jointPosDof.end()) << jName;
    EXPECT_EQ(jDof, jointPosDof[jName]) << jName;
  }

  // Run several more times and ensure that the DOF stays the same
  server.Run(true, 10, false);
  for (const auto &[jName, jDof] : jointsToCheck)
  {
    ASSERT_TRUE(jointPosDof.find(jName) != jointPosDof.end()) << jName;
    EXPECT_EQ(jDof, jointPosDof[jName]) << jName;
  }
}

/////////////////////////////////////////////////
/// Test joint position reset component
TEST_F(PhysicsSystemFixture, ResetPositionComponent)
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

  server.SetUpdatePeriod(1ms);

  const std::string rotatingJointName{"j2"};

  Relay testSystem;

  double pos0 = 0.42;

  // cppcheck-suppress variableScope
  bool firstRun = true;

  testSystem.OnPreUpdate(
    [&](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint, components::Name>(
        [&](const ignition::gazebo::Entity &_entity,
            const components::Joint *, components::Name *_name) -> bool
      {
        if (_name->Data() == rotatingJointName)
        {
          auto resetComp =
              _ecm.Component<components::JointPositionReset>(_entity);
          auto position = _ecm.Component<components::JointPosition>(_entity);

          if (firstRun)
          {
            firstRun = false;

            EXPECT_EQ(nullptr, resetComp);
            _ecm.CreateComponent(_entity,
                                 components::JointPositionReset({pos0}));

            EXPECT_EQ(nullptr, position);
            _ecm.CreateComponent(_entity, components::JointPosition());
          }
          else
          {
              EXPECT_EQ(nullptr, resetComp);
              EXPECT_NE(nullptr, position);
          }
        }
        return true;
      });
    });

  std::vector<double> positions;

  testSystem.OnPostUpdate([&](
    const gazebo::UpdateInfo &, const gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint,
                components::Name, components::JointPosition>(
          [&](const ignition::gazebo::Entity &,
              const components::Joint *,
              const components::Name *_name,
              const components::JointPosition *_pos)
          {
            if (_name->Data() == rotatingJointName)
            {
              positions.push_back(_pos->Data()[0]);
            }
            return true;
          });
    });

    server.AddSystem(testSystem.systemPtr);
    server.Run(true, 2, false);

    ASSERT_EQ(positions.size(), 2ul);

    // First position should be exactly the same
    EXPECT_DOUBLE_EQ(pos0, positions[0]);

    // Second position should be different, but close
    EXPECT_NEAR(pos0, positions[1], 0.01);
}

/////////////////////////////////////////////////
/// Test joint veocity reset component
TEST_F(PhysicsSystemFixture, ResetVelocityComponent)
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

  server.SetUpdatePeriod(1ms);

  const std::string rotatingJointName{"j2"};

  Relay testSystem;

  double vel0 = 3.0;

  // cppcheck-suppress variableScope
  bool firstRun = true;

  testSystem.OnPreUpdate(
    [&](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint, components::Name>(
        [&](const ignition::gazebo::Entity &_entity,
            const components::Joint *, components::Name *_name) -> bool
        {
          if (_name->Data() == rotatingJointName)
          {
            auto resetComp =
                _ecm.Component<components::JointVelocityReset>(_entity);
            auto velocity = _ecm.Component<components::JointVelocity>(_entity);

            if (firstRun)
            {
              firstRun = false;

              EXPECT_EQ(nullptr, resetComp);
              _ecm.CreateComponent(_entity,
                                   components::JointVelocityReset({vel0}));

              EXPECT_EQ(nullptr, velocity);
              _ecm.CreateComponent(_entity, components::JointVelocity());
            }
            else
            {
              EXPECT_EQ(nullptr, resetComp);
              EXPECT_NE(nullptr, velocity);
            }
          }
          return true;
        });
    });

  std::vector<double> velocities;

  testSystem.OnPostUpdate([&](
    const gazebo::UpdateInfo &, const gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint,
                components::Name,
                components::JointVelocity>(
        [&](const ignition::gazebo::Entity &,
            const components::Joint *,
            const components::Name *_name,
            const components::JointVelocity *_vel)
        {
          if (_name->Data() == rotatingJointName)
          {
            velocities.push_back(_vel->Data()[0]);
          }
          return true;
        });
    });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 2, false);

  ASSERT_EQ(velocities.size(), 2ul);

  // First velocity should be exactly the same
  // TODO(anyone): we should use EXPECT_EQ but for some reason the
  //               resulting velocity is 2.9999 instead of 3.0
  EXPECT_NEAR(vel0, velocities[0], 2e-4);

  // Second velocity should be different, but close
  EXPECT_NEAR(vel0, velocities[1], 0.05);
}

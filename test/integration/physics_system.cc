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
#include <string>
#include <vector>

#ifdef HAVE_DART
#include <dart/config.hpp>
#endif
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/utils/ExtraTestMacros.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/World.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/Util.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AxisAlignedBox.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointEffortLimitsCmd.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointTransmittedWrench.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionLimitsCmd.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointVelocityLimitsCmd.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Material.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

class PhysicsSystemFixture : public InternalFixture<::testing::Test>
{
};

class PhysicsSystemFixtureWithDart6_10 : public PhysicsSystemFixture
{
  protected: void SetUp() override
  {
#ifndef HAVE_DART
    GTEST_SKIP();
#elif !DART_VERSION_AT_LEAST(6, 10, 0)
    GTEST_SKIP();
#endif

    PhysicsSystemFixture::SetUp();
  }
};

/////////////////////////////////////////////////
TEST_F(PhysicsSystemFixture, CreatePhysicsWorld)
{
  ServerConfig serverConfig;

  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/shapes.sdf");

  Server server(serverConfig);

  server.SetUpdatePeriod(1ns);

  for (uint64_t i = 1; i < 10; ++i)
  {
    EXPECT_FALSE(server.Running());
    server.Run(true, 1, false);
    EXPECT_FALSE(server.Running());
  }
}

////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(PhysicsSystemFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(FallingObject))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/falling.sdf";
  serverConfig.SetSdfFile(sdfFile);

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  const sdf::Model *model = world->ModelByIndex(0);

  Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  const std::string modelName = "sphere";
  std::vector<math::Pose3d> spherePoses;

  // Create a system that records the poses of the sphere
  test::Relay testSystem;

  testSystem.OnPostUpdate(
    [modelName, &spherePoses](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const Entity &, const components::Model *,
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
TEST_F(PhysicsSystemFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(CanonicalLink))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/canonical.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  const std::string modelName{"canonical"};
  std::vector<std::string> linksToCheck{"base_link", "link1", "link2"};

  const sdf::Model *model = world->ModelByIndex(0);

  std::unordered_map<std::string, math::Pose3d> expectedLinPoses;
  for (auto &linkName : linksToCheck)
    expectedLinPoses[linkName] = model->LinkByName(linkName)->RawPose();
  ASSERT_EQ(3u, expectedLinPoses.size());

  // Create a system that records the poses of the links after physics
  test::Relay testSystem;

  std::unordered_map<std::string, math::Pose3d> postUpLinkPoses;
  testSystem.OnPostUpdate(
    [&modelName, &postUpLinkPoses](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Link, components::Name, components::Pose,
                components::ParentEntity>(
        [&](const Entity &, const components::Link *,
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
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(NonDefaultCanonicalLink))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/nondefault_canonical.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1ns);

  const std::string modelName{"nondefault_canonical"};

  // Create a system that records the pose of the model.
  test::Relay testSystem;

  std::vector<math::Pose3d> modelPoses;
  testSystem.OnPostUpdate(
    [&modelName, &modelPoses](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const Entity &, const components::Model *,
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
TEST_F(PhysicsSystemFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(RevoluteJoint))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/revolute_joint.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  const std::string modelName{"revolute_demo"};
  const std::string rotatingLinkName{"link2"};

  test::Relay testSystem;

  std::vector<double> armDistances;

  // The test checks if the link connected to the joint swings from side to
  // side. To do this, we check that the maximum and minimum distances of the
  // swinging arm from it's parent model frame. The maximum distance is when the
  // arm is in its initial position. The minimum distance is when the arm is in
  // line with the support arm.
  testSystem.OnPostUpdate(
    [&rotatingLinkName, &armDistances](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Link, components::Name, components::Pose>(
        [&](const Entity &, const components::Link *,
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
TEST_F(PhysicsSystemFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(CreateRuntime))
{
  ServerConfig serverConfig;
  Server server(serverConfig);
  server.SetPaused(false);

  // Create a system just to get the ECM
  // TODO(louise) It would be much more convenient if the Server just returned
  // the ECM for us. This would save all the trouble which is causing us to
  // create `Relay` systems in the first place. Consider keeping the ECM in a
  // shared pointer owned by the SimulationRunner.
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const UpdateInfo &,
                             EntityComponentManager &_ecm)
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
  ecm->CreateComponent(modelEntity, components::ModelCanonicalLink(linkEntity));

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
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(SetFrictionCoefficient))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/friction.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1ns);

  std::map<std::string, double> boxParams{
      {"box1", 0.01}, {"box2", 0.1}, {"box3", 1.0}};
  std::map<std::string, std::vector<math::Pose3d>> poses;

  // Create a system that records the poses of the 3 boxes
  test::Relay testSystem;

  testSystem.OnPostUpdate(
    [&boxParams, &poses](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const Entity &, const components::Model *,
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
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(MultiAxisJointPosition))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/demo_joint_types.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(0ns);

  test::Relay testSystem;
  // Create JointPosition components if they don't already exist
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint>(
            [&](const Entity &_entity,
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
    [&](const UpdateInfo &, const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint, components::Name, components::JointPosition>(
        [&](const Entity &,
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
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetPositionComponent))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/revolute_joint.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1ms);

  const std::string rotatingJointName{"j2"};

  test::Relay testSystem;

  double pos0 = 0.42;

  // cppcheck-suppress variableScope
  bool firstRun = true;

  testSystem.OnPreUpdate(
    [&](const UpdateInfo &, EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint, components::Name>(
        [&](const Entity &_entity,
            const components::Joint *, components::Name *_name) -> bool
      {
        EXPECT_NE(nullptr, _name);

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
    const UpdateInfo &, const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint,
                components::Name, components::JointPosition>(
          [&](const Entity &,
              const components::Joint *,
              const components::Name *_name,
              const components::JointPosition *_pos)
          {
            EXPECT_NE(nullptr, _name);

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
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetVelocityComponent))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/revolute_joint.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1ms);

  const std::string rotatingJointName{"j2"};

  test::Relay testSystem;

  double vel0 = 3.0;

  // cppcheck-suppress variableScope
  bool firstRun = true;

  testSystem.OnPreUpdate(
    [&](const UpdateInfo &, EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint, components::Name>(
        [&](const Entity &_entity,
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
    const UpdateInfo &, const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint,
                components::Name,
                components::JointVelocity>(
        [&](const Entity &,
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

/////////////////////////////////////////////////
/// Test joint position limit command component
TEST_F(PhysicsSystemFixtureWithDart6_10, JointPositionLimitsCommandComponent)
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/revolute_joint.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1ms);

  const std::string rotatingJointName{"j2"};

  test::Relay testSystem;

  // cppcheck-suppress variableScope
  size_t iteration = 0u;

  // The system is not in equilibrium at the beginning, so normally, joint j2
  // would move. For the first 50 ms, we set position limits to 1e-6 so that
  // it can't freely move, and we check it did not. For the other 50 ms, we
  // remove the position limit and check that the joint has moved at least by
  // 1e-2. Between times 30 and 40 ms we also add a 100 N force to the joint to
  // check that the limit is held even in presence of force commands. Between
  // times 40 and 50 ms, we add a velocity command to check that velocity
  // commands do not break the positional limit.

  testSystem.OnPreUpdate(
    [&](const UpdateInfo &, EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint, components::Name>(
        [&](const Entity &_entity,
            const components::Joint *, components::Name *_name) -> bool
        {
          if (_name->Data() == rotatingJointName)
          {
            if (iteration == 0u)
            {
              auto limitComp =
                _ecm.Component<components::JointPositionLimitsCmd>(_entity);
              EXPECT_EQ(nullptr, limitComp);
              _ecm.CreateComponent(_entity,
                components::JointPositionLimitsCmd ({{-1e-6, 1e-6}}));
              _ecm.CreateComponent(_entity, components::JointPosition());
            }
            else if (iteration == 50u)
            {
              auto limitComp =
                _ecm.Component<components::JointPositionLimitsCmd>(_entity);
              EXPECT_NE(nullptr, limitComp);
              if (limitComp)
              {
                limitComp->Data() = {{-1e6, 1e6}};
              }
            }
            else
            {
              auto limitComp =
                _ecm.Component<components::JointPositionLimitsCmd>(_entity);
              EXPECT_NE(nullptr, limitComp);
              if (limitComp)
              {
                EXPECT_EQ(0u, limitComp->Data().size());
              }
              if (iteration >= 30u && iteration < 40u)
              {
                _ecm.SetComponentData<components::JointForceCmd>(
                  _entity, {100.0});
              }
              else if (iteration >= 40u && iteration < 50u)
              {
                _ecm.SetComponentData<components::JointVelocityCmd>(
                  _entity, {1.0});
              }
            }
            ++iteration;
          }
          return true;
        });
    });

  std::vector<double> positions;

  testSystem.OnPostUpdate([&](
    const UpdateInfo &, const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint,
                components::Name,
                components::JointPosition>(
        [&](const Entity &,
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
  server.Run(true, 100, false);

  ASSERT_EQ(positions.size(), 100ul);
  // The 1e-6 limit is slightly overcome, but not very much
  EXPECT_NEAR(positions[0], positions[20], 2e-5);
  EXPECT_NEAR(positions[0], positions[30], 3e-5);
  EXPECT_NEAR(positions[0], positions[40], 3e-5);
  EXPECT_NEAR(positions[0], positions[49], 3e-5);
  EXPECT_LT(std::abs(positions[50]) + 1e-2, std::abs(positions[99]));
}

/////////////////////////////////////////////////
/// Test joint velocity limit command component
TEST_F(PhysicsSystemFixtureWithDart6_10, JointVelocityLimitsCommandComponent)
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/revolute_joint.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1ms);

  const std::string rotatingJointName{"j2"};

  test::Relay testSystem;

  // cppcheck-suppress variableScope
  size_t iteration = 0u;

  // The system is not in equilibrium at the beginning, so normally, joint j2
  // would move. For the first 50 ms, we set velocity limits to 0.1 so that
  // it can't move very fast, and we check it does not. For the other 50 ms, we
  // remove the velocity limit and check that the joint has moved faster.
  // Between times 30 and 40 ms we also add a 100 N force to the joint to
  // check that the limit is held even in presence of force commands. Between
  // times 40 and 50 ms, we add a velocity command to check that velocity
  // commands do not break the velocity limit.

  testSystem.OnPreUpdate(
    [&](const UpdateInfo &, EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint, components::Name>(
        [&](const Entity &_entity,
            const components::Joint *, components::Name *_name) -> bool
        {
          if (_name->Data() == rotatingJointName)
          {
            if (iteration == 0u)
            {
              auto limitComp =
                _ecm.Component<components::JointVelocityLimitsCmd>(_entity);
              EXPECT_EQ(nullptr, limitComp);
              _ecm.CreateComponent(_entity,
                components::JointVelocityLimitsCmd ({{-0.1, 0.1}}));
              _ecm.CreateComponent(_entity, components::JointVelocity());
            }
            else if (iteration == 50u)
            {
              auto limitComp =
                _ecm.Component<components::JointVelocityLimitsCmd>(_entity);
              EXPECT_NE(nullptr, limitComp);
              if (limitComp)
              {
                limitComp->Data() = {{-1e6, 1e6}};
              }
            }
            else
            {
              auto limitComp =
                _ecm.Component<components::JointVelocityLimitsCmd>(_entity);
              EXPECT_NE(nullptr, limitComp);
              if (limitComp)
              {
                EXPECT_EQ(0u, limitComp->Data().size());
              }
              if (iteration >= 30u && iteration < 40u)
              {
                _ecm.SetComponentData<components::JointForceCmd>(
                  _entity, {100.0});
              }
              else if (iteration >= 40u && iteration < 50u)
              {
                _ecm.SetComponentData<components::JointVelocityCmd>(
                  _entity, {1.0});
              }
            }
            ++iteration;
          }
          return true;
        });
    });

  std::vector<double> velocities;

  testSystem.OnPostUpdate([&](
    const UpdateInfo &, const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint,
                components::Name,
                components::JointVelocity>(
        [&](const Entity &,
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
  server.Run(true, 100, false);

  ASSERT_EQ(velocities.size(), 100ul);
  // The 0.1 limit is slightly overcome, but not very much
  EXPECT_NEAR(0.1, velocities[20], 1e-2);
  EXPECT_NEAR(0.1, velocities[30], 1e-2);
  EXPECT_NEAR(0.1, velocities[40], 1e-2);
  EXPECT_NEAR(0.1, velocities[49], 1e-2);
  EXPECT_LT(0.5, std::abs(velocities[99]));
}


/////////////////////////////////////////////////
/// Test joint effort limit command component
TEST_F(PhysicsSystemFixtureWithDart6_10, JointEffortLimitsCommandComponent)
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/revolute_joint_equilibrium.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1ms);

  const std::string rotatingJointName{"j2"};

  test::Relay testSystem;

  // cppcheck-suppress variableScope
  size_t iteration = 0u;

  // The system is in equilibrium at the beginning.
  // For the first 50 ms, we set effort limits to 1e-6 so that
  // it can't move, and we check it does not. For the other 50 ms, we
  // remove the effort limit and check that the joint has moved.
  // Between times 30 and 40 ms we also add a 100 N force to the joint to
  // check that the limit is held even in presence of force commands. Between
  // times 40 and 50 ms, we add a velocity command to check that velocity
  // commands do not break the effort limit.

  testSystem.OnPreUpdate(
    [&](const UpdateInfo &, EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Joint, components::Name>(
        [&](const Entity &_entity,
            const components::Joint *, components::Name *_name) -> bool
        {
          if (_name->Data() == rotatingJointName)
          {
            if (iteration == 0u)
            {
              auto limitComp =
                _ecm.Component<components::JointEffortLimitsCmd>(_entity);
              EXPECT_EQ(nullptr, limitComp);
              _ecm.CreateComponent(_entity,
                components::JointEffortLimitsCmd ({{-1e-6, 1e-6}}));
              _ecm.CreateComponent(_entity, components::JointPosition());
            }
            else if (iteration == 50u)
            {
              auto limitComp =
                _ecm.Component<components::JointEffortLimitsCmd>(_entity);
              EXPECT_NE(nullptr, limitComp);
              if (limitComp)
              {
                limitComp->Data() = {{-1e9, 1e9}};
              }
            }
            else
            {
              auto limitComp =
                _ecm.Component<components::JointEffortLimitsCmd>(_entity);
              EXPECT_NE(nullptr, limitComp);
              if (limitComp)
              {
                EXPECT_EQ(0u, limitComp->Data().size());
              }
              if (iteration >= 30u && iteration < 40u)
              {
                _ecm.SetComponentData<components::JointForceCmd>(
                  _entity, {100.0});
              }
              else if (iteration >= 40u && iteration < 50u)
              {
                _ecm.SetComponentData<components::JointVelocityCmd>(
                  _entity, {1.0});
              }
              else if (iteration >= 50u)
              {
                _ecm.Component<components::JointForceCmd>(_entity)->Data() =
                  {1000.0};
              }
            }
            ++iteration;
          }
          return true;
        });
    });

  std::vector<double> positions;

  testSystem.OnPostUpdate([&](
    const UpdateInfo &, const EntityComponentManager &_ecm)
    {
    _ecm.Each<components::Joint,
    components::Name,
    components::JointPosition>(
      [&](const Entity &,
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
  server.Run(true, 100, false);

  ASSERT_EQ(positions.size(), 100ul);
  // The 1e-6 limit is slightly overcome, but not very much
  EXPECT_NEAR(positions[0], positions[20], 2e-5);
  EXPECT_NEAR(positions[0], positions[30], 3e-5);
  EXPECT_NEAR(positions[0], positions[40], 3e-5);
  EXPECT_NEAR(positions[0], positions[49], 3e-5);
  EXPECT_LT(std::abs(positions[50]) + 1e-2, std::abs(positions[99]));
}

/////////////////////////////////////////////////
TEST_F(PhysicsSystemFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(GetBoundingBox))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/contact.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1ns);

  // a map of model name to its axis aligned box
  std::map<std::string, math::AxisAlignedBox> bbox;

  // Create a system that records the bounding box of a model
  test::Relay testSystem;

  testSystem.OnPreUpdate(
    [&](const UpdateInfo &,
    EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Static>(
        [&](const Entity &_entity, const components::Model *,
        const components::Name *_name, const components::Static *)->bool
        {
          // create axis aligned box to be filled by physics
          if (_name->Data() == "box1")
          {
            auto bboxComp = _ecm.Component<components::AxisAlignedBox>(_entity);
            // the test only runs for 1 iteration so the component should be
            // null in the first iteration.
            EXPECT_EQ(bboxComp, nullptr);
            _ecm.CreateComponent(_entity, components::AxisAlignedBox());
            return true;
          }
          return true;
        });
    });

  testSystem.OnPostUpdate(
    [&](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      // store models that have axis aligned box computed
      _ecm.Each<components::Model, components::Name, components::Static,
        components::AxisAlignedBox>(
        [&](const Entity &, const components::Model *,
        const components::Name *_name, const components::Static *,
        const components::AxisAlignedBox *_aabb)->bool
        {
          bbox[_name->Data()] = _aabb->Data();
          return true;
        });
    });

  server.AddSystem(testSystem.systemPtr);
  const size_t iters = 1;
  server.Run(true, iters, false);

  EXPECT_EQ(1u, bbox.size());
  EXPECT_EQ("box1", bbox.begin()->first);
  EXPECT_EQ(math::AxisAlignedBox(
      math::Vector3d(-1.25, -2, 0),
      math::Vector3d(-0.25, 2, 1)),
      bbox.begin()->second);
}


/////////////////////////////////////////////////
// This tests whether nested models can be loaded correctly
TEST_F(PhysicsSystemFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(NestedModel))
{
  ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/nested_model.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  // Create a system that records the poses of the links after physics
  test::Relay testSystem;

  std::unordered_map<std::string, math::Pose3d> postUpModelPoses;
  std::unordered_map<std::string, math::Pose3d> postUpLinkPoses;
  std::unordered_map<std::string, std::string> parents;
  testSystem.OnPostUpdate(
    [&postUpModelPoses, &postUpLinkPoses, &parents](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const Entity &_entity, const components::Model *,
        const components::Name *_name, const components::Pose *_pose)->bool
        {
          // store model pose
          postUpModelPoses[_name->Data()] = _pose->Data();

          // store parent model name, if any
          auto parentId = _ecm.Component<components::ParentEntity>(_entity);
          if (parentId)
          {
            auto parentName =
                _ecm.Component<components::Name>(parentId->Data());
            parents[_name->Data()] = parentName->Data();
          }
          return true;
        });

      _ecm.Each<components::Link, components::Name, components::Pose,
                components::ParentEntity>(
        [&](const Entity &, const components::Link *,
        const components::Name *_name, const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
        {
          // store link pose
          postUpLinkPoses[_name->Data()] = _pose->Data();

          // store parent model name
          auto parentName = _ecm.Component<components::Name>(_parent->Data());
          parents[_name->Data()] = parentName->Data();
          return true;
        });

      return true;
    });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  EXPECT_EQ(2u, postUpModelPoses.size());
  EXPECT_EQ(2u, postUpLinkPoses.size());
  EXPECT_EQ(4u, parents.size());

  auto modelIt = postUpModelPoses.find("model_00");
  EXPECT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(0, 0, 0.5, 0, 0, 0), modelIt->second);

  modelIt = postUpModelPoses.find("model_01");
  EXPECT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(1.0, 0, 0.0, 0, 0, 0), modelIt->second);

  auto linkIt = postUpLinkPoses.find("link_00");
  EXPECT_NE(postUpLinkPoses.end(), linkIt);
  EXPECT_EQ(math::Pose3d(0, 0, 0.0, 0, 0, 0), linkIt->second);

  linkIt = postUpLinkPoses.find("link_01");
  EXPECT_NE(postUpLinkPoses.end(), linkIt);
  EXPECT_EQ(math::Pose3d(0.25, 0, 0.0, 0, 0, 0), linkIt->second);

  auto parentIt = parents.find("model_00");
  EXPECT_NE(parents.end(), parentIt);
  EXPECT_EQ("nested_model_world", parentIt->second);

  parentIt = parents.find("model_01");
  EXPECT_NE(parents.end(), parentIt);
  EXPECT_EQ("model_00", parentIt->second);

  parentIt = parents.find("link_00");
  EXPECT_NE(parents.end(), parentIt);
  EXPECT_EQ("model_00", parentIt->second);

  parentIt = parents.find("link_01");
  EXPECT_NE(parents.end(), parentIt);
  EXPECT_EQ("model_01", parentIt->second);
}

// This tests whether nested models can be loaded correctly
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(IncludeNestedModelDartsim))
{
  std::string path = std::string(PROJECT_SOURCE_PATH) + "/test/worlds/models";
  gz::common::setenv("GZ_SIM_RESOURCE_PATH", path.c_str());
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetResourceCache(path);
  serverConfig.SetPhysicsEngine("gz-physics-dartsim-plugin");

  const std::string sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/include_nested_models.sdf";
  serverConfig.SetSdfFile(sdfFile);
  sim::Server server(serverConfig);

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  server.SetUpdatePeriod(1us);

  // Create a system that records the poses of the links after physics
  test::Relay testSystem;

  std::unordered_map<std::string, gz::math::Pose3d> postUpModelPoses;
  std::unordered_map<std::string, gz::math::Pose3d> postUpLinkPoses;
  std::unordered_map<std::string, std::string> parents;
  testSystem.OnPostUpdate(
    [&postUpModelPoses, &postUpLinkPoses, &parents](const sim::UpdateInfo &,
    const sim::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const gz::sim::Entity &_entity, const components::Model *,
        const components::Name *_name, const components::Pose *_pose)->bool
        {
          // store model pose
          postUpModelPoses[_name->Data()] = _pose->Data();

          // store parent model name, if any
          auto parentId = _ecm.Component<components::ParentEntity>(_entity);
          if (parentId)
          {
            auto parentName =
                _ecm.Component<components::Name>(parentId->Data());
            parents[_name->Data()] = parentName->Data();
          }
          return true;
        });

      _ecm.Each<components::Link, components::Name, components::Pose,
                components::ParentEntity>(
        [&](const gz::sim::Entity &, const components::Link *,
        const components::Name *_name, const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
        {
          auto parentName = _ecm.Component<components::Name>(_parent->Data());
          const std::string qualifiedLinkName =
            parentName->Data() + "::" + _name->Data();
          // store link pose
          postUpLinkPoses[qualifiedLinkName] = _pose->Data();

          // store parent model name
          parents[qualifiedLinkName] = parentName->Data();
          return true;
        });

      return true;
    });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  // 2 in include_nested model, 3 in nested_models model
  EXPECT_EQ(5u, postUpModelPoses.size());

  // 0 in world, 3 in include_nested model, 2 in nested_models model
  EXPECT_EQ(5u, postUpLinkPoses.size());

  // 1 in world, 2 in include_nested, 4 in nested_models
  EXPECT_EQ(10u, parents.size());

  // From nested_models
  auto modelIt = postUpModelPoses.find("model_00");
  ASSERT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(0, 0, 0, 0, 0, 0), modelIt->second);

  // From nested_models
  modelIt = postUpModelPoses.find("model_01");
  ASSERT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(0, 0, 0.0, 0, 0, 0), modelIt->second);

  // From include_nested, but with name overwritten by include_nested_models
  modelIt = postUpModelPoses.find("include_nested_new_name");
  ASSERT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(1.0, 2.0, 3.0, 0, 0, 0), modelIt->second);

  // From nested_models, but with name overwritten by include_nested
  modelIt = postUpModelPoses.find("nested_models_new_name");
  ASSERT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(0, 0, 0, 0, 0, 0), modelIt->second);

  auto linkIt = postUpLinkPoses.find("model_00::link_00");
  ASSERT_NE(postUpLinkPoses.end(), linkIt);
  EXPECT_EQ(math::Pose3d(20, 21, 22, 0, 0, 0), linkIt->second);

  linkIt = postUpLinkPoses.find("include_nested_new_name::link_00");
  ASSERT_NE(postUpLinkPoses.end(), linkIt);
  EXPECT_EQ(math::Pose3d(30, 32, 34, 0, 0, 0), linkIt->second);

  linkIt = postUpLinkPoses.find("model_01::link_01");
  ASSERT_NE(postUpLinkPoses.end(), linkIt);
  EXPECT_EQ(math::Pose3d(20, 21, 22.0, 0, 0, 0), linkIt->second);

  auto parentIt = parents.find("model_00");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("nested_models_new_name", parentIt->second);

  parentIt = parents.find("model_01");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("model_00", parentIt->second);

  parentIt = parents.find("nested_models_new_name");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("include_nested_new_name", parentIt->second);

  parentIt = parents.find("include_nested_new_name");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("include_nested_models_world", parentIt->second);

  parentIt = parents.find("model_00::link_00");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("model_00", parentIt->second);

  parentIt = parents.find("include_nested_new_name::link_00");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("include_nested_new_name", parentIt->second);

  parentIt = parents.find("model_01::link_01");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("model_01", parentIt->second);
}

// This tests whether nested models can be loaded correctly
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(IncludeNestedModelTPE))
{
  std::string path = std::string(PROJECT_SOURCE_PATH) + "/test/worlds/models";
  gz::common::setenv("GZ_SIM_RESOURCE_PATH", path.c_str());
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetResourceCache(path);
  serverConfig.SetPhysicsEngine("gz-physics-tpe-plugin");

  const std::string sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/include_nested_models.sdf";
  serverConfig.SetSdfFile(sdfFile);
  sim::Server server(serverConfig);

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  server.SetUpdatePeriod(1us);

  // Create a system that records the poses of the links after physics
  test::Relay testSystem;

  std::unordered_map<std::string, gz::math::Pose3d> postUpModelPoses;
  std::unordered_map<std::string, gz::math::Pose3d> postUpLinkPoses;
  std::unordered_map<std::string, std::string> parents;
  testSystem.OnPostUpdate(
    [&postUpModelPoses, &postUpLinkPoses, &parents](const sim::UpdateInfo &,
    const sim::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const gz::sim::Entity &_entity, const components::Model *,
        const components::Name *_name, const components::Pose *_pose)->bool
        {
          // store model pose
          postUpModelPoses[_name->Data()] = _pose->Data();

          // store parent model name, if any
          auto parentId = _ecm.Component<components::ParentEntity>(_entity);
          if (parentId)
          {
            auto parentName =
                _ecm.Component<components::Name>(parentId->Data());
            parents[_name->Data()] = parentName->Data();
          }
          return true;
        });

      _ecm.Each<components::Link, components::Name, components::Pose,
                components::ParentEntity>(
        [&](const gz::sim::Entity &, const components::Link *,
        const components::Name *_name, const components::Pose *_pose,
        const components::ParentEntity *_parent)->bool
        {
          auto parentName = _ecm.Component<components::Name>(_parent->Data());
          const std::string qualifiedLinkName =
            parentName->Data() + "::" + _name->Data();
          // store link pose
          postUpLinkPoses[qualifiedLinkName] = _pose->Data();

          // store parent model name
          parents[qualifiedLinkName] = parentName->Data();
          return true;
        });

      return true;
    });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  // 2 in include_nested model, 3 in nested_models model
  EXPECT_EQ(5u, postUpModelPoses.size());

  // 0 in world, 3 in include_nested model, 2 in nested_models model
  EXPECT_EQ(5u, postUpLinkPoses.size());

  // 1 in world, 2 in include_nested, 4 in nested_models
  EXPECT_EQ(10u, parents.size());

  // From nested_models
  auto modelIt = postUpModelPoses.find("model_00");
  ASSERT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(0, 0, 0, 0, 0, 0), modelIt->second);

  // From nested_models
  modelIt = postUpModelPoses.find("model_01");
  ASSERT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(0, 0, 0.0, 0, 0, 0), modelIt->second);

  // From include_nested, but with name overwritten by include_nested_models
  modelIt = postUpModelPoses.find("include_nested_new_name");
  ASSERT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(1.0, 2.0, 3.0, 0, 0, 0), modelIt->second);

  // From nested_models, but with name overwritten by include_nested
  modelIt = postUpModelPoses.find("nested_models_new_name");
  ASSERT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_EQ(math::Pose3d(0, 0, 0, 0, 0, 0), modelIt->second);

  auto linkIt = postUpLinkPoses.find("model_00::link_00");
  ASSERT_NE(postUpLinkPoses.end(), linkIt);
  EXPECT_EQ(math::Pose3d(20, 21, 22, 0, 0, 0), linkIt->second);

  linkIt = postUpLinkPoses.find("include_nested_new_name::link_00");
  ASSERT_NE(postUpLinkPoses.end(), linkIt);
  EXPECT_EQ(math::Pose3d(30, 32, 34, 0, 0, 0), linkIt->second);

  linkIt = postUpLinkPoses.find("model_01::link_01");
  ASSERT_NE(postUpLinkPoses.end(), linkIt);
  EXPECT_EQ(math::Pose3d(20, 21, 22.0, 0, 0, 0), linkIt->second);

  auto parentIt = parents.find("model_00");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("nested_models_new_name", parentIt->second);

  parentIt = parents.find("model_01");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("model_00", parentIt->second);

  parentIt = parents.find("nested_models_new_name");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("include_nested_new_name", parentIt->second);

  parentIt = parents.find("include_nested_new_name");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("include_nested_models_world", parentIt->second);

  parentIt = parents.find("model_00::link_00");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("model_00", parentIt->second);

  parentIt = parents.find("include_nested_new_name::link_00");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("include_nested_new_name", parentIt->second);

  parentIt = parents.find("model_01::link_01");
  ASSERT_NE(parents.end(), parentIt);
  EXPECT_EQ("model_01", parentIt->second);
}

// This tests whether the poses of nested models are updated correctly
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(NestedModelIndividualCanonicalLinks))
{
  gz::sim::ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/nested_model_canonical_link.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  sim::Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  // Create a system that records the poses of the links after physics
  test::Relay testSystem;

  // Store a pointer to the ECM used in testSystem so that it can be used to
  // help verify poses at the end of this test
  const sim::EntityComponentManager *ecm;

  std::unordered_map<std::string, gz::math::Pose3d> postUpModelPoses;
  testSystem.OnPostUpdate(
    [&postUpModelPoses, &ecm](const sim::UpdateInfo &,
    const sim::EntityComponentManager &_ecm)
    {
      ecm = &_ecm;

      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const gz::sim::Entity &, const components::Model *,
        const components::Name *_name, const components::Pose *_pose)->bool
        {
          // store model pose
          postUpModelPoses[_name->Data()] = _pose->Data();
          return true;
        });

      return true;
    });

  server.AddSystem(testSystem.systemPtr);
  const size_t iters = 500;
  server.Run(true, iters, false);
  EXPECT_EQ(6u, postUpModelPoses.size());

  auto modelIt = postUpModelPoses.find("model_00");
  ASSERT_NE(postUpModelPoses.end(), modelIt);

  // link_00 is resting on the ground_box, so it remains stationary. And since
  // it is the canonical link of the parent model, model_00, the model frame
  // should also remain stationary.
  EXPECT_EQ(math::Pose3d::Zero, modelIt->second);

  // link_01 is floating, so it should fall due to gravity. And since it is the
  // canonical link of the nested model, model_01, the model frame should also
  // fall. If the model pose is not updated according to this canonical link,
  // the test would fail.
  const double dt = 0.001;
  const double zExpected = 0.5 * world->Gravity().Z() * pow(dt * iters, 2);
  modelIt = postUpModelPoses.find("model_01");
  ASSERT_NE(postUpModelPoses.end(), modelIt);
  EXPECT_NEAR(zExpected, modelIt->second.Z(), 1e-2);

  // model_10, model_11 and model_12 share the same canonical link (link_12),
  // which is a floating link. So, link_12 should fall due to gravity, which
  // also means that the frames for model_10, model_11 and model_12 should fall
  auto model10It = postUpModelPoses.find("model_10");
  ASSERT_NE(postUpModelPoses.end(), model10It);
  EXPECT_NEAR(zExpected, model10It->second.Z(), 1e-2);
  auto model11It = postUpModelPoses.find("model_11");
  ASSERT_NE(postUpModelPoses.end(), model11It);
  auto model12It = postUpModelPoses.find("model_12");
  ASSERT_NE(postUpModelPoses.end(), model12It);
  // (since model_11 and model_12 are nested, their poses are computed w.r.t.
  // model_10)
  EXPECT_DOUBLE_EQ(0.0, model11It->second.Z());
  EXPECT_DOUBLE_EQ(0.0, model12It->second.Z());
  // check the world pose of model_11 and model_12 to make sure the z components
  // of these world poses match zExpected
  ASSERT_NE(nullptr, ecm);
  const auto model11Entity =
    ecm->EntityByComponents(components::Name(model11It->first));
  ASSERT_NE(kNullEntity, model11Entity);
  const auto model11WorldPose = sim::worldPose(model11Entity, *ecm);
  EXPECT_NEAR(zExpected, model11WorldPose.Z(), 1e-2);
  const auto model12Entity =
    ecm->EntityByComponents(components::Name(model12It->first));
  ASSERT_NE(kNullEntity, model12Entity);
  const auto model12WorldPose = sim::worldPose(model12Entity, *ecm);
  EXPECT_NEAR(zExpected, model12WorldPose.Z(), 1e-2);
  EXPECT_DOUBLE_EQ(model11WorldPose.Z(), model12WorldPose.Z());
  // zExpected is also the z component of the world pose for model_10
  EXPECT_DOUBLE_EQ(model11WorldPose.Z(), model10It->second.Z());
  EXPECT_DOUBLE_EQ(model12WorldPose.Z(), model10It->second.Z());
}

/////////////////////////////////////////////////
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(DefaultPhysicsOptions))
{
  gz::sim::ServerConfig serverConfig;

  bool checked{false};

  // Create a system to check components
  test::Relay testSystem;
  testSystem.OnPostUpdate(
    [&checked](const sim::UpdateInfo &,
    const sim::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::World, components::PhysicsCollisionDetector,
                components::PhysicsSolver>(
        [&](const gz::sim::Entity &, const components::World *,
            const components::PhysicsCollisionDetector *_collisionDetector,
            const components::PhysicsSolver *_solver)->bool
        {
          EXPECT_NE(nullptr, _collisionDetector);
          if (_collisionDetector)
          {
            EXPECT_EQ("ode", _collisionDetector->Data());
          }
          EXPECT_NE(nullptr, _solver);
          if (_solver)
          {
            EXPECT_EQ("DantzigBoxedLcpSolver", _solver->Data());
          }
          checked = true;
          return true;
        });
    });

  sim::Server server(serverConfig);
  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  EXPECT_TRUE(checked);
}

/////////////////////////////////////////////////
TEST_F(PhysicsSystemFixture, PhysicsOptions)
{
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "physics_options.sdf"));

  bool checked{false};

  // Create a system to check components
  test::Relay testSystem;
  testSystem.OnPostUpdate(
    [&checked](const sim::UpdateInfo &,
    const sim::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::World, components::PhysicsCollisionDetector,
                components::PhysicsSolver>(
        [&](const gz::sim::Entity &, const components::World *,
            const components::PhysicsCollisionDetector *_collisionDetector,
            const components::PhysicsSolver *_solver)->bool
        {
          EXPECT_NE(nullptr, _collisionDetector);
          if (_collisionDetector)
          {
            EXPECT_EQ("bullet", _collisionDetector->Data());
          }
          EXPECT_NE(nullptr, _solver);
          if (_solver)
          {
            EXPECT_EQ("pgs", _solver->Data());
          }
          checked = true;
          return true;
        });
    });

  sim::Server server(serverConfig);
  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  EXPECT_TRUE(checked);
}

/////////////////////////////////////////////////
// This tests whether pose updates are correct for a model whose canonical link
// changes, but other links do not
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(MovingCanonicalLinkOnly))
{
  gz::sim::ServerConfig serverConfig;

  const auto sdfFile = common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds",
    "only_canonical_link_moves.sdf");

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  sim::Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  // Create a system that records the poses of the links after physics
  test::Relay testSystem;

  size_t numBaseLinkChecks = 0;
  size_t numOuterLinkChecks = 0;
  size_t numBaseLinkCustomChecks = 0;
  size_t numOuterLinkCustomChecks = 0;
  size_t numBaseLinkParentChecks = 0;
  size_t numNestedModelLinkChecks = 0;
  size_t numParentModelLinkChecks = 0;
  size_t numBaseLinkChildChecks = 0;

  size_t currIter = 0;
  testSystem.OnPostUpdate(
    [&](const sim::UpdateInfo &, const sim::EntityComponentManager &_ecm)
    {
      currIter++;
      _ecm.Each<components::Link, components::Name>(
          [&](const gz::sim::Entity &_entity,
          const components::Link *, const components::Name *_name)->bool
          {
            // ignore the link for the ground plane
            if (_name->Data() != "surface")
            {
              const double dt = 0.001;
              const double zExpected =
                0.5 * world->Gravity().Z() * pow(dt * currIter, 2);

              if (_name->Data() == "base_link")
              {
                // link "base_link" falls due to gravity, starting from rest
                EXPECT_NEAR(zExpected,
                    gz::sim::worldPose(_entity, _ecm).Z(), 1e-2);
                numBaseLinkChecks++;
              }
              else if (_name->Data() == "link0_outer")
              {
                // link "link0_outer" is resting on the ground and does not
                // move, so it should always have a pose of (1 0 0 0 0 0)
                EXPECT_EQ(gz::math::Pose3d(1, 0, 0, 0, 0, 0),
                    gz::sim::worldPose(_entity, _ecm));
                numOuterLinkChecks++;
              }
              else if (_name->Data() == "base_link_custom")
              {
                // same as link "base_link"
                EXPECT_NEAR(zExpected,
                    gz::sim::worldPose(_entity, _ecm).Z(), 1e-2);
                numBaseLinkCustomChecks++;
              }
              else if (_name->Data() == "link0_outer_custom")
              {
                // same as "link0_outer", but with an offset pose
                EXPECT_EQ(gz::math::Pose3d(1, 2, 0, 0, 0, 0),
                    gz::sim::worldPose(_entity, _ecm));
                numOuterLinkCustomChecks++;
              }
              else if (_name->Data() == "base_link_parent")
              {
                // same as link "base_link"
                EXPECT_NEAR(zExpected,
                    gz::sim::worldPose(_entity, _ecm).Z(), 1e-2);
                numBaseLinkParentChecks++;
              }
              else if (_name->Data() == "nested_model_link")
              {
                // same as "link0_outer", but with an offset pose
                EXPECT_EQ(gz::math::Pose3d(1, -2, 0, 0, 0, 0),
                    gz::sim::worldPose(_entity, _ecm));
                numNestedModelLinkChecks++;
              }
              else if (_name->Data() == "parent_model_link")
              {
                // same as "link0_outer", but with an offset pose
                EXPECT_EQ(gz::math::Pose3d(1, -4, 0, 0, 0, 0),
                    gz::sim::worldPose(_entity, _ecm));
                numParentModelLinkChecks++;
              }
              else if (_name->Data() == "base_link_child")
              {
                // same as link "base_link"
                EXPECT_NEAR(zExpected,
                    gz::sim::worldPose(_entity, _ecm).Z(), 1e-2);
                numBaseLinkChildChecks++;
              }
            }

            return true;
          });

      return true;
    });

  server.AddSystem(testSystem.systemPtr);
  const size_t iters = 500;
  server.Run(true, iters, false);

  EXPECT_EQ(iters, currIter);
  EXPECT_EQ(iters, numBaseLinkChecks);
  EXPECT_EQ(iters, numOuterLinkChecks);
  EXPECT_EQ(iters, numBaseLinkCustomChecks);
  EXPECT_EQ(iters, numOuterLinkCustomChecks);
  EXPECT_EQ(iters, numBaseLinkParentChecks);
  EXPECT_EQ(iters, numNestedModelLinkChecks);
  EXPECT_EQ(iters, numParentModelLinkChecks);
  EXPECT_EQ(iters, numBaseLinkChildChecks);
}

/////////////////////////////////////////////////
TEST_F(PhysicsSystemFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(Heightmap))
{
  gz::sim::ServerConfig serverConfig;

  const auto sdfFile = common::joinPaths(PROJECT_BINARY_PATH,
    "test", "worlds", "heightmap.sdf");
  serverConfig.SetSdfFile(sdfFile);

  sdf::Root root;
  root.Load(sdfFile);

  bool checked{false};
  int maxIt{0};

  test::Relay testSystem;
  testSystem.OnPostUpdate(
    [&](const sim::UpdateInfo &_info,
    const sim::EntityComponentManager &_ecm)
    {
      double aboveHeight;
      double farHeight;
      bool checkedAbove{false};
      bool checkedFar{false};
      bool checkedHeightmap{false};

      _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const gz::sim::Entity &, const components::Model *,
        const components::Name *_name, const components::Pose *_pose)->bool
        {
          if (_name->Data() == "above_heightmap")
          {
            aboveHeight = _pose->Data().Pos().Z();
            checkedAbove = true;
          }
          else if (_name->Data() == "far_from_heightmap")
          {
            farHeight = _pose->Data().Pos().Z();
            checkedFar = true;
          }
          else
          {
            EXPECT_EQ("Heightmap Bowl", _name->Data());
            EXPECT_EQ(math::Pose3d(), _pose->Data());
            checkedHeightmap = true;
          }

          return true;
        });

      EXPECT_TRUE(checkedAbove);
      EXPECT_TRUE(checkedFar);
      EXPECT_TRUE(checkedHeightmap);

      // Both models drop from 7m
      EXPECT_GE(7.01, aboveHeight) << _info.iterations;
      EXPECT_GE(7.01, farHeight) << _info.iterations;

      // Model above heightmap hits it and never drops below 5.5m
      EXPECT_LE(5.5, aboveHeight) << _info.iterations;

      // Model far from heightmap keeps falling
      if (_info.iterations > 600)
      {
        EXPECT_GT(5.5, farHeight) << _info.iterations;
      }

      checked = true;
      maxIt = _info.iterations;
      return true;
    });

  sim::Server server(serverConfig);
  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1000, false);

  EXPECT_TRUE(checked);
  EXPECT_EQ(1000, maxIt);
}

/////////////////////////////////////////////////
// Joint force
TEST_F(PhysicsSystemFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(JointTransmittedWrench))
{
  common::Console::SetVerbosity(4);
  gz::sim::ServerConfig serverConfig;

  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/joint_transmitted_wrench.sdf";

  serverConfig.SetSdfFile(sdfFile);

  sim::Server server(serverConfig);

  server.SetUpdatePeriod(1us);

  // Create a system that records the poses of the links after physics
  test::Relay testSystem;

  testSystem.OnPreUpdate(
      [&](const sim::UpdateInfo &_info, sim::EntityComponentManager &_ecm)
      {
        if (_info.iterations == 1)
        {
          _ecm.Each<components::Joint>(
              [&](const gz::sim::Entity &_entity,
                  const components::Joint *) -> bool
              {
                _ecm.CreateComponent(_entity,
                                     components::JointTransmittedWrench());
                return true;
              });
        }
      });

  const std::size_t totalIters = 1800;
  std::vector<msgs::Wrench> wrenches;
  wrenches.reserve(totalIters);
  // Simply collect joint wrenches. We check the values later.
  testSystem.OnPostUpdate(
      [&](const sim::UpdateInfo &,
          const sim::EntityComponentManager &_ecm)
      {
        const auto sensorJointEntity = _ecm.EntityByComponents(
            components::Joint(), components::Name("sensor_joint"));
        const auto jointWrench =
            _ecm.ComponentData<components::JointTransmittedWrench>(
                sensorJointEntity);
        if (jointWrench.has_value())
        {
          wrenches.push_back(*jointWrench);
        }
      });
  server.AddSystem(testSystem.systemPtr);
  server.Run(true, totalIters, false);

  ASSERT_EQ(totalIters, wrenches.size());

  const double kWeightScaleContactHeight = 0.05 + 0.25;
  const double kSensorMass = 0.2;
  const double kWeightMass = 10;
  const double kGravity = 9.8;
  const double kWeightInitialHeight = 0.5;
  const double dt = 0.001;
  const double timeOfContact = std::sqrt(
      2 * (kWeightInitialHeight - kWeightScaleContactHeight) / kGravity);
  std::size_t iterOfContact =
      static_cast<std::size_t>(std::round(timeOfContact / dt));

  for (std::size_t i = 0; i < iterOfContact - 10; ++i)
  {
    const auto &wrench = wrenches[i];
    EXPECT_NEAR(0.0, wrench.force().x(), 1e-3);
    EXPECT_NEAR(0.0, wrench.force().y(), 1e-3);
    EXPECT_NEAR(kGravity * kSensorMass, wrench.force().z(), 1e-3);
    EXPECT_EQ(math::Vector3d::Zero, msgs::Convert(wrench.torque()));
  }

  // Wait 300 (determined empirically) iterations for values to stabilize.
  for (std::size_t i = iterOfContact + 300; i < wrenches.size(); ++i)
  {
    const auto &wrench = wrenches[i];
    EXPECT_NEAR(0.0, wrench.force().x(), 1e-3);
    EXPECT_NEAR(0.0, wrench.force().y(), 1e-3);
    EXPECT_NEAR(kGravity * (kSensorMass + kWeightMass), wrench.force().z(),
                1e-3);
    EXPECT_EQ(math::Vector3d::Zero, msgs::Convert(wrench.torque()));
  }

  // Move the weight off center so it generates torque
  testSystem.OnPreUpdate(
      [&](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
      {
        const auto weightEntity = _ecm.EntityByComponents(
            components::Model(), components::Name("weight"));
        _ecm.SetComponentData<components::WorldPoseCmd>(
            weightEntity, math::Pose3d(0.2, 0.1, 0.5, 0, 0, 0));
      });

  server.RunOnce();
  // Reset PreUpdate so it doesn't keep moving the weight
  testSystem.OnPreUpdate({});
  wrenches.clear();
  server.Run(true, totalIters, false);
  ASSERT_EQ(totalIters, wrenches.size());

  // Wait 300 (determined empirically) iterations for values to stabilize.
  for (std::size_t i = iterOfContact + 300; i < wrenches.size(); ++i)
  {
    const auto &wrench = wrenches[i];
    EXPECT_NEAR(0.0, wrench.force().x(), 1e-3);
    EXPECT_NEAR(0.0, wrench.force().y(), 1e-3);
    EXPECT_NEAR(kGravity * (kSensorMass + kWeightMass), wrench.force().z(),
                1e-3);

    EXPECT_NEAR(0.1 * kGravity * kWeightMass, wrench.torque().x(), 1e-3);
    EXPECT_NEAR(-0.2 * kGravity * kWeightMass, wrench.torque().y(), 1e-3);
    EXPECT_NEAR(0.0, wrench.torque().z(), 1e-3);
  }
}

/////////////////////////////////////////////////
// Test that joint velocity limit is applied
TEST_F(PhysicsSystemFixtureWithDart6_10,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(JointVelocityLimitTest))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
          "/test/worlds/joint_velocity_limit.sdf";
  serverConfig.SetSdfFile(sdfFile);

  auto server = std::make_unique<Server>(serverConfig);
  EXPECT_FALSE(server->Running());
  EXPECT_FALSE(*server->Running(0));

  server->SetUpdatePeriod(1ns);

  test::Relay testSystem;

  const std::size_t nIters{600};
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &_info, EntityComponentManager &_ecm)
      {
        // Create components, if they don't exist on the first iteration
        if (_info.iterations == 1)
        {
          for (const auto &e : _ecm.EntitiesByComponents(components::Joint()))
          {
            if (!_ecm.Component<components::JointVelocity>(e))
            {
              _ecm.CreateComponent(e, components::JointVelocity());
            }
          }
        }
      });

  testSystem.OnPostUpdate(
      [&](const UpdateInfo &_info,
          const EntityComponentManager &_ecm)
      {
        // At nIters iterations, check angular velocity of each of the joints
        if (_info.iterations == nIters)
        {
          int count = 0;
          for (const auto &e : _ecm.EntitiesByComponents(components::Joint()))
          {
            auto *jointVel = _ecm.Component<components::JointVelocity>(e);
            EXPECT_NE(nullptr, jointVel);
            EXPECT_FALSE(jointVel->Data().empty());
            if (jointVel->Data().size() > 0)
            {
              ++count;
              // Joint velocity should lie between
              // - (kVelocityLimit + kTolerance) and
              // + (kVelocityLimit + kTolerance)
              const double kVelocityLimit = 1.0;
              const double kTolerance = 1e-6;
              EXPECT_NEAR(jointVel->Data()[0], 0, kVelocityLimit + kTolerance);
            }
          }
          EXPECT_EQ(count, 2);
        }
      });

  server->AddSystem(testSystem.systemPtr);
  server->Run(true, nIters, false);
}


//////////////////////////////////////////////////
/// This test makes sure that non-link entities' components
/// are updated by the physics system if they have been enabled.
/// A collision is a non-link entity
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(NonLinkComponentsAreUpdated))
{
  ServerConfig serverConfig;

  const auto sdfFile =
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/non_link_components.sdf";

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *world = root.WorldByIndex(0);
  auto gravity = world->Gravity();
  ASSERT_TRUE(nullptr != world);

  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  server.SetUpdatePeriod(1ms);

  const std::string collisionName{"collision"};
  const std::string collisionWithOffsetName{"collision_with_offset"};

  // Create a system that records the pose, vel, acc of the collisions.
  test::Relay testSystem;

  size_t iterations = 0u;
  std::size_t nIters{100};
  std::size_t halfIters{nIters / 2};

  std::vector<math::Pose3d> collisionPoses;
  std::vector<math::Vector3d> linVels, angVels, worldLinVels, worldAngVels;
  std::vector<math::Vector3d> linAccs, angAccs, worldLinAccs, worldAngAccs;
  testSystem.OnPreUpdate(
      [&iterations, &nIters, &halfIters,
       &collisionWithOffsetName](const UpdateInfo &,
                                 EntityComponentManager &_ecm)
      {
        _ecm.EachNew<components::Collision, components::Name,
                     components::ParentEntity>(
            [&](const Entity &_entity, const components::Collision *,
                const components::Name *_name,
                const components::ParentEntity *) -> bool
            {
              // we only enable the velocity, acceleration components for
              // the collision_with_offset
              if (_name->Data() == collisionWithOffsetName)
              {
                enableComponent<components::WorldPose>(_ecm, _entity);
                enableComponent<components::LinearVelocity>(_ecm, _entity);
                enableComponent<components::AngularVelocity>(_ecm, _entity);
                enableComponent<components::WorldLinearVelocity>(_ecm, _entity);
                enableComponent<components::WorldAngularVelocity>(_ecm,
                                                                  _entity);
                enableComponent<components::LinearAcceleration>(_ecm, _entity);
                enableComponent<components::AngularAcceleration>(_ecm, _entity);
                enableComponent<components::WorldLinearAcceleration>(_ecm,
                                                                     _entity);
                enableComponent<components::WorldAngularAcceleration>(_ecm,
                                                                      _entity);
              }
              return true;
            });

        _ecm.Each<components::Collision, components::Name,
                  components::ParentEntity>(
            [&](const Entity &, const components::Collision *,
                const components::Name *_name,
                const components::ParentEntity *_parentEntity) -> bool
            {
              // after half of the simulation, apply a varying wrench, on the
              // link
              if (_name->Data() == collisionWithOffsetName &&
                  iterations >= halfIters)
              {
                gz::sim::Link parentLink(_parentEntity->Data());
                parentLink.EnableVelocityChecks(_ecm);
                parentLink.EnableAccelerationChecks(_ecm);
                using namespace gz::math;
                // increasing force downward
                auto force =
                    -Vector3d::UnitZ * (static_cast<double>(iterations) /
                                        static_cast<double>(nIters));
                // increasing torque around z
                auto torque =
                    Vector3d::UnitZ * (static_cast<double>(iterations) /
                                       static_cast<double>(nIters));
                parentLink.AddWorldWrench(_ecm, force, torque);
              }
              return true;
            });
      });

  // save all the components history
  testSystem.OnPostUpdate(
      [&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        _ecm.Each<
            components::Collision, components::Name, components::WorldPose,
            components::LinearVelocity, components::AngularVelocity,
            components::WorldLinearVelocity, components::WorldAngularVelocity,
            components::LinearAcceleration, components::AngularAcceleration,
            components::WorldLinearAcceleration,
            components::WorldAngularAcceleration>(
            [&](const Entity &, const components::Collision *,
                const components::Name *_name,
                const components::WorldPose *_pose,
                const components::LinearVelocity *_linearVel,
                const components::AngularVelocity *_angularVel,
                const components::WorldLinearVelocity *_worldLinearVel,
                const components::WorldAngularVelocity *_worldAngularVel,
                const components::LinearAcceleration *_linearAcc,
                const components::AngularAcceleration *_angularAcc,
                const components::WorldLinearAcceleration *_worldLinearAcc,
                const components::WorldAngularAcceleration *_worldAngularAcc)
                -> bool
            {
              EXPECT_TRUE(_name->Data() == collisionWithOffsetName);
              if (_name->Data() == collisionWithOffsetName)
              {
                collisionPoses.push_back(_pose->Data());
                linVels.push_back(_linearVel->Data());
                angVels.push_back(_angularVel->Data());
                worldLinVels.push_back(_worldLinearVel->Data());
                worldAngVels.push_back(_worldAngularVel->Data());
                linAccs.push_back(_linearAcc->Data());
                angAccs.push_back(_angularAcc->Data());
                worldLinAccs.push_back(_worldLinearAcc->Data());
                worldAngAccs.push_back(_worldAngularAcc->Data());
              }
              return true;
            });
        ++iterations;
        return true;
      });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, nIters, false);

  ASSERT_EQ(nIters, collisionPoses.size());
  ASSERT_EQ(nIters, linVels.size());
  ASSERT_EQ(nIters, angVels.size());
  ASSERT_EQ(nIters, worldLinVels.size());
  ASSERT_EQ(nIters, worldAngVels.size());
  ASSERT_EQ(nIters, linAccs.size());
  ASSERT_EQ(nIters, angAccs.size());
  ASSERT_EQ(nIters, worldLinAccs.size());
  ASSERT_EQ(nIters, worldAngAccs.size());
  ASSERT_EQ(nIters, iterations);

  // The collision offset should be taken into account in the world pose
  EXPECT_NEAR(0.1, collisionPoses.front().Pos().X(), 1e-2);
  EXPECT_NEAR(0.2, collisionPoses.front().Pos().Y(), 1e-2);
  EXPECT_NEAR(2, collisionPoses.front().Pos().Z(), 1e-2);

  // Make sure the position is updated every time step
  // (here, the model is falling down, so only z should be updated)
  for (size_t i = 1; i < nIters; i++)
  {
    EXPECT_GT(collisionPoses[i - 1].Pos().Z(), collisionPoses[i].Pos().Z())
        << "Model should be falling down.";
  }

  double normLinVelYz, normWorldLinVelXy;
  double normLinAccYz, normWorldLinAccXy;
  // the first part checks that:
  // - the collision poses are updated correctly
  // - linear velocities (world/local) are updated.
  for (size_t i = 1; i < halfIters; i++)
  {
    // --- LIN ACC (WORLD, LOCAL)
    // linear acc is constant = gravity (both world and local)
    EXPECT_NEAR(linAccs[i].Length(), gravity.Length(), 1e-2);
    EXPECT_NEAR(worldLinAccs[i].Length(), gravity.Length(), 1e-2);
    // local should mostly be X , world should be -Z
    EXPECT_NEAR(linAccs[i].X(), gravity.Length(), 1e-2)
        << "Local linear acceleration should be along X axis";
    EXPECT_NEAR(worldLinAccs[i].Z(), -gravity.Length(), 1e-2)
        << "World Linear acceleration should be along -Z axis";
    // on the local(world) YZ(XY) plane the acceleration should be 0
    normLinAccYz =
        sqrt(linAccs[i].Y() * linAccs[i].Y() + linAccs[i].Z() * linAccs[i].Z());
    EXPECT_NEAR(normLinAccYz, 0, 1e-2)
        << "Local Linear acceleration on YZ-plane should be 0";
    normWorldLinAccXy = sqrt(worldLinAccs[i].X() * worldLinAccs[i].X() +
                             worldLinAccs[i].Y() * worldLinAccs[i].Y());
    EXPECT_NEAR(normWorldLinAccXy, 0, 1e-2)
        << "World Linear acceleration on XY-plane should be 0";

    // --- LIN VEL (WORLD, LOCAL)
    // linear velocity should keep increasing, both local and world
    EXPECT_LT(linVels[i - 1].Length(), linVels[i].Length())
        << "Local linear velocity should keep increasing";
    EXPECT_LT(worldLinVels[i - 1].Length(), worldLinVels[i].Length())
        << "World linear velocity should keep increasing";
    // local should mostly be X , world should be -Z
    EXPECT_GT(linVels[i - 1].X(), 0)
        << "Local linear vel should be positive in x";
    EXPECT_LT(worldLinVels[i - 1].Z(), 0)
        << "World linear vel should be negative in z";
    // on the local(world) YZ(XY) plane the velocity should be 0
    normLinVelYz =
        sqrt(linVels[i].Y() * linVels[i].Y() + linVels[i].Z() * linVels[i].Z());
    EXPECT_NEAR(normLinVelYz, 0, 1e-2)
        << "Local Linear velocity on YZ-plane should be zero";
    normWorldLinVelXy = sqrt(worldLinVels[i].X() * worldLinVels[i].X() +
                             worldLinVels[i].Y() * worldLinVels[i].Y());
    EXPECT_NEAR(normWorldLinVelXy, 0, 1e-2)
        << "World Linear acceleration on XY-plane should be zero";

    // --- ANG ACC (WORLD, LOCAL)
    // angular acc is constant = 0 (both world and local)
    EXPECT_NEAR(angAccs[i].Length(), 0, 1e-2);
    EXPECT_NEAR(worldAngAccs[i].Length(), 0, 1e-2);
    // --- ANG VEL (WORLD, LOCAL)
    // angular velocity is constant
    EXPECT_NEAR(angVels[i].Length(), 0, 1e-2);
    EXPECT_NEAR(worldAngVels[i].Length(), 0, 1e-2);
  }

  // Second part, we applied a varying wrench, and we should see:
  //  - linear acceleration (world/local) are updated
  //  - angular acceleration (world/local) are updated
  //  - angular velocities (world/local) are updated
  for (size_t i = halfIters + 1; i < nIters; i++)
  {
    // --- LIN ACC (WORLD, LOCAL)
    EXPECT_LT(linAccs[i - 1].Length(), linAccs[i].Length())
        << "Local Linear Acceleration should be increasing.";
    EXPECT_LT(worldLinAccs[i - 1].Length(), worldLinAccs[i].Length())
        << "World Linear Acceleration should be increasing.";
    // on the local(world) YZ(XY) plane the acceleration norm should be positive
    normLinAccYz =
        sqrt(linAccs[i].Y() * linAccs[i].Y() + linAccs[i].Z() * linAccs[i].Z());
    EXPECT_GT(normLinAccYz, 0)
        << "Local Linear acceleration on YZ-plane should be positive";
    normWorldLinAccXy = sqrt(worldLinAccs[i].X() * worldLinAccs[i].X() +
                             worldLinAccs[i].Y() * worldLinAccs[i].Y());
    EXPECT_GT(normWorldLinAccXy, 0)
        << "World Linear acceleration on XY-plane should be positive";
    // on the local(world) YZ(XY) plane the velocity should be positive
    normLinVelYz =
        sqrt(linVels[i].Y() * linVels[i].Y() + linVels[i].Z() * linVels[i].Z());
    EXPECT_GT(normLinVelYz, 0)
        << "Local Linear velocity on YZ-plane should be positive";
    normWorldLinVelXy = sqrt(worldLinVels[i].X() * worldLinVels[i].X() +
                             worldLinVels[i].Y() * worldLinVels[i].Y());
    EXPECT_GT(normWorldLinVelXy, 0)
        << "World Linear acceleration on XY-plane should be positive";

    // --- ANG ACC (WORLD, LOCAL)
    EXPECT_LT(angAccs[i - 1].Length(), angAccs[i].Length())
        << "Local Angular Acceleration should be increasing.";
    EXPECT_LT(worldAngAccs[i - 1].Length(), worldAngAccs[i].Length())
        << "World Angular Acceleration should be increasing.";

    // --- ANG VEL (WORLD, LOCAL)
    EXPECT_LT(angVels[i - 1].Length(), angVels[i].Length())
        << "Local Angular Velocity should be increasing.";
    EXPECT_LT(worldAngVels[i - 1].Length(), worldAngVels[i].Length())
        << "World Angular Velocity should be increasing.";
  }
}

//////////////////////////////////////////////////
/// Tests that joints inside <world> are supported by computing the position of
/// a pendulum bob from the joint angle. This also tests that commands such as
/// JointPositionReset work as expected.
TEST_F(PhysicsSystemFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(JointsInWorld))
{
  ServerConfig serverConfig;

  const auto sdfFile = common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds",
                                         "joints_in_world.sdf");

  serverConfig.SetSdfFile(sdfFile);
  Server server(serverConfig);
  server.SetUpdatePeriod(1ns);

  const int kIters = 1000;
  test::Relay testSystem;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &_info, EntityComponentManager &_ecm)
      {
        _ecm.EachNew<components::Joint, components::ParentEntity>(
            [&](const Entity &_entity, const components::Joint *,
                const components::ParentEntity *) -> bool
            {
              enableComponent<components::JointPosition>(_ecm, _entity);
              enableComponent<components::JointVelocity>(_ecm, _entity);
              return true;
            });

        if (_info.iterations == kIters / 2)
        {
          // After half the iterations, reset the joint position and velocity so
          // that the bob at its equilibrium point and at rest.
          auto jointEntity = _ecm.EntityByComponents(components::Name("j1"),
                                                     components::Joint());
          ASSERT_NE(jointEntity, kNullEntity);
          _ecm.SetComponentData<components::JointVelocityReset>(jointEntity,
                                                                {0});
          _ecm.SetComponentData<components::JointPositionReset>(jointEntity,
                                                                {GZ_PI_2});
        }
      });
  testSystem.OnPostUpdate(
      [&](const UpdateInfo &_info, const EntityComponentManager &_ecm)
      {
        auto jointEntity = _ecm.EntityByComponents(components::Name("j1"),
                                                   components::Joint());
        ASSERT_NE(jointEntity, kNullEntity);

        auto m2Entity = _ecm.EntityByComponents(components::Name("m2"),
                                                components::Model());
        ASSERT_NE(m2Entity, kNullEntity);

        math::Pose3d m2Pose = worldPose(m2Entity, _ecm);

        auto jointPos =
            _ecm.ComponentData<components::JointPosition>(jointEntity);
        ASSERT_TRUE(jointPos.has_value());
        ASSERT_EQ(1u, jointPos->size());

        if (_info.iterations < kIters / 2)
        {
          // If the joint is properly working, the position of the model can be
          // determined from the joint angle with the equations:
          // x = L*cos(θ)
          // z = -L*sin(θ)
          // where L is the length of the model from the pivot (2m for this
          // test).
          const double theta = (*jointPos)[0];
          const double kLength = 2.0;
          EXPECT_NEAR(m2Pose.X(), kLength * cos(theta), 1e-2);
          EXPECT_NEAR(m2Pose.Z(), -kLength * sin(theta), 1e-2);
        }
        else
        {
          EXPECT_NEAR((*jointPos)[0], GZ_PI_2, 1e-2);
        }
      });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1000, false);
}

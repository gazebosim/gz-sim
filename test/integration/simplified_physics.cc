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

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>
#include <sdf/Plugin.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/System.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/PhysicsOwner.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "gz/sim/components/SupportedPhysicsFeatures.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

namespace
{
const char *kProvider = "gz::sim::systems::SimplifiedPhysics";

/// \brief World step size, from the world file.
constexpr double kDt{0.001};

/// \brief Number of steps of a run: one second of simulated time.
constexpr int kIterations{1000};

/// \brief Step in which the test sends a WorldPoseCmd.
constexpr int kTeleportStep{100};

/// \brief Step until which the test sends velocity commands by hand.
constexpr int kCommandedUntil{200};

/// \brief Models that the test follows.
const std::vector<std::string> kModels{
    "full_falling", "full_driven", "kin_line", "kin_yawed", "kin_arc",
    "kin_spin", "kin_tilted", "kin_idle", "kin_gated", "kin_gated_test_plugin",
    "kin_teleport", "kin_teleport_moving", "dynamic_requested"};

/// \brief Models that ask for a planar mode and are refused.
const std::vector<std::string> kRefused{"kin_unknown_mode", "kin_static",
    "inner"};

/// \brief What the test saw for one model.
struct Observation
{
  /// \brief Entities of the model.
  Entity model{kNullEntity};
  Entity link{kNullEntity};
  Entity collision{kNullEntity};

  /// \brief Initial pose, from the world file.
  math::Pose3d initialPose;

  /// \brief World pose after each step.
  std::vector<math::Pose3d> poses;

  /// \brief State of the model's Pose component after each step.
  std::vector<ComponentState> poseStates;

  /// \brief World linear velocity of the link after each step.
  std::vector<math::Vector3d> linkWorldVelocity;

  /// \brief Linear velocity of the link in the link frame after each step.
  std::vector<math::Vector3d> linkBodyVelocity;

  /// \brief World angular velocity of the link after each step.
  std::vector<math::Vector3d> linkWorldAngularVelocity;

  /// \brief World pose of the link after each step.
  std::vector<math::Pose3d> linkWorldPose;

  /// \brief World pose, linear velocity, and body frame linear acceleration
  /// of the collision after each step.
  std::vector<math::Pose3d> collisionWorldPose;
  std::vector<math::Vector3d> collisionWorldVelocity;
  std::vector<math::Vector3d> collisionBodyAcceleration;

  /// \brief True after the step in which a WorldPoseCmd existed.
  std::vector<bool> worldPoseCmdPresent;

  /// \brief Number of steps after which a velocity command was still set.
  int velocityCmdKept{0};
};

/// \brief Test fixture.
class SimplifiedPhysicsTest : public InternalFixture<::testing::Test>
{
  /// \brief Run the world.
  /// \param[in] _preUpdate Optional hook that runs in every PreUpdate with
  /// the number of the step that is about to run, starting at 1.
  /// \param[out] _stderr Text that the server wrote to stderr.
  /// \return What the test saw, by model name.
  protected: std::map<std::string, Observation> Run(
      const std::function<void(EntityComponentManager &,
          std::map<std::string, Observation> &, int)> &_preUpdate,
      std::string &_stderr)
  {
    ::testing::internal::CaptureStderr();

    TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "simplified_physics.sdf"));

    std::map<std::string, Observation> obs;
    int step{0};

    fixture.OnConfigure([&](
        const Entity &,
        const std::shared_ptr<const sdf::Element> &,
        EntityComponentManager &_ecm,
        EventManager &)
        {
          for (const auto &name : kModels)
          {
            Model model(_ecm.EntityByComponents(
                components::Model(), components::Name(name)));
            ASSERT_TRUE(model.Valid(_ecm)) << name;

            auto &o = obs[name];
            o.model = model.Entity();
            o.link = model.LinkByName(_ecm, "link");
            o.collision = Link(o.link).CollisionByName(_ecm, "collision");
            o.initialPose = worldPose(o.model, _ecm);

            // Other systems create these when they need the data. Create
            // them here so that the physics provider writes them.
            for (const auto entity : {o.link, o.collision})
            {
              _ecm.CreateComponent(entity, components::WorldPose());
              _ecm.CreateComponent(entity, components::WorldLinearVelocity());
              _ecm.CreateComponent(entity, components::WorldAngularVelocity());
              _ecm.CreateComponent(entity, components::LinearVelocity());
              _ecm.CreateComponent(entity, components::LinearAcceleration());
            }
          }
        })
    .OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
        {
          ++step;
          if (_preUpdate)
            _preUpdate(_ecm, obs, step);
        })
    .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
        {
          for (auto &[name, o] : obs)
          {
            o.poses.push_back(worldPose(o.model, _ecm));
            o.poseStates.push_back(
                _ecm.ComponentState(o.model, components::Pose::typeId));

            o.linkWorldPose.push_back(
                _ecm.Component<components::WorldPose>(o.link)->Data());
            o.linkWorldVelocity.push_back(
                _ecm.Component<components::WorldLinearVelocity>(o.link)
                    ->Data());
            o.linkWorldAngularVelocity.push_back(
                _ecm.Component<components::WorldAngularVelocity>(o.link)
                    ->Data());
            o.linkBodyVelocity.push_back(
                _ecm.Component<components::LinearVelocity>(o.link)->Data());

            o.collisionWorldPose.push_back(
                _ecm.Component<components::WorldPose>(o.collision)->Data());
            o.collisionWorldVelocity.push_back(
                _ecm.Component<components::WorldLinearVelocity>(o.collision)
                    ->Data());
            o.collisionBodyAcceleration.push_back(
                _ecm.Component<components::LinearAcceleration>(o.collision)
                    ->Data());

            o.worldPoseCmdPresent.push_back(
                _ecm.Component<components::WorldPoseCmd>(o.model) != nullptr);
            // The system zeroes the command and keeps the component.
            auto cmd = _ecm.Component<components::LinearVelocityCmd>(o.model);
            if (cmd && cmd->Data() != math::Vector3d::Zero)
              ++o.velocityCmdKept;
          }
        })
    .Finalize();

    fixture.Server()->Run(true, kIterations, false);

    _stderr = ::testing::internal::GetCapturedStderr();
    return obs;
  }

  /// \brief Check that a vector is close to an expected one.
  protected: static void ExpectNear(const math::Vector3d &_want,
      const math::Vector3d &_got, double _tol, const std::string &_what)
  {
    EXPECT_NEAR(_want.X(), _got.X(), _tol) << _what;
    EXPECT_NEAR(_want.Y(), _got.Y(), _tol) << _what;
    EXPECT_NEAR(_want.Z(), _got.Z(), _tol) << _what;
  }
};
}

/////////////////////////////////////////////////
/// The model plugin hands the model to SimplifiedPhysics, and the features
/// are an empty list: no feature is served.
TEST_F(SimplifiedPhysicsTest, ModelPluginComponents)
{
  ::testing::internal::CaptureStderr();
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "simplified_physics.sdf"));

  std::map<std::string, std::string> owners;
  std::map<std::string, bool> hasFeatures;
  std::map<std::string, size_t> featureCount;
  auto names = kModels;
  names.insert(names.end(), kRefused.begin(), kRefused.end());
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        for (const auto &name : names)
        {
          const auto model = _ecm.EntityByComponents(
              components::Model(), components::Name(name));
          if (auto owner = _ecm.Component<components::PhysicsOwner>(model))
            owners[name] = owner->Data();
          auto features =
              _ecm.Component<components::SupportedPhysicsFeatures>(model);
          hasFeatures[name] = features != nullptr;
          if (features)
            featureCount[name] = features->Data().size();
        }
      }).Finalize();
  fixture.Server()->Run(true, 2, false);
  const auto output = ::testing::internal::GetCapturedStderr();

  for (const auto &name : {"kin_line", "kin_yawed", "kin_arc", "kin_spin",
      "kin_tilted", "kin_idle", "kin_gated", "kin_gated_test_plugin",
      "kin_teleport", "kin_teleport_moving"})
  {
    EXPECT_EQ(kProvider, owners[name]) << name;
    EXPECT_TRUE(hasFeatures[name]) << name;
    EXPECT_EQ(0u, featureCount[name]) << name;
  }

  // Models on Physics have neither component. This includes the models whose
  // request was refused.
  auto onPhysics = kRefused;
  onPhysics.insert(onPhysics.end(),
      {"full_falling", "full_driven", "dynamic_requested"});
  for (const auto &name : onPhysics)
  {
    EXPECT_EQ(0u, owners.count(name)) << name;
    EXPECT_FALSE(hasFeatures[name]) << name;
  }

  // Each refusal names the model and says why.
  EXPECT_NE(std::string::npos, output.find("Model [dynamic_requested] asks "
      "for physics mode [planar_dynamic], which is not implemented yet"))
      << output;
  EXPECT_NE(std::string::npos, output.find("Model [kin_unknown_mode] has "
      "unknown physics mode [tier1]")) << output;
  EXPECT_NE(std::string::npos, output.find("Model [kin_static] is static"))
      << output;
  EXPECT_NE(std::string::npos, output.find("Model [inner] is nested"))
      << output;
}

/////////////////////////////////////////////////
/// A planar_kinematic model under VelocityControl reaches the pose that the
/// exact integration predicts, and Physics does not touch it.
TEST_F(SimplifiedPhysicsTest, KinematicMotion)
{
  std::string output;
  auto obs = this->Run(nullptr, output);

  for (int step : {1, 7, 500, kIterations})
  {
    const double t = step * kDt;
    const auto index = static_cast<size_t>(step - 1);

    // Straight line along x.
    {
      const auto &o = obs["kin_line"];
      ASSERT_EQ(static_cast<size_t>(kIterations), o.poses.size());
      math::Pose3d want = o.initialPose;
      want.Pos().X() += t;
      EXPECT_NEAR(want.Pos().X(), o.poses[index].Pos().X(), 1e-6) << step;
      EXPECT_NEAR(want.Pos().Y(), o.poses[index].Pos().Y(), 1e-6) << step;
      EXPECT_NEAR(want.Pos().Z(), o.poses[index].Pos().Z(), 1e-6) << step;
      EXPECT_TRUE(want.Rot().Equal(o.poses[index].Rot(), 1e-6)) << step;
    }

    // Facing +y: forward is world y.
    {
      const auto &o = obs["kin_yawed"];
      const auto &pose = o.poses[index];
      EXPECT_NEAR(o.initialPose.Pos().X(), pose.Pos().X(), 1e-6) << step;
      EXPECT_NEAR(o.initialPose.Pos().Y() + t, pose.Pos().Y(), 1e-6) << step;
      EXPECT_NEAR(GZ_PI_2, pose.Rot().Yaw(), 1e-6) << step;
    }

    // v = 1 m/s and w = 1 rad/s: a circle of radius 1.
    {
      const auto &o = obs["kin_arc"];
      const auto &pose = o.poses[index];
      EXPECT_NEAR(o.initialPose.Pos().X() + std::sin(t), pose.Pos().X(), 1e-6)
          << step;
      EXPECT_NEAR(o.initialPose.Pos().Y() + 1.0 - std::cos(t), pose.Pos().Y(),
          1e-6) << step;
      EXPECT_NEAR(t, pose.Rot().Yaw(), 1e-6) << step;
    }

    // Turning in place.
    {
      const auto &o = obs["kin_spin"];
      const auto &pose = o.poses[index];
      EXPECT_NEAR(o.initialPose.Pos().X(), pose.Pos().X(), 1e-6) << step;
      EXPECT_NEAR(o.initialPose.Pos().Y(), pose.Pos().Y(), 1e-6) << step;
      EXPECT_NEAR(2.0 * t, pose.Rot().Yaw(), 1e-6) << step;
    }

    // Pitched by 0.3 rad. The forward command has a vertical part, which
    // the planar mode drops. z, roll and pitch keep their values.
    {
      const auto &o = obs["kin_tilted"];
      const auto &pose = o.poses[index];
      EXPECT_NEAR(o.initialPose.Pos().X() + t * std::cos(0.3), pose.Pos().X(),
          1e-6) << step;
      EXPECT_NEAR(o.initialPose.Pos().Y(), pose.Pos().Y(), 1e-6) << step;
      EXPECT_NEAR(o.initialPose.Pos().Z(), pose.Pos().Z(), 1e-9) << step;
      EXPECT_NEAR(0.0, pose.Rot().Roll(), 1e-9) << step;
      EXPECT_NEAR(0.3, pose.Rot().Pitch(), 1e-9) << step;
    }
  }

  // Physics did not simulate any of them: no model fell.
  for (const auto &name : {"kin_line", "kin_yawed", "kin_arc", "kin_spin",
      "kin_tilted", "kin_idle"})
  {
    EXPECT_NEAR(1.0, obs[name].poses.back().Pos().Z(), 1e-9) << name;
  }

  // A model without a command stays where it is, and the system does not
  // mark its pose as changed.
  const auto &idle = obs["kin_idle"];
  EXPECT_EQ(idle.initialPose, idle.poses.back());
  for (size_t i = 5; i < idle.poseStates.size(); ++i)
    EXPECT_EQ(ComponentState::NoChange, idle.poseStates[i]) << i;

  // A moving model is marked as changed in every step.
  for (const auto &name : {"kin_line", "kin_arc", "kin_spin"})
  {
    for (const auto state : obs[name].poseStates)
      EXPECT_EQ(ComponentState::PeriodicChange, state) << name;
  }

  // SimplifiedPhysics consumed the velocity commands.
  EXPECT_EQ(0, obs["kin_line"].velocityCmdKept);
  EXPECT_EQ(0, obs["kin_arc"].velocityCmdKept);
}

/////////////////////////////////////////////////
/// Physics still simulates its own models, next to the other provider.
TEST_F(SimplifiedPhysicsTest, PhysicsKeepsItsModels)
{
  std::string output;
  auto obs = this->Run(nullptr, output);

  // A model on Physics falls and lands on the ground.
  const auto &falling = obs["full_falling"];
  EXPECT_LT(falling.poses[300].Pos().Z(), falling.initialPose.Pos().Z() - 0.3);
  EXPECT_NEAR(0.1, falling.poses.back().Pos().Z(), 0.05);

  // A model on Physics follows its velocity command.
  const auto &driven = obs["full_driven"];
  EXPECT_GT(driven.poses.back().Pos().X(),
      driven.initialPose.Pos().X() + 0.3);

  // A model that asked for a mode that does not exist yet is a model on
  // Physics too.
  const auto &dynamic = obs["dynamic_requested"];
  EXPECT_NEAR(0.1, dynamic.poses.back().Pos().Z(), 0.05);
}

/////////////////////////////////////////////////
/// The link and its collision get the frame data that other systems read.
TEST_F(SimplifiedPhysicsTest, FrameData)
{
  std::string output;
  auto obs = this->Run(nullptr, output);
  const auto last = static_cast<size_t>(kIterations - 1);
  const double t = kIterations * kDt;

  // Facing +y, driving forward. The body frame sees the forward speed.
  {
    const auto &o = obs["kin_yawed"];
    ExpectNear(math::Vector3d(0, 1, 0), o.linkWorldVelocity[last], 1e-9,
        "world velocity");
    ExpectNear(math::Vector3d(1, 0, 0), o.linkBodyVelocity[last], 1e-9,
        "body velocity");
    ExpectNear(math::Vector3d::Zero, o.linkWorldAngularVelocity[last], 1e-9,
        "angular velocity");
    ExpectNear(o.poses[last].Pos(), o.linkWorldPose[last].Pos(), 1e-9,
        "link pose");
  }

  // On the arc the velocity has turned with the heading.
  {
    const auto &o = obs["kin_arc"];
    ExpectNear(math::Vector3d(std::cos(t), std::sin(t), 0),
        o.linkWorldVelocity[last], 1e-6, "arc velocity");
    ExpectNear(math::Vector3d(1, 0, 0), o.linkBodyVelocity[last], 1e-6,
        "arc body velocity");
    ExpectNear(math::Vector3d(0, 0, 1), o.linkWorldAngularVelocity[last],
        1e-9, "arc angular velocity");
  }

  // Turning in place. The link at the model origin stands still, and the
  // collision 0.5 m ahead goes around it at 1 m/s and accelerates toward
  // the center at w^2 r = 2 m/s^2.
  {
    const auto &o = obs["kin_spin"];
    ExpectNear(math::Vector3d::Zero, o.linkWorldVelocity[last], 1e-9,
        "spin link velocity");
    ExpectNear(math::Vector3d(0, 0, 2), o.linkWorldAngularVelocity[last],
        1e-9, "spin angular velocity");

    const auto &pose = o.collisionWorldPose[last];
    const double yaw = 2.0 * t;
    ExpectNear(o.initialPose.Pos() +
        math::Vector3d(0.5 * std::cos(yaw), 0.5 * std::sin(yaw), 0.0),
        pose.Pos(), 1e-6, "collision position");
    ExpectNear(math::Vector3d(-std::sin(yaw), std::cos(yaw), 0),
        o.collisionWorldVelocity[last], 1e-6, "collision velocity");
    // The collision frame is turned with the body, so the center of the
    // turn is along the collision's -x axis.
    ExpectNear(math::Vector3d(-2, 0, 0), o.collisionBodyAcceleration[last],
        1e-6, "collision acceleration");
  }

  // A model without a command has zero velocity.
  {
    const auto &o = obs["kin_idle"];
    ExpectNear(math::Vector3d::Zero, o.linkWorldVelocity[last], 1e-12,
        "idle velocity");
    ExpectNear(o.initialPose.Pos(), o.linkWorldPose[last].Pos(), 1e-12,
        "idle link pose");
    ExpectNear(o.initialPose.Pos(), o.collisionWorldPose[last].Pos(), 1e-12,
        "idle collision pose");
  }
}

/////////////////////////////////////////////////
/// A step without a command is a step without motion, and the frame data
/// reads zero velocity again.
TEST_F(SimplifiedPhysicsTest, StopsWithoutCommand)
{
  std::string output;
  auto obs = this->Run(
      [](EntityComponentManager &_ecm,
          std::map<std::string, Observation> &_obs, int _step)
      {
        if (_step <= kCommandedUntil)
        {
          _ecm.SetComponentData<components::LinearVelocityCmd>(
              _obs["kin_teleport"].model, {0.5, 0, 0});
        }
      }, output);

  const auto &o = obs["kin_teleport"];
  const auto indexOf = [](int _step) { return static_cast<size_t>(_step - 1); };
  EXPECT_NEAR(o.initialPose.Pos().X() + 0.5 * kCommandedUntil * kDt,
      o.poses.back().Pos().X(), 1e-6);

  // Moved at the last commanded step, and did not move at the next one.
  EXPECT_NEAR(0.5, o.linkWorldVelocity[indexOf(kCommandedUntil)].X(), 1e-9);
  EXPECT_NEAR(0.0, o.linkWorldVelocity[indexOf(kCommandedUntil + 1)].X(),
      1e-12);
  EXPECT_NEAR(o.poses[indexOf(kCommandedUntil)].Pos().X(),
      o.poses[indexOf(kCommandedUntil + 1)].Pos().X(), 1e-12);
  EXPECT_EQ(ComponentState::PeriodicChange,
      o.poseStates[indexOf(kCommandedUntil)]);
  EXPECT_EQ(ComponentState::NoChange,
      o.poseStates[indexOf(kCommandedUntil + 1)]);
}

/////////////////////////////////////////////////
/// A WorldPoseCmd teleports the model once, even though the component stays
/// in the ECM for one more step.
TEST_F(SimplifiedPhysicsTest, Teleport)
{
  const math::Pose3d target(2, 3, 4, 0, 0, 1.0);
  const math::Pose3d movingTarget(50, 0, 1, 0, 0, 0);

  // Whether other systems still see the command when the next step starts.
  bool visibleNextStep{false};

  std::string output;
  auto obs = this->Run(
      [&](EntityComponentManager &_ecm,
          std::map<std::string, Observation> &_obs, int _step)
      {
        if (_step == kTeleportStep + 1)
        {
          visibleNextStep = _ecm.Component<components::WorldPoseCmd>(
              _obs["kin_teleport"].model) != nullptr;
        }
        if (_step == kTeleportStep)
        {
          _ecm.SetComponentData<components::WorldPoseCmd>(
              _obs["kin_teleport"].model, target);
          _ecm.SetComponentData<components::WorldPoseCmd>(
              _obs["kin_teleport_moving"].model, movingTarget);
        }
      }, output);

  const auto indexOf = [](int _step) { return static_cast<size_t>(_step - 1); };

  // The command exists after the step that applies it and at the start of
  // the next step, so that other systems can react to it. It is removed in
  // the update of the next step.
  const auto &still = obs["kin_teleport"];
  EXPECT_TRUE(still.worldPoseCmdPresent[indexOf(kTeleportStep)]);
  EXPECT_TRUE(visibleNextStep);
  EXPECT_FALSE(still.worldPoseCmdPresent[indexOf(kTeleportStep + 1)]);
  EXPECT_FALSE(still.worldPoseCmdPresent.back());

  // The model before the teleport, in the step of it, and long after it.
  EXPECT_EQ(still.initialPose, still.poses[indexOf(kTeleportStep - 1)]);
  for (int step : {kTeleportStep, kTeleportStep + 1, kTeleportStep + 2,
      kIterations})
  {
    const auto &pose = still.poses[indexOf(step)];
    ExpectNear(target.Pos(), pose.Pos(), 1e-9, "teleported position");
    EXPECT_NEAR(target.Rot().Yaw(), pose.Rot().Yaw(), 1e-9);
  }
  EXPECT_EQ(ComponentState::OneTimeChange,
      still.poseStates[indexOf(kTeleportStep)]);
  EXPECT_EQ(ComponentState::NoChange,
      still.poseStates[indexOf(kTeleportStep + 1)]);

  // The frame data follows the teleport.
  ExpectNear(target.Pos(), still.linkWorldPose.back().Pos(), 1e-9,
      "teleported link pose");

  // A moving model continues from the new pose. The teleport step includes
  // that step's motion, and the repeat in the next step must not undo it.
  const auto &moving = obs["kin_teleport_moving"];
  for (int step : {kTeleportStep, kTeleportStep + 1, kTeleportStep + 2,
      kIterations})
  {
    const auto &pose = moving.poses[indexOf(step)];
    EXPECT_NEAR(movingTarget.Pos().X() + (step - kTeleportStep + 1) * kDt,
        pose.Pos().X(), 1e-9) << step;
    EXPECT_NEAR(movingTarget.Pos().Y(), pose.Pos().Y(), 1e-9) << step;
  }
  EXPECT_FALSE(moving.worldPoseCmdPresent.back());
}

/////////////////////////////////////////////////
/// A plugin that needs a feature that the mode does not serve is not run,
/// and the warning names the plugin and the model (rule 7).
TEST_F(SimplifiedPhysicsTest, UnmetRequirementsAreNotRun)
{
  std::string output;
  auto obs = this->Run(nullptr, output);

  // The annotated VelocityControl on the planar model does not run, so the
  // model does not move. The model keeps its mode: Physics still leaves it
  // alone, and the model does not fall.
  const auto &gated = obs["kin_gated"];
  EXPECT_EQ(gated.initialPose, gated.poses.back());
  EXPECT_NEAR(1.0, gated.poses.back().Pos().Z(), 1e-9);

  EXPECT_NE(std::string::npos,
      output.find("System [gz::sim::systems::VelocityControl] on model "
                  "[kin_gated] needs physics feature [joint_force]"))
      << output;

  // The same through the plugin interface.
  const auto &testPlugin = obs["kin_gated_test_plugin"];
  EXPECT_NEAR(1.0, testPlugin.poses.back().Pos().Z(), 1e-9);
  EXPECT_NE(std::string::npos,
      output.find("System [gz::sim::TestPhysicsRequirementsSystem] on model "
                  "[kin_gated_test_plugin] needs physics features "
                  "[feature_a], [feature_b]"))
      << output;

  // The model on Physics has the same annotation and keeps its plugin: no
  // warning, and the model moves.
  EXPECT_EQ(std::string::npos, output.find("model [full_driven]")) << output;
  EXPECT_GT(obs["full_driven"].poses.back().Pos().X(),
      obs["full_driven"].initialPose.Pos().X() + 0.3);

  // The plugins of the models that do run are not warned about.
  EXPECT_EQ(std::string::npos, output.find("model [kin_line]")) << output;
}

/////////////////////////////////////////////////
/// A world reset puts the models back, and they move again afterwards.
TEST_F(SimplifiedPhysicsTest, Reset)
{
  ::testing::internal::CaptureStderr();
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "simplified_physics.sdf"));

  Entity model{kNullEntity};
  math::Pose3d initialPose;
  std::vector<double> x;
  std::vector<double> z;
  fixture.OnConfigure([&](const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &_ecm, EventManager &)
      {
        model = _ecm.EntityByComponents(
            components::Model(), components::Name("kin_line"));
        initialPose = worldPose(model, _ecm);
      })
  .OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        x.push_back(worldPose(model, _ecm).Pos().X());
        z.push_back(worldPose(model, _ecm).Pos().Z());
      }).Finalize();

  constexpr int kBefore{200};
  constexpr int kAfter{100};
  fixture.Server()->Run(true, kBefore, false);
  ASSERT_EQ(static_cast<size_t>(kBefore), x.size());
  EXPECT_NEAR(x.back(), initialPose.Pos().X() + kBefore * kDt, 1e-6);

  msgs::WorldControl req;
  msgs::Boolean rep;
  req.mutable_reset()->set_all(true);
  transport::Node node;
  bool result{false};
  ASSERT_TRUE(node.Request("/world/simplified_physics_test/control", req, 1000,
      rep, result));
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());

  fixture.Server()->Run(true, kAfter, false);
  ::testing::internal::GetCapturedStderr();

  // The model went back to its start, and it is still owned by the provider:
  // it moves and does not fall.
  const double minimum = *std::min_element(x.begin() + kBefore, x.end());
  EXPECT_NEAR(initialPose.Pos().X(), minimum, 3 * kDt);
  EXPECT_GT(x.back(), minimum + 0.05);
  EXPECT_NEAR(1.0, z.back(), 1e-9);
}

/////////////////////////////////////////////////
/// The mode can be chosen where a model is included, with
/// <experimental:params>, and one model file serves both modes.
TEST_F(SimplifiedPhysicsTest, ChosenAtInclude)
{
  common::setenv("GZ_SIM_RESOURCE_PATH", common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "models").c_str());

  ::testing::internal::CaptureStderr();
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "simplified_physics_include.sdf"));

  std::map<std::string, math::Pose3d> last;
  std::map<std::string, math::Pose3d> first;
  std::map<std::string, std::string> owners;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        for (const auto &name : {"amr_full", "amr_planar"})
        {
          const auto model = _ecm.EntityByComponents(
              components::Model(), components::Name(name));
          ASSERT_NE(kNullEntity, model) << name;
          last[name] = worldPose(model, _ecm);
          if (first.count(name) == 0)
            first[name] = last[name];
          if (auto owner = _ecm.Component<components::PhysicsOwner>(model))
            owners[name] = owner->Data();
        }
      }).Finalize();
  fixture.Server()->Run(true, kIterations, false);
  ::testing::internal::GetCapturedStderr();

  EXPECT_EQ(0u, owners.count("amr_full"));
  EXPECT_EQ(kProvider, owners["amr_planar"]);

  // The planar model drives and does not fall. Physics simulates the other
  // one, so gravity pulls it down. VelocityControl sets its vertical speed to
  // zero in every step, so it sinks slowly instead of falling.
  EXPECT_NEAR(1.0, last["amr_planar"].Pos().Z(), 1e-9);
  EXPECT_NEAR(kIterations * kDt - kDt + first["amr_planar"].Pos().X(),
      last["amr_planar"].Pos().X(), 1e-6);
  EXPECT_LT(last["amr_full"].Pos().Z(), 0.999);
}

/////////////////////////////////////////////////
/// The example world loads and its robots move on both providers.
TEST_F(SimplifiedPhysicsTest, ExampleWorld)
{
  ::testing::internal::CaptureStderr();
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "examples", "worlds", "simplified_physics.sdf"));

  const std::vector<std::string> names{"amr_full", "amr_planar_1",
      "amr_planar_2", "amr_planar_3"};
  std::map<std::string, math::Pose3d> first;
  std::map<std::string, math::Pose3d> last;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        for (const auto &name : names)
        {
          const auto model = _ecm.EntityByComponents(
              components::Model(), components::Name(name));
          ASSERT_NE(kNullEntity, model) << name;
          last[name] = worldPose(model, _ecm);
          if (first.count(name) == 0)
            first[name] = last[name];
        }
      }).Finalize();
  fixture.Server()->Run(true, 500, false);
  const auto output = ::testing::internal::GetCapturedStderr();

  for (const auto &name : names)
  {
    ASSERT_EQ(1u, first.count(name)) << name;
    EXPECT_GT(last[name].Pos().Distance(first[name].Pos()) +
        std::abs(last[name].Rot().Yaw() - first[name].Rot().Yaw()), 0.1)
        << name;
  }
  EXPECT_EQ(std::string::npos, output.find("[error]")) << output;
  EXPECT_EQ(std::string::npos, output.find("[warn]")) << output;
}

/////////////////////////////////////////////////
/// The world path of the test worlds.
static std::string WorldPath(const std::string &_name)
{
  return common::joinPaths(std::string(PROJECT_SOURCE_PATH), "test", "worlds",
      _name);
}

/////////////////////////////////////////////////
/// Velocity commands turn about the canonical link, like they do on Physics.
/// The canonical link of both models is off the model origin and turned. In
/// half a turn at 1 m/s and 1 rad/s the link goes around a circle of radius
/// 1 m, so a pivot at the model origin would put the link a whole meter off.
///
/// The reference is the closed form. Physics integrates the same motion in
/// 1 ms steps with the velocity held constant over each step, so its circle
/// grows by (w dt)^2 / 2 per step, 1.6 mm after half a turn. The tolerance for
/// the comparison with Physics is 5 mm in position and 5 mrad in heading,
/// three times that error.
TEST_F(SimplifiedPhysicsTest, VelocityCommandsTurnAboutCanonicalLink)
{
  ::testing::internal::CaptureStderr();
  TestFixture fixture(WorldPath("simplified_physics_pivot.sdf"));

  struct Track
  {
    Entity model{kNullEntity};
    Entity link{kNullEntity};
    math::Pose3d link0;
    math::Pose3d link1;
  };
  std::map<std::string, Track> tracks{{"full_offset", {}}, {"kin_offset", {}}};
  bool first{true};
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        for (auto &[name, track] : tracks)
        {
          if (track.model == kNullEntity)
          {
            track.model = _ecm.EntityByComponents(
                components::Model(), components::Name(name));
            track.link = Model(track.model).LinkByName(_ecm, "link");
          }
          track.link1 = worldPose(track.link, _ecm);
          if (first)
            track.link0 = track.link1;
        }
        first = false;
      }).Finalize();

  // The first step already moves the models by one step.
  const int steps = static_cast<int>(GZ_PI / kDt);
  fixture.Server()->Run(true, steps, false);
  ::testing::internal::GetCapturedStderr();

  // The model frame turns by 0.3 rad, and the command is in the model frame.
  const double t = steps * kDt;
  const double yaw0 = 0.3;
  const auto &full = tracks["full_offset"];
  const auto &kin = tracks["kin_offset"];

  // Closed form for the link: it drives along its heading at 1 m/s while
  // the heading turns at 1 rad/s.
  const math::Vector3d expected(
      std::sin(yaw0 + t) - std::sin(yaw0),
      std::cos(yaw0) - std::cos(yaw0 + t), 0.0);
  ExpectNear(expected, kin.link1.Pos() - kin.link0.Pos(), 2e-3,
      "planar_kinematic link displacement");
  ExpectNear(expected, full.link1.Pos() - full.link0.Pos(), 5e-3,
      "Physics link displacement");
  const auto turned = [](const Track &_track)
  {
    return math::Angle(
        _track.link1.Rot().Yaw() - _track.link0.Rot().Yaw()).Normalized()
        .Radian();
  };
  const double wanted = math::Angle(t).Normalized().Radian();

  // The same path on both providers.
  ExpectNear(full.link1.Pos() - full.link0.Pos(),
      kin.link1.Pos() - kin.link0.Pos(), 5e-3, "kinematic against Physics");
  EXPECT_NEAR(wanted, turned(kin), 2e-3);
  EXPECT_NEAR(turned(full), turned(kin), 5e-3);

  // The link stays at its height.
  EXPECT_NEAR(kin.link0.Pos().Z(), kin.link1.Pos().Z(), 1e-9);
}

/////////////////////////////////////////////////
/// A world PhysicsOwner that names SimplifiedPhysics must not leave a model
/// without a simulator. The static ground goes back to Physics, so a box on
/// it rests there, and the rule-7 gate sees the features of the models that
/// the default gave to SimplifiedPhysics.
TEST_F(SimplifiedPhysicsTest, WorldDefaultOwner)
{
  ::testing::internal::CaptureStderr();
  TestFixture fixture(WorldPath("simplified_physics_default.sdf"));

  struct Seen
  {
    bool hasOwner{false};
    std::string owner;
    bool hasFeatures{false};
    math::Pose3d first;
    math::Pose3d last;
    bool any{false};
  };
  std::map<std::string, Seen> seen;
  const std::vector<std::string> names{"ground", "full_box", "default_robot",
      "default_gated", "default_double"};
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        for (const auto &name : names)
        {
          const auto model = _ecm.EntityByComponents(
              components::Model(), components::Name(name));
          auto &o = seen[name];
          if (const auto owner =
              _ecm.Component<components::PhysicsOwner>(model))
          {
            o.hasOwner = true;
            o.owner = owner->Data();
          }
          o.hasFeatures = _ecm.Component<
              components::SupportedPhysicsFeatures>(model) != nullptr;
          o.last = worldPose(model, _ecm);
          if (!o.any)
            o.first = o.last;
          o.any = true;
        }
      }).Finalize();
  fixture.Server()->Run(true, kIterations, false);
  const auto output = ::testing::internal::GetCapturedStderr();

  // The static ground has no use for the planar mode. It has an empty owner
  // on the model, which takes precedence over the world's, and no features.
  EXPECT_TRUE(seen["ground"].hasOwner);
  EXPECT_EQ("", seen["ground"].owner);
  EXPECT_FALSE(seen["ground"].hasFeatures);

  // The box on Physics falls onto the ground and stops. Without the hand
  // back, the ground had no simulator and the box fell through it.
  EXPECT_NEAR(0.1, seen["full_box"].last.Pos().Z(), 0.02);
  EXPECT_TRUE(seen["full_box"].hasOwner);
  EXPECT_EQ("", seen["full_box"].owner);
  EXPECT_FALSE(seen["full_box"].hasFeatures);

  // The robot with the default owner drives and does not fall.
  EXPECT_FALSE(seen["default_robot"].hasOwner);
  EXPECT_TRUE(seen["default_robot"].hasFeatures);
  EXPECT_NEAR(seen["default_robot"].first.Pos().X() + 0.999,
      seen["default_robot"].last.Pos().X(), 1e-6);
  EXPECT_NEAR(1.0, seen["default_robot"].last.Pos().Z(), 1e-9);

  // A plugin that needs a feature the planar mode does not serve is not
  // run on a model that got the mode from the default (rule 7).
  EXPECT_TRUE(seen["default_gated"].hasFeatures);
  EXPECT_EQ(seen["default_gated"].first, seen["default_gated"].last);
  EXPECT_NE(std::string::npos,
      output.find("System [gz::sim::systems::VelocityControl] on model "
                  "[default_gated] needs physics feature [joint_force]"))
      << output;
  EXPECT_EQ(std::string::npos, output.find("model [default_robot]"))
      << output;

  // The instance that sits in a model says so and does nothing.
  EXPECT_NE(std::string::npos,
      output.find("SimplifiedPhysics must be attached to a world"))
      << output;
  EXPECT_NEAR(seen["default_double"].first.Pos().X() + 0.999,
      seen["default_double"].last.Pos().X(), 1e-6);
}

/////////////////////////////////////////////////
/// A second instance in a model must not apply a teleport again, which
/// would undo the motion of the step.
TEST_F(SimplifiedPhysicsTest, InstanceInModelDoesNotRepeatTeleport)
{
  ::testing::internal::CaptureStderr();
  TestFixture fixture(WorldPath("simplified_physics_default.sdf"));

  const math::Pose3d target(20, 6, 1, 0, 0, 0);
  std::vector<double> x;
  Entity model{kNullEntity};
  int step{0};
  fixture.OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        ++step;
        if (model == kNullEntity)
        {
          model = _ecm.EntityByComponents(
              components::Model(), components::Name("default_double"));
        }
        if (step == 10)
          _ecm.SetComponentData<components::WorldPoseCmd>(model, target);
      })
  .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        x.push_back(worldPose(model, _ecm).Pos().X());
      }).Finalize();
  fixture.Server()->Run(true, 20, false);
  ::testing::internal::GetCapturedStderr();

  // The step that teleports also moves the model by one step. The next step
  // moves it again, from where it is.
  ASSERT_EQ(20u, x.size());
  EXPECT_NEAR(target.Pos().X() + kDt, x[9], 1e-9);
  EXPECT_NEAR(target.Pos().X() + 2 * kDt, x[10], 1e-9);
  EXPECT_NEAR(target.Pos().X() + 3 * kDt, x[11], 1e-9);
}

/////////////////////////////////////////////////
/// A system that sends a different WorldPoseCmd in every step has each one
/// applied. A command that repeats its value cannot be told from the one the
/// system has not removed yet, so it is applied in every other step. That is
/// documented in the class description.
TEST_F(SimplifiedPhysicsTest, WorldPoseCmdSentEveryStep)
{
  std::string output;
  auto obs = this->Run(
      [](EntityComponentManager &_ecm,
          std::map<std::string, Observation> &_obs, int _step)
      {
        // This model drives along x at 1 m/s.
        _ecm.SetComponentData<components::WorldPoseCmd>(
            _obs["kin_teleport_moving"].model,
            math::Pose3d(50 + 0.01 * _step, 0, 1, 0, 0, 0));
        // This one has no velocity command. The same value every step.
        _ecm.SetComponentData<components::WorldPoseCmd>(
            _obs["kin_teleport"].model, math::Pose3d(30, 0, 1, 0, 0, 0));
      }, output);

  const auto &moving = obs["kin_teleport_moving"];
  for (int step = 1; step <= kIterations; ++step)
  {
    EXPECT_NEAR(50 + 0.01 * step + kDt,
        moving.poses[static_cast<size_t>(step - 1)].Pos().X(), 1e-9) << step;
  }

  // The repeated value is applied at least every other step, so the model
  // never leaves it.
  const auto &still = obs["kin_teleport"];
  for (const auto &pose : still.poses)
    EXPECT_NEAR(30.0, pose.Pos().X(), 1e-9);
}

/////////////////////////////////////////////////
/// A teleport takes effect while the simulation is paused.
TEST_F(SimplifiedPhysicsTest, TeleportWhilePaused)
{
  ::testing::internal::CaptureStderr();
  TestFixture fixture(WorldPath("simplified_physics.sdf"));

  const math::Pose3d target(7, 8, 3, 0, 0, 0.5);
  Entity moving{kNullEntity};
  int step{0};
  std::vector<math::Pose3d> poses;
  fixture.OnPreUpdate([&](const UpdateInfo &_info, EntityComponentManager &_ecm)
      {
        ++step;
        EXPECT_TRUE(_info.paused);
        if (moving == kNullEntity)
        {
          moving = _ecm.EntityByComponents(
              components::Model(), components::Name("kin_teleport_moving"));
        }
        if (step == 3)
          _ecm.SetComponentData<components::WorldPoseCmd>(moving, target);
      })
  .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        poses.push_back(worldPose(moving, _ecm));
      }).Finalize();
  for (int i = 0; i < 6; ++i)
    fixture.Server()->RunOnce(true);
  ::testing::internal::GetCapturedStderr();

  // The model has a velocity command, but a paused world does not move it.
  ASSERT_EQ(6u, poses.size());
  EXPECT_EQ(poses[0], poses[1]);
  ExpectNear(target.Pos(), poses[2].Pos(), 1e-9, "teleport while paused");
  ExpectNear(target.Pos(), poses[5].Pos(), 1e-9, "after the teleport");
  EXPECT_NEAR(target.Rot().Yaw(), poses[5].Rot().Yaw(), 1e-9);
}

/////////////////////////////////////////////////
/// Link velocity commands are not followed. The system clears them, warns
/// once for the model, and Physics does not clear them for it.
TEST_F(SimplifiedPhysicsTest, UnsupportedCommandsWarnOnce)
{
  ::testing::internal::CaptureStderr();
  TestFixture fixture(WorldPath("simplified_physics.sdf"));

  Entity link{kNullEntity};
  Entity model{kNullEntity};
  int step{0};
  int linkCmdMoved{0};
  math::Pose3d first, last;
  fixture.OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        ++step;
        if (model == kNullEntity)
        {
          model = _ecm.EntityByComponents(
              components::Model(), components::Name("kin_idle"));
          link = Model(model).LinkByName(_ecm, "link");
          first = worldPose(model, _ecm);
        }
        if (step >= 3 && step <= 6)
        {
          _ecm.SetComponentData<components::LinearVelocityCmd>(link,
              {1.0, 0, 0});
        }
      })
  .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        auto linkCmd = _ecm.Component<components::LinearVelocityCmd>(link);
        if (linkCmd && linkCmd->Data() != math::Vector3d::Zero)
          ++linkCmdMoved;
        last = worldPose(model, _ecm);
      }).Finalize();
  fixture.Server()->Run(true, 10, false);
  const auto output = ::testing::internal::GetCapturedStderr();

  EXPECT_EQ(0, linkCmdMoved);
  EXPECT_EQ(first, last);

  const std::string text{"Model [kin_idle] gets a link velocity command"};
  const auto pos = output.find(text);
  ASSERT_NE(std::string::npos, pos) << output;
  EXPECT_EQ(std::string::npos, output.find(text, pos + 1)) << output;
}

/////////////////////////////////////////////////
/// A model spawned while the simulation runs is simulated, and removing it
/// leaves the system working.
TEST_F(SimplifiedPhysicsTest, SpawnAndRemoveAtRuntime)
{
  ::testing::internal::CaptureStderr();
  TestFixture fixture(WorldPath("simplified_physics_default.sdf"));

  math::Pose3d first, last;
  bool seen{false};
  int missingAfterSeen{0};
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        const auto model = _ecm.EntityByComponents(
            components::Model(), components::Name("spawned"));
        if (model == kNullEntity)
        {
          if (seen)
            ++missingAfterSeen;
          return;
        }
        last = worldPose(model, _ecm);
        if (!seen)
          first = last;
        seen = true;
      }).Finalize();
  fixture.Server()->Run(true, 5, false);

  const std::string sdfString = R"(<?xml version="1.0" ?>
    <sdf version="1.9">
      <model name="spawned">
        <pose>3 30 1 0 0 0</pose>
        <link name="link">
          <collision name="c">
            <geometry><box><size>1 1 1</size></box></geometry>
          </collision>
        </link>
        <plugin filename="gz-sim-velocity-control-system"
                name="gz::sim::systems::VelocityControl">
          <initial_linear>1 0 0</initial_linear>
        </plugin>
        <plugin filename="gz-sim-simplified-physics-system"
                name="gz::sim::systems::SimplifiedPhysicsModel">
          <mode>planar_kinematic</mode>
        </plugin>
      </model>
    </sdf>)";
  transport::Node node;
  msgs::EntityFactory createReq;
  createReq.set_sdf(sdfString);
  msgs::Boolean rep;
  bool result{false};
  ASSERT_TRUE(node.Request("/world/simplified_physics_default_test/create",
      createReq, 2000, rep, result));
  ASSERT_TRUE(result);

  fixture.Server()->Run(true, 501, false);
  ASSERT_TRUE(seen);
  EXPECT_NEAR(0.5, last.Pos().X() - first.Pos().X(), 0.01);
  EXPECT_NEAR(1.0, last.Pos().Z(), 1e-9);

  msgs::Entity removeReq;
  removeReq.set_name("spawned");
  removeReq.set_type(msgs::Entity::MODEL);
  ASSERT_TRUE(node.Request("/world/simplified_physics_default_test/remove",
      removeReq, 2000, rep, result));
  ASSERT_TRUE(result);
  fixture.Server()->Run(true, 50, false);
  EXPECT_GT(missingAfterSeen, 0);

  // The only error is the one that the world's misplaced instance prints.
  const auto output = ::testing::internal::GetCapturedStderr();
  const auto firstError = output.find("[error]");
  ASSERT_NE(std::string::npos, firstError) << output;
  EXPECT_EQ(std::string::npos, output.find("[error]", firstError + 1))
      << output;
}

/////////////////////////////////////////////////
/// Both plugins load from one library, and the system lists the interfaces
/// that it needs. A priority interface that is left out of the plugin
/// registration is silently ignored, so the test looks for it.
TEST_F(SimplifiedPhysicsTest, LoadPlugins)
{
  SystemLoader loader;

  sdf::Plugin worldPlugin;
  worldPlugin.SetFilename("gz-sim-simplified-physics-system");
  worldPlugin.SetName("gz::sim::systems::SimplifiedPhysics");
  auto system = loader.LoadPlugin(worldPlugin);
  ASSERT_TRUE(system.has_value());
  ASSERT_NE(nullptr, (*system)->QueryInterface<System>());
  EXPECT_NE(nullptr, (*system)->QueryInterface<ISystemConfigure>());
  EXPECT_NE(nullptr, (*system)->QueryInterface<ISystemReset>());
  EXPECT_NE(nullptr, (*system)->QueryInterface<ISystemUpdate>());
  auto priority = (*system)->QueryInterface<ISystemConfigurePriority>();
  ASSERT_NE(nullptr, priority);
  EXPECT_EQ(systems::kPrePhysicsPriority, priority->ConfigurePriority());
  EXPECT_LT(priority->ConfigurePriority(), systems::kPhysicsPriority);

  sdf::Plugin modelPlugin;
  modelPlugin.SetFilename("gz-sim-simplified-physics-system");
  modelPlugin.SetName("gz::sim::systems::SimplifiedPhysicsModel");
  auto model = loader.LoadPlugin(modelPlugin);
  ASSERT_TRUE(model.has_value());
  EXPECT_NE(nullptr, (*model)->QueryInterface<ISystemConfigure>());
  EXPECT_EQ(nullptr, (*model)->QueryInterface<ISystemUpdate>());
}

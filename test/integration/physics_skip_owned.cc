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

#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Root.hh>

#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Joint.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/AngularVelocityReset.hh"
#include "gz/sim/components/AxisAlignedBox.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/CollisionBitmask.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityReset.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/PhysicsOwner.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "gz/sim/components/SlipComplianceCmd.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/World.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/ResetUtils.hh"

using namespace gz;
using namespace sim;

namespace
{
const char *kPhysics = "gz::sim::systems::Physics";
const char *kOther = "gz::sim::systems::Other";

/// \brief Id that marks a contact the test wrote, so it can be told apart from
/// a contact the Physics system wrote.
constexpr int kFakeContactId{999};

constexpr int kIterations{600};

/// \brief What the test saw for one model.
struct Observation
{
  /// \brief Number of updates in which the model existed.
  int updates{0};

  /// \brief Number of updates in which the model's joint command was still
  /// there after Physics ran.
  int jointCmdKept{0};

  /// \brief Number of updates in which the model's link wrench was still
  /// there after Physics ran.
  int wrenchKept{0};

  /// \brief Last pose of the model.
  math::Pose3d pose;

  /// \brief Initial pose of the model.
  math::Pose3d initialPose;

  /// \brief Last JointPosition and JointVelocity, if the model has a joint.
  std::vector<double> jointPosition;
  std::vector<double> jointVelocity;

  /// \brief True if a contact that the test did not write ever appeared.
  bool sawEngineContact{false};

  /// \brief True if the contact that the test wrote is still there.
  bool fakeContactKept{false};

  /// \brief Number of updates in which every extra command that the test
  /// added to the model and its children was still there after Physics ran.
  int extraCmdsKept{0};

  /// \brief True if a bounding box was ever written for the link.
  bool linkBoxWritten{false};

  /// \brief True if a bounding box was ever written for the model.
  bool modelBoxWritten{false};
};

/// \brief Entities of one test model.
struct ModelEntities
{
  Entity model{kNullEntity};
  Entity link{kNullEntity};
  Entity joint{kNullEntity};
  Entity collision{kNullEntity};
};

/// \brief Wrench that the test applies to every link in every update.
msgs::Wrench TestWrench()
{
  msgs::Wrench wrench;
  wrench.mutable_force()->set_x(5.0);
  return wrench;
}

/// \brief True if the model holds a box, as opposed to an arm.
bool IsBox(const std::string &_name)
{
  return _name.find("box") != std::string::npos;
}

/// \brief Create a component on an entity unless it has one.
template<typename ComponentT>
void EnsureComponent(EntityComponentManager &_ecm, const Entity &_entity,
    const typename ComponentT::Type &_data)
{
  if (!_ecm.EntityHasComponentType(_entity, ComponentT::typeId))
    _ecm.CreateComponent(_entity, ComponentT(_data));
}

/// \brief Add the components that a test reads. Other systems create these
/// when they need the data. Create them so that the Physics system would write
/// them. The call does nothing for a component that exists, so the test can
/// repeat it after a world reset removed them.
void AddObservedComponents(EntityComponentManager &_ecm,
    const ModelEntities &_e)
{
  if (_e.joint != kNullEntity)
  {
    EnsureComponent<components::JointPosition>(_ecm, _e.joint, {});
    EnsureComponent<components::JointVelocity>(_ecm, _e.joint, {});
  }
  else if (!_ecm.EntityHasComponentType(_e.collision,
        components::ContactSensorData::typeId))
  {
    // A contact that the Physics system would erase if it handled the
    // collision, because the engine reports no contact for it.
    msgs::Contacts contacts;
    contacts.add_contact()->mutable_collision1()->set_id(kFakeContactId);
    _ecm.CreateComponent(_e.collision,
        components::ContactSensorData(contacts));
  }

  // Ask for bounding boxes. Physics writes them for the models it simulates.
  EnsureComponent<components::AxisAlignedBox>(_ecm, _e.link, {});
  EnsureComponent<components::AxisAlignedBox>(_ecm, _e.model, {});
}

/// \brief Add commands that only the owner of an entity should consume.
/// Physics must leave all of them in place.
void AddExtraCommands(EntityComponentManager &_ecm, const ModelEntities &_e)
{
  // Would teleport and freeze the model, and switch off gravity and contacts.
  EnsureComponent<components::WorldPoseCmd>(_ecm, _e.model,
      math::Pose3d(10, 10, 10, 0, 0, 0));
  EnsureComponent<components::StaticCmd>(_ecm, _e.model, true);
  EnsureComponent<components::GravityEnabledCmd>(_ecm, _e.model, false);
  EnsureComponent<components::CollisionEnabledCmd>(_ecm, _e.model, false);

  EnsureComponent<components::GravityEnabledCmd>(_ecm, _e.link, false);
  EnsureComponent<components::AngularVelocityCmd>(_ecm, _e.link,
      math::Vector3d(0, 0, 1));
  EnsureComponent<components::LinearVelocityCmd>(_ecm, _e.link,
      math::Vector3d(1, 0, 0));
  EnsureComponent<components::WorldAngularVelocityReset>(_ecm, _e.link,
      math::Vector3d(0, 0, 1));
  EnsureComponent<components::WorldLinearVelocityReset>(_ecm, _e.link,
      math::Vector3d(1, 0, 0));

  EnsureComponent<components::SlipComplianceCmd>(_ecm, _e.collision,
      std::vector<double>{0.1, 0.1});
  EnsureComponent<components::EnableContactSurfaceCustomization>(_ecm,
      _e.collision, true);
  EnsureComponent<components::CollideBitmaskCmd>(_ecm, _e.collision, 0x1);
  EnsureComponent<components::CategoryBitmaskCmd>(_ecm, _e.collision, 0x1);
}

/// \brief Check that every command of AddExtraCommands is still there.
bool ExtraCommandsPresent(const EntityComponentManager &_ecm,
    const ModelEntities &_e)
{
  return
      _ecm.EntityHasComponentType(_e.model,
          components::WorldPoseCmd::typeId) &&
      _ecm.EntityHasComponentType(_e.model, components::StaticCmd::typeId) &&
      _ecm.EntityHasComponentType(_e.model,
          components::GravityEnabledCmd::typeId) &&
      _ecm.EntityHasComponentType(_e.model,
          components::CollisionEnabledCmd::typeId) &&
      _ecm.EntityHasComponentType(_e.link,
          components::GravityEnabledCmd::typeId) &&
      _ecm.EntityHasComponentType(_e.link,
          components::AngularVelocityCmd::typeId) &&
      _ecm.EntityHasComponentType(_e.link,
          components::LinearVelocityCmd::typeId) &&
      _ecm.EntityHasComponentType(_e.link,
          components::WorldAngularVelocityReset::typeId) &&
      _ecm.EntityHasComponentType(_e.link,
          components::WorldLinearVelocityReset::typeId) &&
      _ecm.EntityHasComponentType(_e.collision,
          components::SlipComplianceCmd::typeId) &&
      _ecm.EntityHasComponentType(_e.collision,
          components::EnableContactSurfaceCustomization::typeId) &&
      _ecm.EntityHasComponentType(_e.collision,
          components::CollideBitmaskCmd::typeId) &&
      _ecm.EntityHasComponentType(_e.collision,
          components::CategoryBitmaskCmd::typeId);
}

/// \brief Test fixture.
class PhysicsSkipOwnedTest : public InternalFixture<::testing::Test>
{
  /// \brief Run the world and record what happens to each model.
  /// \param[in] _setOwners Called in Configure, with the model entities by
  /// name and the world entity, to set the PhysicsOwner components.
  /// \param[in] _preUpdate Optional hook that runs in every PreUpdate with
  /// the iteration number.
  /// \param[out] _stderr Text that the server wrote to stderr.
  /// \return Observation of each model, by name.
  protected: std::map<std::string, Observation> Run(
      const std::function<void(EntityComponentManager &,
          const Entity &, std::map<std::string, ModelEntities> &)> &_setOwners,
      const std::function<void(EntityComponentManager &,
          std::map<std::string, ModelEntities> &, int)> &_preUpdate,
      std::string &_stderr)
  {
    ::testing::internal::CaptureStderr();

    TestFixture fixture(this->WorldFile());

    std::map<std::string, ModelEntities> entities;
    std::map<std::string, Observation> observations;
    int iteration{0};

    fixture.OnConfigure([&](
        const Entity &_world,
        const std::shared_ptr<const sdf::Element> &,
        EntityComponentManager &_ecm,
        EventManager &)
        {
          for (const auto &name : {"free_box", "owned_box", "free_arm",
                                   "owned_arm", "nested_box"})
          {
            Model model(_ecm.EntityByComponents(
                components::Model(), components::Name(name)));
            ASSERT_TRUE(model.Valid(_ecm)) << name;

            ModelEntities &e = entities[name];
            e.model = model.Entity();
            e.link = model.LinkByName(_ecm, "link");
            e.collision = Link(e.link).CollisionByName(_ecm, "collision");
            e.joint = model.JointByName(_ecm, "spin");

            AddObservedComponents(_ecm, e);

            observations[name].initialPose = worldPose(e.model, _ecm);
          }
          _setOwners(_ecm, _world, entities);
        })
    .OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
        {
          for (auto &[name, e] : entities)
          {
            if (!_ecm.HasEntity(e.model))
              continue;

            AddObservedComponents(_ecm, e);

            if (e.joint != kNullEntity)
            {
              Joint(e.joint).SetVelocity(_ecm, {1.0});
            }

            if (this->extraCmdTargets.count(name) > 0)
              AddExtraCommands(_ecm, e);

            if (!_ecm.SetComponentData<components::ExternalWorldWrenchCmd>(
                  e.link, TestWrench()))
            {
              _ecm.CreateComponent(e.link,
                  components::ExternalWorldWrenchCmd(TestWrench()));
            }
          }

          if (_preUpdate)
            _preUpdate(_ecm, entities, iteration);
        })
    .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
        {
          ++iteration;
          for (auto &[name, e] : entities)
          {
            if (!_ecm.HasEntity(e.model))
              continue;

            auto &obs = observations[name];
            ++obs.updates;
            obs.pose = worldPose(e.model, _ecm);

            if (this->extraCmdTargets.count(name) > 0 &&
                ExtraCommandsPresent(_ecm, e))
            {
              ++obs.extraCmdsKept;
            }

            const math::AxisAlignedBox unset;
            const auto *linkBox =
                _ecm.Component<components::AxisAlignedBox>(e.link);
            if (linkBox && linkBox->Data() != unset)
              obs.linkBoxWritten = true;
            const auto *modelBox =
                _ecm.Component<components::AxisAlignedBox>(e.model);
            if (modelBox && modelBox->Data() != unset)
              obs.modelBoxWritten = true;

            if (e.joint != kNullEntity)
            {
              if (_ecm.Component<components::JointVelocityCmd>(e.joint))
                ++obs.jointCmdKept;
              const auto *position =
                  _ecm.Component<components::JointPosition>(e.joint);
              const auto *velocity =
                  _ecm.Component<components::JointVelocity>(e.joint);
              if (position && velocity)
              {
                obs.jointPosition = position->Data();
                obs.jointVelocity = velocity->Data();
              }
            }

            auto wrench =
                _ecm.Component<components::ExternalWorldWrenchCmd>(e.link);
            if (wrench && wrench->Data().force().x() > 4.0)
              ++obs.wrenchKept;

            auto contacts =
                _ecm.Component<components::ContactSensorData>(e.collision);
            if (contacts)
            {
              obs.fakeContactKept = contacts->Data().contact_size() == 1 &&
                  contacts->Data().contact(0).collision1().id() ==
                  kFakeContactId;
              if (contacts->Data().contact_size() > 0 &&
                  !obs.fakeContactKept)
              {
                obs.sawEngineContact = true;
              }
            }
          }
        })
    .Finalize();

    if (this->paused)
    {
      // Run() does not count paused iterations.
      for (int i = 0; i < this->iterations; ++i)
        fixture.Server()->RunOnce(true);
    }
    else if (this->resetAt > 0)
    {
      fixture.Server()->Run(true, this->resetAt, false);
      test::reset::RequestAndApplyWorldReset(*fixture.Server(),
          "physics_skip_owned");
      fixture.Server()->Run(true, this->iterations - this->resetAt, false);
    }
    else
    {
      fixture.Server()->Run(true, this->iterations, false);
    }

    _stderr = ::testing::internal::GetCapturedStderr();
    return observations;
  }

  /// \brief Path of the world to run.
  protected: std::string WorldFile() const
  {
    return this->worldFile.empty() ?
        common::joinPaths(std::string(PROJECT_SOURCE_PATH), "test", "worlds",
            "physics_skip_owned.sdf") : this->worldFile;
  }

  /// \brief Write a copy of the world in which the owned models carry a
  /// SetPhysicsOwnerSystem plugin. Their PhysicsOwner then exists in the
  /// initial state of the ECM, which a world reset restores.
  protected: void UseWorldWithOwnerPlugins()
  {
    std::ifstream in(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "physics_skip_owned.sdf"));
    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string sdf = buffer.str();

    const std::string plugin = std::string("\n<plugin filename=\"") +
        "SetPhysicsOwnerSystem\" name=\"gz::sim::SetPhysicsOwnerSystem\">"
        "<owner>" + kOther + "</owner></plugin>\n";
    for (const auto &name : {"owned_box", "owned_arm"})
    {
      const std::string tag = std::string("<model name=\"") + name + "\">";
      const auto pos = sdf.find(tag);
      ASSERT_NE(std::string::npos, pos) << name;
      sdf.insert(pos + tag.size(), plugin);
    }

    ASSERT_TRUE(common::createDirectories(this->kFakeHome));
    this->worldFile = common::joinPaths(this->kFakeHome,
        "physics_skip_owned_plugins.sdf");
    std::ofstream out(this->worldFile);
    out << sdf;
  }

  /// \brief World to run, if not the default one.
  protected: std::string worldFile;

  /// \brief Names of the models that get the commands of AddExtraCommands in
  /// every update.
  protected: std::set<std::string> extraCmdTargets;

  /// \brief Number of iterations to run.
  protected: int iterations{kIterations};

  /// \brief Iteration after which to reset the world. Zero for no reset.
  protected: int resetAt{0};

  /// \brief True to run paused. Physics then finds changed links by
  /// visiting every link.
  protected: bool paused{false};

  /// \brief Check that the Physics system simulated a model.
  protected: void ExpectSimulated(const Observation &_obs,
      const std::string &_name)
  {
    SCOPED_TRACE(_name);
    EXPECT_GT(_obs.updates, 0);

    // The commands were consumed.
    EXPECT_EQ(0, _obs.jointCmdKept);
    EXPECT_EQ(0, _obs.wrenchKept);

    // Physics writes the bounding boxes that were asked for.
    EXPECT_TRUE(_obs.linkBoxWritten);
    EXPECT_TRUE(_obs.modelBoxWritten);

    if (IsBox(_name))
    {
      EXPECT_LT(_obs.pose.Z(), _obs.initialPose.Z() - 0.3);
      EXPECT_TRUE(_obs.sawEngineContact);
      EXPECT_FALSE(_obs.fakeContactKept);
    }
    else
    {
      ASSERT_EQ(1u, _obs.jointPosition.size());
      EXPECT_GT(_obs.jointPosition[0], 0.3);
      ASSERT_EQ(1u, _obs.jointVelocity.size());
      EXPECT_NEAR(1.0, _obs.jointVelocity[0], 0.1);
    }
  }

  /// \brief Check that the Physics system left a model alone: it did not move
  /// it, did not write its state, and did not clear its commands.
  protected: void ExpectSkipped(const Observation &_obs,
      const std::string &_name)
  {
    SCOPED_TRACE(_name);
    EXPECT_GT(_obs.updates, 0);

    // The commands are still there for the provider that owns the model.
    EXPECT_EQ(_obs.updates, _obs.wrenchKept);

    EXPECT_EQ(_obs.initialPose, _obs.pose);

    // Physics writes no bounding box and leaves the extra commands, if the
    // test added any, for the owner to consume.
    EXPECT_FALSE(_obs.linkBoxWritten);
    EXPECT_FALSE(_obs.modelBoxWritten);
    if (this->extraCmdTargets.count(_name) > 0)
    {
      EXPECT_EQ(_obs.updates, _obs.extraCmdsKept);
    }

    if (IsBox(_name))
    {
      EXPECT_FALSE(_obs.sawEngineContact);
      EXPECT_TRUE(_obs.fakeContactKept);
    }
    else
    {
      EXPECT_EQ(_obs.updates, _obs.jointCmdKept);
      EXPECT_TRUE(_obs.jointPosition.empty());
      EXPECT_TRUE(_obs.jointVelocity.empty());
    }
  }

  /// \brief Check that the Physics system did not warn about entities that it
  /// does not own.
  protected: void ExpectNoPhysicsWarnings(const std::string &_stderr)
  {
    for (const auto &text : {"Failed to find", "not found on",
        "Internal error", "not found in"})
    {
      const auto pos = _stderr.find(text);
      EXPECT_EQ(std::string::npos, pos) << (pos == std::string::npos ? "" :
          _stderr.substr(pos > 80 ? pos - 80 : 0, 240));
    }
  }
};

/// \brief Mark a model as owned by a provider.
void SetOwner(EntityComponentManager &_ecm, const Entity &_entity,
    const std::string &_owner)
{
  _ecm.CreateComponent(_entity, components::PhysicsOwner(_owner));
}
}

/////////////////////////////////////////////////
TEST_F(PhysicsSkipOwnedTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(ModelOwnedByOtherProvider))
{
  // Commands that Physics would act on if it handled these models.
  this->extraCmdTargets = {"owned_box", "owned_arm", "nested_box"};

  std::string err;
  auto obs = this->Run(
      [](EntityComponentManager &_ecm, const Entity &,
         std::map<std::string, ModelEntities> &_entities)
      {
        SetOwner(_ecm, _entities["owned_box"].model, kOther);
        SetOwner(_ecm, _entities["owned_arm"].model, kOther);
      }, nullptr, err);

  this->ExpectSimulated(obs["free_box"], "free_box");
  this->ExpectSimulated(obs["free_arm"], "free_arm");
  this->ExpectSkipped(obs["owned_box"], "owned_box");
  this->ExpectSkipped(obs["owned_arm"], "owned_arm");
  this->ExpectSkipped(obs["nested_box"], "nested_box");
  this->ExpectNoPhysicsWarnings(err);
}

/////////////////////////////////////////////////
TEST_F(PhysicsSkipOwnedTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(OwnedByPhysicsOrEmptyIsSimulated))
{
  std::string err;
  auto obs = this->Run(
      [](EntityComponentManager &_ecm, const Entity &,
         std::map<std::string, ModelEntities> &_entities)
      {
        SetOwner(_ecm, _entities["owned_box"].model, kPhysics);
        SetOwner(_ecm, _entities["owned_arm"].model, "");
      }, nullptr, err);

  for (const auto &name : {"free_box", "owned_box", "free_arm", "owned_arm",
                           "nested_box"})
  {
    this->ExpectSimulated(obs[name], name);
  }
  this->ExpectNoPhysicsWarnings(err);
}

/////////////////////////////////////////////////
TEST_F(PhysicsSkipOwnedTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(WorldDefaultWithModelOverride))
{
  std::string err;
  auto obs = this->Run(
      [](EntityComponentManager &_ecm, const Entity &_world,
         std::map<std::string, ModelEntities> &_entities)
      {
        // Every model goes to another provider, except the ground and the
        // two free models, which carry their own PhysicsOwner.
        SetOwner(_ecm, _world, kOther);
        SetOwner(_ecm, _ecm.EntityByComponents(components::Model(),
            components::Name("ground")), "");
        SetOwner(_ecm, _entities["free_box"].model, "");
        SetOwner(_ecm, _entities["free_arm"].model, kPhysics);
      }, nullptr, err);

  this->ExpectSimulated(obs["free_box"], "free_box");
  this->ExpectSimulated(obs["free_arm"], "free_arm");
  this->ExpectSkipped(obs["owned_box"], "owned_box");
  this->ExpectSkipped(obs["owned_arm"], "owned_arm");
  this->ExpectSkipped(obs["nested_box"], "nested_box");
  this->ExpectNoPhysicsWarnings(err);
}

/////////////////////////////////////////////////
TEST_F(PhysicsSkipOwnedTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(RemoveOwnedModels))
{
  std::string err;
  auto obs = this->Run(
      [](EntityComponentManager &_ecm, const Entity &,
         std::map<std::string, ModelEntities> &_entities)
      {
        SetOwner(_ecm, _entities["owned_box"].model, kOther);
        SetOwner(_ecm, _entities["owned_arm"].model, kOther);
      },
      [](EntityComponentManager &_ecm,
         std::map<std::string, ModelEntities> &_entities, int _iteration)
      {
        if (_iteration == 100)
        {
          _ecm.RequestRemoveEntity(_entities["owned_box"].model);
          _ecm.RequestRemoveEntity(_entities["owned_arm"].model);
        }
      }, err);

  // The owned models are gone, and the others are not affected.
  EXPECT_LT(obs["owned_box"].updates, kIterations);
  EXPECT_LT(obs["owned_arm"].updates, kIterations);
  EXPECT_LT(obs["nested_box"].updates, kIterations);
  this->ExpectSimulated(obs["free_box"], "free_box");
  this->ExpectSimulated(obs["free_arm"], "free_arm");
  this->ExpectNoPhysicsWarnings(err);
}

/////////////////////////////////////////////////
// A world reset restores the initial state of the ECM and marks every entity
// as new. The owners that a model plugin set before that state was stored come
// back with it, and Physics still leaves the owned models alone.
TEST_F(PhysicsSkipOwnedTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(OwnedModelsAcrossReset))
{
  this->UseWorldWithOwnerPlugins();
  this->resetAt = 200;

  std::string err;
  auto obs = this->Run(
      [](EntityComponentManager &, const Entity &,
         std::map<std::string, ModelEntities> &) {}, nullptr, err);

  this->ExpectSimulated(obs["free_box"], "free_box");
  this->ExpectSimulated(obs["free_arm"], "free_arm");
  this->ExpectSkipped(obs["owned_box"], "owned_box");
  this->ExpectSkipped(obs["owned_arm"], "owned_arm");
  this->ExpectSkipped(obs["nested_box"], "nested_box");
  this->ExpectNoPhysicsWarnings(err);
}

/////////////////////////////////////////////////
// A PhysicsOwner that was added after the initial state is gone after a
// reset. Physics decides again who owns each entity and then simulates the
// model, without stale entries from before the reset.
TEST_F(PhysicsSkipOwnedTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(OwnerLostInReset))
{
  this->resetAt = 200;

  std::string err;
  auto obs = this->Run(
      [](EntityComponentManager &_ecm, const Entity &,
         std::map<std::string, ModelEntities> &_entities)
      {
        SetOwner(_ecm, _entities["owned_box"].model, kOther);
        SetOwner(_ecm, _entities["owned_arm"].model, kOther);
      }, nullptr, err);

  // The models that were owned before the reset fall after it.
  for (const auto &name : {"owned_box", "nested_box"})
  {
    SCOPED_TRACE(name);
    EXPECT_LT(obs[name].pose.Z(), obs[name].initialPose.Z() - 0.3);
  }
  EXPECT_TRUE(obs["owned_box"].sawEngineContact);

  // The arm reports the joint state that it only reports while simulated.
  ASSERT_EQ(1u, obs["owned_arm"].jointPosition.size());
  EXPECT_GT(obs["owned_arm"].jointPosition[0], 0.3);
  this->ExpectNoPhysicsWarnings(err);
}

/////////////////////////////////////////////////
// While paused, Physics has no list of changed links from the engine and
// visits every link in the ECM. That path must skip the owned links too.
TEST_F(PhysicsSkipOwnedTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(OwnedModelsWhilePaused))
{
  this->extraCmdTargets = {"owned_box", "owned_arm", "nested_box"};
  this->iterations = 50;
  this->paused = true;

  std::string err;
  auto obs = this->Run(
      [](EntityComponentManager &_ecm, const Entity &,
         std::map<std::string, ModelEntities> &_entities)
      {
        SetOwner(_ecm, _entities["owned_box"].model, kOther);
        SetOwner(_ecm, _entities["owned_arm"].model, kOther);
      }, nullptr, err);

  this->ExpectSkipped(obs["owned_box"], "owned_box");
  this->ExpectSkipped(obs["owned_arm"], "owned_arm");
  this->ExpectSkipped(obs["nested_box"], "nested_box");
  this->ExpectNoPhysicsWarnings(err);
}

/////////////////////////////////////////////////
// Only the creation of a model decides who simulates it. A PhysicsOwner that
// appears later changes nothing, so Physics keeps writing the state of the
// model and clearing its commands, and it says so once.
TEST_F(PhysicsSkipOwnedTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(OwnerAddedAfterCreation))
{
  std::string err;
  auto obs = this->Run(
      [](EntityComponentManager &, const Entity &,
         std::map<std::string, ModelEntities> &) {},
      [](EntityComponentManager &_ecm,
         std::map<std::string, ModelEntities> &_entities, int _iteration)
      {
        if (_iteration == 100)
        {
          SetOwner(_ecm, _entities["free_box"].model, kOther);
          SetOwner(_ecm, _entities["free_arm"].model, kOther);
        }
      }, err);

  for (const auto &name : {"free_box", "free_arm"})
  {
    // No command is left behind, and the arm keeps reporting its state.
    this->ExpectSimulated(obs[name], name);

    const std::string text = std::string("Entity [") + name +
        "] was given PhysicsOwner";
    const auto first = err.find(text);
    ASSERT_NE(std::string::npos, first) << name;
    EXPECT_EQ(std::string::npos, err.find(text, first + 1)) << name;
  }
  this->ExpectNoPhysicsWarnings(err);
}

/////////////////////////////////////////////////
// A model that is spawned while the simulation runs gets its PhysicsOwner
// from a plugin in its own SDF. The plugin is configured when the model is
// created, which is before the Physics system sees the model, so Physics
// never builds a body for it. A change to plugin loading that breaks this
// order makes the owned model fall for one step or more.
TEST_F(PhysicsSkipOwnedTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(SpawnedModelOwnedByPlugin))
{
  const auto modelSdf = [](const std::string &_name, double _x,
                           const std::string &_plugin)
  {
    return std::string("<sdf version='1.9'><model name='") + _name +
        "'><pose>" + std::to_string(_x) + " 0 2 0 0 0</pose>"
        "<link name='link'>"
        "<inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy>"
        "<izz>0.01</izz></inertia></inertial>"
        "<collision name='collision'><geometry><box><size>0.2 0.2 0.2</size>"
        "</box></geometry></collision></link>" + _plugin + "</model></sdf>";
  };

  const std::string ownerPlugin =
      "<plugin filename='SetPhysicsOwnerSystem' "
      "name='gz::sim::SetPhysicsOwnerSystem'><owner>" +
      std::string(kOther) + "</owner></plugin>";

  ::testing::internal::CaptureStderr();

  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "physics_skip_owned.sdf"));

  EventManager *eventMgr{nullptr};
  Entity worldEntity{kNullEntity};
  int iteration{0};
  std::map<std::string, std::vector<math::Pose3d>> poses;

  fixture.OnConfigure([&](
      const Entity &_world,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &,
      EventManager &_eventMgr)
      {
        worldEntity = _world;
        eventMgr = &_eventMgr;
      })
  .OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        if (iteration != 50)
          return;

        SdfEntityCreator creator(_ecm, *eventMgr);
        for (const auto &[name, x, plugin] :
             {std::make_tuple("spawned_owned", -3.0, ownerPlugin),
              std::make_tuple("spawned_free", -6.0, std::string())})
        {
          sdf::Root root;
          const auto errors = root.LoadSdfString(modelSdf(name, x, plugin));
          ASSERT_TRUE(errors.empty()) << errors[0].Message();
          ASSERT_NE(nullptr, root.Model());
          creator.SetParent(creator.CreateEntities(root.Model()), worldEntity);
        }
      })
  .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        ++iteration;
        for (const auto &name : {"spawned_owned", "spawned_free"})
        {
          const auto model = _ecm.EntityByComponents(
              components::Model(), components::Name(name));
          if (model != kNullEntity)
            poses[name].push_back(worldPose(model, _ecm));
        }
      })
  .Finalize();

  fixture.Server()->Run(true, kIterations, false);
  const std::string err = ::testing::internal::GetCapturedStderr();

  // The owned model is in the ECM from the first update after it was spawned
  // and never moves, not even in the step that created it.
  ASSERT_FALSE(poses["spawned_owned"].empty());
  for (const auto &pose : poses["spawned_owned"])
    EXPECT_EQ(poses["spawned_owned"].front(), pose);
  EXPECT_NEAR(2.0, poses["spawned_owned"].back().Z(), 1e-9);

  // The model without an owner falls.
  ASSERT_FALSE(poses["spawned_free"].empty());
  EXPECT_LT(poses["spawned_free"].back().Z(), 1.5);

  this->ExpectNoPhysicsWarnings(err);
}

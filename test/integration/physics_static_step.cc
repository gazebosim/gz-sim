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

// The Physics system does not step the engine while it holds no body that can
// move. These tests start in that state, change it in each way the system must
// notice, and check that the world then behaves as if every step had run.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/utils/ExtraTestMacros.hh>
#include <sdf/Root.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/World.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

namespace
{
/// \brief A world with a ground plane, a static box and a static box that
/// hangs in the air. Nothing in it can move.
const char *kStaticWorld = R"(
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="static_only">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
    </plugin>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
      </link>
    </model>
    <model name="shelf">
      <static>true</static>
      <pose>3 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
      </link>
    </model>
    <model name="hanging">
      <static>true</static>
      <pose>0 5 3 0 0 0</pose>
      <link name="link">
        <inertial><mass>1</mass></inertial>
        <collision name="collision">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
)";

/// \brief A static model that holds a nested model without `<static>`. The
/// engine simulates the nested model as dynamic.
const char *kNestedWorld = R"(
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="nested">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
    </plugin>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
      </link>
    </model>
    <model name="outer">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
        </collision>
      </link>
      <model name="inner">
        <pose>0 0 3 0 0 0</pose>
        <link name="link">
          <inertial>
            <mass>1</mass>
            <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia>
          </inertial>
          <collision name="collision">
            <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
          </collision>
        </link>
      </model>
    </model>
  </world>
</sdf>
)";

/// \brief A dynamic sphere with a 0.1 m radius.
std::string BallSdf(const std::string &_name, const math::Pose3d &_pose)
{
  std::ostringstream stream;
  stream << R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <model name=")" << _name << R"(">
    <pose>)" << _pose << R"(</pose>
    <link name="link">
      <inertial>
        <mass>1</mass>
        <inertia><ixx>0.004</ixx><iyy>0.004</iyy><izz>0.004</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.1</radius></sphere></geometry>
      </collision>
    </link>
  </model>
</sdf>)";
  return stream.str();
}
}  // namespace

/// \brief Runs the static world and gives the tests the ECM.
class PhysicsStaticStepTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture<::testing::Test>::SetUp();

    ServerConfig serverConfig;
    serverConfig.SetSdfString(this->WorldSdf());
    this->server = std::make_unique<Server>(serverConfig);
    this->server->SetUpdatePeriod(0ns);

    this->relay.OnPreUpdate(
        [this](const UpdateInfo &_info, EntityComponentManager &_ecm)
        {
          this->ecm = &_ecm;
          this->simTime = _info.simTime;
        });
    this->server->AddSystem(this->relay.systemPtr);
    this->server->Run(true, 1, false);
    ASSERT_NE(nullptr, this->ecm);
    this->creator = std::make_unique<SdfEntityCreator>(
        *this->ecm, this->eventManager);
  }

  /// \brief The world that the test starts from.
  /// \return SDF of the world.
  protected: virtual std::string WorldSdf() const
  {
    return kStaticWorld;
  }

  /// \brief Advance the world.
  /// \param[in] _steps Number of steps.
  /// \param[in] _paused True to run the steps paused.
  protected: void Run(const unsigned int _steps, const bool _paused = false)
  {
    if (!_paused)
    {
      this->server->Run(true, _steps, false);
      return;
    }
    for (unsigned int i = 0; i < _steps; ++i)
      EXPECT_TRUE(this->server->RunOnce(true));
  }

  /// \brief Add a model from an SDF string to the world.
  /// \param[in] _sdf Model SDF.
  /// \return The model entity.
  protected: Entity Spawn(const std::string &_sdf)
  {
    sdf::Root root;
    const auto errors = root.LoadSdfString(_sdf);
    EXPECT_TRUE(errors.empty());
    EXPECT_NE(nullptr, root.Model());
    const auto model = this->creator->CreateEntities(root.Model());
    this->creator->SetParent(model,
        this->ecm->EntityByComponents(components::World()));
    return model;
  }

  /// \brief Find a model by name.
  /// \param[in] _name Model name.
  /// \return The model entity.
  protected: Entity ModelByName(const std::string &_name) const
  {
    return this->ecm->EntityByComponents(
        components::Model(), components::Name(_name));
  }

  /// \brief Pose component of a model.
  /// \param[in] _model Model entity.
  /// \return The model pose.
  protected: math::Pose3d PoseOf(const Entity _model) const
  {
    return this->ecm->Component<components::Pose>(_model)->Data();
  }

  protected: std::unique_ptr<Server> server;
  protected: test::Relay relay;
  protected: EntityComponentManager *ecm{nullptr};
  protected: EventManager eventManager;
  protected: std::unique_ptr<SdfEntityCreator> creator;
  protected: std::chrono::steady_clock::duration simTime{0};
};

/////////////////////////////////////////////////
// Sim time and static models do not depend on the engine step.
TEST_F(PhysicsStaticStepTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(StaticWorldRuns))
{
  const auto shelf = this->ModelByName("shelf");
  ASSERT_NE(kNullEntity, shelf);
  const auto shelfPose = this->PoseOf(shelf);

  const auto start = this->simTime;
  this->Run(200);
  EXPECT_EQ(start + 200ms, this->simTime);
  EXPECT_EQ(shelfPose, this->PoseOf(shelf));
  EXPECT_EQ(math::Pose3d(0, 5, 3, 0, 0, 0),
      this->PoseOf(this->ModelByName("hanging")));
}

/////////////////////////////////////////////////
// A model that is added while the engine only holds static bodies falls and
// comes to rest on the ground.
TEST_F(PhysicsStaticStepTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(SpawnedModelFalls))
{
  this->Run(100);

  const auto ball =
      this->Spawn(BallSdf("ball", math::Pose3d(0, 0, 1, 0, 0, 0)));
  this->Run(5);
  EXPECT_GT(1.0, this->PoseOf(ball).Z());

  this->Run(800);
  EXPECT_NEAR(0.1, this->PoseOf(ball).Z(), 0.01);
}

/////////////////////////////////////////////////
// The engine is stepped again after the last dynamic model is removed and
// after another one is added.
TEST_F(PhysicsStaticStepTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(RemoveThenSpawn))
{
  const auto first =
      this->Spawn(BallSdf("first", math::Pose3d(0, 0, 1, 0, 0, 0)));
  this->Run(20);
  EXPECT_GT(1.0, this->PoseOf(first).Z());

  this->creator->RequestRemoveEntity(first, true);
  this->Run(100);
  EXPECT_EQ(kNullEntity, this->ModelByName("first"));

  const auto second =
      this->Spawn(BallSdf("second", math::Pose3d(0, 0, 1, 0, 0, 0)));
  this->Run(800);
  EXPECT_NEAR(0.1, this->PoseOf(second).Z(), 0.01);
}

/////////////////////////////////////////////////
// A static model that is made dynamic starts to fall, and one that is made
// static again stops.
TEST_F(PhysicsStaticStepTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(StaticCmdWakesAndFreezes))
{
  const auto hanging = this->ModelByName("hanging");
  ASSERT_NE(kNullEntity, hanging);

  this->Run(100);
  EXPECT_DOUBLE_EQ(3.0, this->PoseOf(hanging).Z());

  this->ecm->CreateComponent(hanging, components::StaticCmd(false));
  this->Run(100);
  const double fallen = this->PoseOf(hanging).Z();
  EXPECT_GT(3.0 - 0.02, fallen);

  this->ecm->CreateComponent(hanging, components::StaticCmd(true));
  this->Run(10);
  const double frozen = this->PoseOf(hanging).Z();
  this->Run(100);
  EXPECT_DOUBLE_EQ(frozen, this->PoseOf(hanging).Z());
  EXPECT_GT(fallen, 0.0);

  this->ecm->CreateComponent(hanging, components::StaticCmd(false));
  this->Run(100);
  EXPECT_GT(frozen - 0.02, this->PoseOf(hanging).Z());
}

/////////////////////////////////////////////////
// A world pose command moves a static model while the engine is idle, both in
// the ECM and in the engine, where a ball lands on it.
TEST_F(PhysicsStaticStepTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WorldPoseCmdOnStaticModel))
{
  const auto shelf = this->ModelByName("shelf");
  ASSERT_NE(kNullEntity, shelf);

  this->Run(100);

  const math::Pose3d moved(-4, 2, 0.5, 0, 0, 0);
  Model(shelf).SetWorldPoseCmd(*this->ecm, moved);
  this->Run(2);
  EXPECT_EQ(moved, this->PoseOf(shelf));
  this->Run(100);
  EXPECT_EQ(moved, this->PoseOf(shelf));

  // The top of the shelf is at z = 1 m.
  const auto ball =
      this->Spawn(BallSdf("ball", math::Pose3d(-4, 2, 2, 0, 0, 0)));
  this->Run(1000);
  EXPECT_NEAR(1.1, this->PoseOf(ball).Z(), 0.01);
}

/////////////////////////////////////////////////
// Contact data of a static model is cleared after the body that touched it is
// gone.
TEST_F(PhysicsStaticStepTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ContactsEndWithTheLastBody))
{
  const auto shelf = Model(this->ModelByName("shelf"));
  const auto shelfLink = shelf.Links(*this->ecm).front();
  const auto shelfCollision =
      this->ecm->ChildrenByComponents(shelfLink, components::Collision())
          .front();
  this->ecm->CreateComponent(shelfCollision, components::ContactSensorData());

  // The ball rests on the shelf.
  const auto ball =
      this->Spawn(BallSdf("ball", math::Pose3d(3, 0, 1.2, 0, 0, 0)));
  this->Run(800);
  const auto *contacts = this->ecm->Component<components::ContactSensorData>(
      shelfCollision);
  ASSERT_NE(nullptr, contacts);
  EXPECT_LT(0, contacts->Data().contact_size());

  this->creator->RequestRemoveEntity(ball, true);
  this->Run(10);
  contacts = this->ecm->Component<components::ContactSensorData>(
      shelfCollision);
  ASSERT_NE(nullptr, contacts);
  EXPECT_EQ(0, contacts->Data().contact_size());
}

/////////////////////////////////////////////////
// A joint reset on a static model still reaches the engine while it is idle.
TEST_F(PhysicsStaticStepTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointResetOnStaticModel))
{
  const std::string sdfString = R"(
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="arm">
    <static>true</static>
    <pose>0 -5 2 0 0 0</pose>
    <link name="base">
      <inertial><mass>1</mass></inertial>
    </link>
    <link name="tip">
      <pose>0 0 1 0 0 0</pose>
      <inertial><mass>1</mass></inertial>
      <collision name="collision">
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
      </collision>
    </link>
    <joint name="hinge" type="revolute">
      <parent>base</parent>
      <child>tip</child>
      <axis><xyz>1 0 0</xyz></axis>
    </joint>
  </model>
</sdf>)";
  const auto arm = this->Spawn(sdfString);
  this->Run(100);

  const auto armModel = Model(arm);
  const auto tipLink = armModel.LinkByName(*this->ecm, "tip");
  const auto joint = armModel.JointByName(*this->ecm, "hinge");
  ASSERT_NE(kNullEntity, tipLink);
  ASSERT_NE(kNullEntity, joint);

  const auto before = this->ecm->Component<components::Pose>(tipLink)->Data();
  this->ecm->CreateComponent(joint,
      components::JointPositionReset(std::vector<double>{1.0}));
  this->Run(3);
  const auto after = this->ecm->Component<components::Pose>(tipLink)->Data();
  EXPECT_NE(before, after);
}

/////////////////////////////////////////////////
// A nested model without `<static>` inside a static model is dynamic in the
// engine. The engine must keep stepping for it.
class PhysicsStaticStepNestedTest : public PhysicsStaticStepTest
{
  // Documentation inherited
  protected: std::string WorldSdf() const override
  {
    return kNestedWorld;
  }
};

TEST_F(PhysicsStaticStepNestedTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(DynamicNestedModelInStaticModel))
{
  const auto inner = this->ecm->EntityByComponents(
      components::Model(), components::Name("inner"));
  ASSERT_NE(kNullEntity, inner);

  this->Run(200);
  EXPECT_GT(2.9, this->PoseOf(inner).Z());
}

/////////////////////////////////////////////////
// A write that arrives while the world is paused reaches the first step after
// the pause, also for a model that was made static at runtime.
TEST_F(PhysicsStaticStepTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WriteWhilePausedReachesTheStep))
{
  const std::string sdfString = R"(
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="arm">
    <pose>0 -5 2 0 0 0</pose>
    <link name="base">
      <gravity>false</gravity>
      <inertial><mass>1</mass></inertial>
    </link>
    <link name="tip">
      <gravity>false</gravity>
      <pose>0 0 1 0 0 0</pose>
      <inertial><mass>1</mass></inertial>
    </link>
    <joint name="world_fixed" type="fixed">
      <parent>world</parent>
      <child>base</child>
    </joint>
    <joint name="hinge" type="revolute">
      <parent>base</parent>
      <child>tip</child>
      <axis><xyz>1 0 0</xyz></axis>
    </joint>
  </model>
</sdf>)";
  const auto arm = this->Spawn(sdfString);
  this->Run(10);

  this->ecm->CreateComponent(arm, components::StaticCmd(true));
  this->Run(100);

  const auto armModel = Model(arm);
  const auto tipLink = armModel.LinkByName(*this->ecm, "tip");
  const auto joint = armModel.JointByName(*this->ecm, "hinge");
  ASSERT_NE(kNullEntity, tipLink);
  ASSERT_NE(kNullEntity, joint);
  const auto before = this->ecm->Component<components::Pose>(tipLink)->Data();

  this->ecm->CreateComponent(joint,
      components::JointPositionReset(std::vector<double>{1.0}));
  this->Run(3, true);
  this->Run(5);
  const auto after = this->ecm->Component<components::Pose>(tipLink)->Data();
  EXPECT_NE(before, after);
}

/////////////////////////////////////////////////
// A world with many static links, where an engine step costs real time.
class PhysicsStaticStepManyTest : public PhysicsStaticStepTest
{
  // Documentation inherited
  protected: std::string WorldSdf() const override
  {
    std::ostringstream stream;
    stream << R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="many">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
    </plugin>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
      </link>
    </model>)";
    for (int i = 0; i < 2000; ++i)
    {
      stream << "<model name=\"shelf" << i << "\"><static>true</static>"
             << "<pose>" << 3 * (i % 50) + 3 << " " << 3 * (i / 50) + 3
             << " 0.5 0 0 0</pose><link name=\"link\"><collision "
             << "name=\"collision\"><geometry><box><size>1 1 1</size></box>"
             << "</geometry></collision></link></model>";
    }
    stream << "</world></sdf>";
    return stream.str();
  }

  /// \brief Wall time of a number of steps.
  /// \param[in] _steps Number of steps.
  /// \return The shortest of three runs of that many steps.
  protected: std::chrono::nanoseconds Time(const unsigned int _steps)
  {
    auto best = std::chrono::nanoseconds::max();
    for (int i = 0; i < 3; ++i)
    {
      const auto start = std::chrono::steady_clock::now();
      this->Run(_steps);
      best = std::min(best, std::chrono::duration_cast<
          std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start));
    }
    return best;
  }
};

// The engine step is absent while nothing can move. The step of a world with
// 2000 static boxes costs far more than the rest of an update, so a world
// that is idle must run much faster than the same world with a ball in it.
TEST_F(PhysicsStaticStepManyTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(IdleWorldDoesNotStep))
{
  this->Run(20);
  const auto idle = this->Time(100);

  this->Spawn(BallSdf("ball", math::Pose3d(0, 0, 0.5, 0, 0, 0)));
  this->Run(20);
  const auto busy = this->Time(100);

  EXPECT_LT(idle * 10, busy)
      << "idle " << idle.count() << " ns, with a ball " << busy.count()
      << " ns";
}

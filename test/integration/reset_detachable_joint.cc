/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

/// This test exercises resetting a world with a detachable joint.
/// The simulation starts with an arm (simple_arm) and an object to be
/// maniupulated(object).
/// When simulation runs:
///  * At t=1.0, the arm is aligned with the object (x=+0.15),
///  * At t=1.1, the arm forms the detchable joint
///  * At t=3.0, the arm moves over the end of the table (x=+0.3)
///  * At t=5.0, the simulation is reset
///
/// If physics and reset are implemented correctly, when the position of
/// the arm is reset to x=-0.06, the block should not follow or stay
/// attached via the detachable joint.
///
/// If physics and reset are implemented incorrectly, the block will move
/// with the arm and fail the test.

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <string>
#include <vector>

#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/Util.hh"

#include "test_config.hh"

#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Name of the arm model
constexpr const char* kArmName = "simple_arm";

/// \brief Name of the manipulated object model
constexpr const char* kObjectName = "object";

/////////////////////////////////////////////////
class TestPlugin:
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate,
  public ISystemReset
{
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override;

  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override;

  public: void Reset(const UpdateInfo &_info,
                     EntityComponentManager &_ecm) override;

  /// brief Flag to indicate if the gripper is in use or not
  bool grasping = false;

  /// \brief Flag to indicate if error state has been reached
  bool errorLogged = false;

  /// \brief Flag to indicate if the system has encountered a reset
  bool didReset = false;

  /// \brief Link object simple_arm::arm_link
  Link link;

  /// \brief Entity for simple_arm::arm_link::arm_link_collision
  Entity linkCollisionEntity;

  /// \brief Entity for simple_arm::base_joint
  Entity jointEntity;

  /// \brief Entity for the object to be manipulated
  Entity objectModelEntity;

  /// \brief Entity for the link in the object to be manipulated
  Entity collidingObjectLinkEntity;

  /// \brief Entity for the detachable joint created between the
  /// arm and the object
  Entity detachableJointEntity;

  /// \brief Set point for the prismatic base joint
  double commandPosition = 0;
};

/////////////////////////////////////////////////
void TestPlugin::Configure(const Entity &,
               const std::shared_ptr<const sdf::Element>&,
               EntityComponentManager &_ecm, EventManager&) {

  Entity simpleArmEntity;

  _ecm.Each<components::Model, components::Name>(
      [&](const Entity &_entityIt,
          const components::Model *,
          const components::Name *_name) -> bool
      {
        if (_name->Data() == kArmName)
        {
          simpleArmEntity = _entityIt;
          return false;
        }
        return true;
      });

  auto model = Model(simpleArmEntity);
  this->link = Link(model.Links(_ecm)[0]);
  this->jointEntity = model.Joints(_ecm)[0];
  this->linkCollisionEntity = this->link.Collisions(_ecm)[0];

  // Add ContactSensorData component to link collision so that we can grasp
  // the object when the arm is in contact with it.
  _ecm.CreateComponent(this->linkCollisionEntity,
                      components::ContactSensorData());

  auto objects = gz::sim::entitiesFromScopedName(kObjectName, _ecm);
  this->objectModelEntity = *objects.begin();
}

/////////////////////////////////////////////////
void TestPlugin::Reset(const UpdateInfo &, EntityComponentManager &) {
  this->didReset = true;
}

/////////////////////////////////////////////////
void TestPlugin::PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) {

  auto pose = worldPose(this->objectModelEntity, _ecm);

  if (pose.Y() < -0.01 && !this->errorLogged)
  {
    this->errorLogged = true;
    gzerr << "Object moved unexpectedly to left of table! \n";
  }

  // The plugin behavior below is only for the first time the simulation runs,
  // so if we did reset already, just ignore.
  if (this->didReset) return;

  // Try grasping object with arm after 1.1 seconds
  if (!this->grasping && _info.iterations > 1100) {
    std::optional<gz::msgs::Contacts> contactDataOptional =
        _ecm.ComponentData<components::ContactSensorData>(
            this->linkCollisionEntity);

    if (contactDataOptional &&
        contactDataOptional.value().contact_size() > 0) {
      auto collision1 = contactDataOptional.value().contact(0).collision1();
      if (collision1.id() != this->linkCollisionEntity) {
        this->collidingObjectLinkEntity = _ecm.ParentEntity(collision1.id());
      }
      auto collision2 = contactDataOptional.value().contact(0).collision2();
      if (collision2.id() != this->linkCollisionEntity) {
        this->collidingObjectLinkEntity = _ecm.ParentEntity(collision2.id());
      }

      // Add a DetachableJoint between arm link and object to emulate a
      // suction gripper.
      this->detachableJointEntity = _ecm.CreateEntity();
      components::DetachableJointInfo info;
      info.parentLink = this->link.Entity();
      info.childLink = this->collidingObjectLinkEntity;

      _ecm.CreateComponent(this->detachableJointEntity,
                          components::DetachableJoint(info));
      this->grasping = true;
    }
  }

  // At 1 second, set command position to 0.15, which corresponds to the arm
  // lining up with the object.
  // At 3 seconds, set command position to 0.3, which corresponds to the arm
  // hanging out to the right of the table.
  if (_info.iterations == 1000) {
    this->commandPosition = 0.15;
  } else if (_info.iterations == 3000) {
    this->commandPosition = 0.3;
  }

  // Set joint position directly to commanded position by setting the
  // commanded position to the JointPositionReset component.
  auto& component =
      *(_ecm.CreateComponent(this->jointEntity,
                             components::JointPositionReset()));
  component.Data().resize(1);
  component.Data()[0] = this->commandPosition;
}

/////////////////////////////////////////////////
/// \brief Test DetachableJoint system
class ResetDetachableJointTest : public InternalFixture<::testing::Test>
{
  /// \brief Start the server with a test plugin
  /// \param[in] _sdfFile sdf world to load
  public: void StartServer(const std::string &_sdfFile)
  {
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH, _sdfFile));
    this->server = std::make_unique<Server>(serverConfig);

    this->system = std::make_shared<TestPlugin>();
    this->server->AddSystem(this->system);

    EXPECT_FALSE(this->server->Running());
    EXPECT_FALSE(*this->server->Running(0));
  }

  /// \brief the Simulation server instance
  public: std::unique_ptr<Server> server;

  /// \brief the Simulation system instance
  public: std::shared_ptr<TestPlugin> system;
};

/////////////////////////////////////////////////
/// \brief Reset the simulation world via transport
void worldReset()
{
  gz::msgs::WorldControl req;
  gz::msgs::Boolean rep;
  req.mutable_reset()->set_all(true);
  transport::Node node;

  unsigned int timeout = 1000;
  bool result;
  bool executed =
    node.Request("/world/default/control", req, timeout, rep, result);

  ASSERT_TRUE(executed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());
}

/////////////////////////////////////////////////
TEST_F(ResetDetachableJointTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(HandleReset))
{
  this->StartServer("/test/worlds/reset_detachable_joint.sdf");
  this->server->Run(true, 5000, false);
  ASSERT_FALSE(this->system->didReset);
  ASSERT_FALSE(this->system->errorLogged);

  // First Reset
  worldReset();
  this->server->Run(true, 5000, false);
  ASSERT_TRUE(this->system->didReset);
  ASSERT_FALSE(this->system->errorLogged);

  // Second Reset
  worldReset();
  server->Run(true, 1000, false);
  ASSERT_TRUE(this->system->didReset);
  ASSERT_FALSE(this->system->errorLogged);
}

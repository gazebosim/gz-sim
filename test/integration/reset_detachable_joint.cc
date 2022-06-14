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

#include <gtest/gtest.h>

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

#include "gz/sim/test_config.hh"

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

  bool grasping_ = false;
  Entity object_model_entity_;
  Entity joint_entity_;
  Link link_;
  Entity link_collision_entity_;
  Entity colliding_object_link_entity_;
  Entity detachable_joint_entity_;
  double command_position_ = 0;
  bool error_logged_ = false;
  bool did_reset_ = false;
};

/////////////////////////////////////////////////
void TestPlugin::Configure(const Entity &,
               const std::shared_ptr<const sdf::Element>&,
               EntityComponentManager &_ecm, EventManager&) {

  Entity simple_arm_entity;
  _ecm.Each<components::Model, components::Name>(
      [&](const Entity &_entityIt,
          const components::Model *,
          const components::Name *_name) -> bool
      {
        if (_name->Data() == "simple_arm")
        {
          simple_arm_entity = _entityIt;
          return false;
        }
        return true;
      });

  auto model = Model(simple_arm_entity);

  link_ = Link(model.Links(_ecm)[0]);
  joint_entity_ = model.Joints(_ecm)[0];

  link_collision_entity_ = link_.Collisions(_ecm)[0];

  // Add ContactSensorData component to link collision so that we can grasp
  // the object when the arm is in contact with it.
  _ecm.CreateComponent(link_collision_entity_,
                      components::ContactSensorData());

  auto objects = gz::sim::entitiesFromScopedName("object", _ecm);
  object_model_entity_ = *objects.begin();
}

/////////////////////////////////////////////////
void TestPlugin::Reset(const UpdateInfo &_info, EntityComponentManager &_ecm) {
  igndbg << "TestPlugin::Reset" << std::endl;
  this->did_reset_ = true;
}

/////////////////////////////////////////////////
void TestPlugin::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) {

  auto pose = worldPose(object_model_entity_, _ecm);
  // igndbg << "Object pose: " << pose.X() << " " << pose.Y() << " " << pose.Z() << "\n";

  if (pose.Y() < -0.01 && !error_logged_) {
    error_logged_ = true;
    ignerr << "Object moved unexpectedly to left of table! \n";
  }

  // The plugin behavior below is only for the first time the simulation runs,
  // so if we did reset already, just ignore.
  if (this->did_reset_) return;

  // Try grasping object with arm after 1.1 seconds
  if (!grasping_ && _info.iterations > 1100) {
    std::optional<gz::msgs::Contacts> contact_data_optional =
        _ecm.ComponentData<components::ContactSensorData>(
            link_collision_entity_);

    if (contact_data_optional &&
        contact_data_optional.value().contact_size() > 0) {
      auto collision1 = contact_data_optional.value().contact(0).collision1();
      if (collision1.id() != link_collision_entity_) {
        colliding_object_link_entity_ = _ecm.ParentEntity(collision1.id());
      }
      auto collision2 = contact_data_optional.value().contact(0).collision2();
      if (collision2.id() != link_collision_entity_) {
        colliding_object_link_entity_ = _ecm.ParentEntity(collision2.id());
      }

      // Add a DetachableJoint between arm link and object to emulate a
      // suction gripper.
      detachable_joint_entity_ = _ecm.CreateEntity();
      components::DetachableJointInfo info;
      info.parentLink = link_.Entity();
      info.childLink = colliding_object_link_entity_;

      _ecm.CreateComponent(detachable_joint_entity_,
                          components::DetachableJoint(info));
      grasping_ = true;
    }
  }

  // At 1 second, set command position to 0.15, which corresponds to the arm
  // lining up with the object.
  // At 3 seconds, set command position to 0.3, which corresponds to the arm
  // hanging out to the right of the table.
  if (_info.iterations == 1000) {
    command_position_ = 0.15;
  } else if (_info.iterations == 3000) {
    command_position_ = 0.3;
  }

  // Set joint position directly to commanded position by setting the
  // commanded position to the JointPositionReset component.
  auto& component =
      *(_ecm.CreateComponent(joint_entity_, components::JointPositionReset()));
  component.Data().resize(1);
  component.Data()[0] = command_position_;
}

/////////////////////////////////////////////////
/// \brief Test DetachableJoint system
class ResetDetachableJointTest : public InternalFixture<::testing::Test>
{
  public: void StartServer(const std::string &_sdfFile)
  {
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) + _sdfFile);
    this->server = std::make_unique<Server>(serverConfig);

    this->system = std::make_shared<TestPlugin>();
    this->server->AddSystem(this->system);

    EXPECT_FALSE(this->server->Running());
    EXPECT_FALSE(*this->server->Running(0));
  }

  public: std::unique_ptr<Server> server;
  public: std::shared_ptr<TestPlugin> system;

};


/////////////////////////////////////////////////
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

TEST_F(ResetDetachableJointTest, Reset)
{
  this->StartServer("/test/worlds/reset_detachable_joint.sdf");

  this->server->Run(true, 5000, false);
  ignmsg << "Server ran for " << server->IterationCount().value_or(0)
            << " iterations \n";

  // First Reset
  worldReset();

  this->server->Run(true, 5000, false);
  ignmsg << "Server ran for " << server->IterationCount().value_or(0)
            << " iterations \n";

  // Second Reset
  worldReset();

  server->Run(true, 1000, false);
  ignmsg << "Server ran for " << server->IterationCount().value_or(0)
            << " iterations \n";
}

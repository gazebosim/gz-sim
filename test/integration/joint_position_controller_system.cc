/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 * Copyright (C) 2023 Benjamin Perseghetti, Rudis Laboratories
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
#include <thread>
#include <utility>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/actuators.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/parameters/Registry.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Util.hh"

#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/UnitTestUtil.hh"

#define TOL 1e-4

using namespace gz;
using namespace sim;

/// \brief Test fixture for JointPositionController system
class JointPositionControllerTestFixture
  : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
/// \brief Test that the controller accepts position commands with physics.
/// \param[in] _derivativeOnMeasurement Whether to use measured velocity.
/// See https://github.com/gazebosim/gz-sim/issues/1175
void TestJointPositionForceCommand(bool _derivativeOnMeasurement)
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller.sdf"));

  if (_derivativeOnMeasurement)
  {
    sdf::Root root;
    ASSERT_TRUE(root.Load(serverConfig.SdfFile()).empty());
    auto model = root.WorldByIndex(0)->ModelByName(
        "joint_position_controller_test");
    ASSERT_NE(nullptr, model);
    auto plugin = model->Plugins().at(0);
    ASSERT_TRUE(plugin.InsertContent(
        "<derivative_on_measurement>true</derivative_on_measurement>"));
    // Isolate the PD response from the integral's slower settling transient.
    for (const auto &content : plugin.Contents())
    {
      if (content->GetName() == "i_gain")
        content->Set(0.0);
    }
    model->ClearPlugins();
    model->AddPlugin(plugin);
    serverConfig.SetSdfRoot(root);
  }

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));
        // Create a JointPosition component if it doesn't exist. This signals
        // physics system to populate the component
        if (nullptr == _ecm.Component<components::JointPosition>(joint))
        {
          _ecm.CreateComponent(joint, components::JointPosition());
        }
      });

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
                const components::Joint *,
                const components::Name *_name,
                const components::JointPosition *_position) -> bool
            {
              EXPECT_EQ(_name->Data(), jointName);
              currentPosition = _position->Data();
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, currentPosition.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_position_controller_test/joint/j1/0/cmd_pos");

  const double targetPosition{2.0};
  msgs::Double msg;
  msg.set_data(targetPosition);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);

  EXPECT_NEAR(targetPosition, currentPosition.at(0), TOL);
}

/////////////////////////////////////////////////
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointPositionForceCommand))
{
  TestJointPositionForceCommand(false);
}

/////////////////////////////////////////////////
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointPositionForceCommandOnMeasurement))
{
  TestJointPositionForceCommand(true);
}

/////////////////////////////////////////////////
// Tests that the JointPositionController accepts joint position commands
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointPositionActuatorsCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller_actuators.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));
        // Create a JointPosition component if it doesn't exist. This signals
        // physics system to populate the component
        if (nullptr == _ecm.Component<components::JointPosition>(joint))
        {
          _ecm.CreateComponent(joint, components::JointPosition());
        }
      });

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
                const components::Joint *,
                const components::Name *_name,
                const components::JointPosition *_position) -> bool
            {
              EXPECT_EQ(_name->Data(), jointName);
              currentPosition = _position->Data();
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, currentPosition.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Actuators>(
      "/actuators");

  const double targetPosition{1.0};
  msgs::Actuators msg;
  msg.add_position(targetPosition);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 3000;
  server.Run(true, testIters , false);

  EXPECT_NEAR(targetPosition, currentPosition.at(0), TOL);
}

/////////////////////////////////////////////////
// Tests that the JointPositionController accepts joint position commands
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointPositionVelocityCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller_velocity.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));
        // Create a JointPosition component if it doesn't exist. This signals
        // physics system to populate the component
        if (nullptr == _ecm.Component<components::JointPosition>(joint))
        {
          _ecm.CreateComponent(joint, components::JointPosition());
        }
      });

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
                const components::Joint *,
                const components::Name *_name,
                const components::JointPosition *_position) -> bool
            {
              if (_name->Data() == jointName)
                currentPosition = _position->Data();
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // joint pos starts at 0
  const std::size_t initIters = 1;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, currentPosition.at(0), TOL);

  // joint moves to initial_position at -2.0
  const std::size_t initPosIters = 1;
  server.Run(true, initPosIters, false);
  double expectedInitialPosition = -2.0;
  EXPECT_NEAR(expectedInitialPosition, currentPosition.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_position_controller_test/joint/j1/0/cmd_pos");

  const double targetPosition{2.0};
  msgs::Double msg;
  msg.set_data(targetPosition);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(1ms);

  const std::size_t testIters = 1;
  server.Run(true, testIters, false);

  EXPECT_NEAR(targetPosition, currentPosition.at(0), TOL);
}


/////////////////////////////////////////////////
// Tests that the JointPositionController accepts joint position
// sub_topic commands
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(
       JointPositionMultipleJointsSubTopicCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller_multiple_joints_subtopic.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));
        // Create a JointPosition component if it doesn't exist. This signals
        // physics system to populate the component
        if (nullptr == _ecm.Component<components::JointPosition>(joint))
        {
          _ecm.CreateComponent(joint, components::JointPosition());
        }
      });

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
                const components::Joint *,
                const components::Name *_name,
                const components::JointPosition *_position) -> bool
            {
              EXPECT_EQ(_name->Data(), jointName);
              currentPosition = _position->Data();
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, currentPosition.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_position_controller_test/joints");

  const double targetPosition{2.0};
  msgs::Double msg;
  msg.set_data(targetPosition);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);

  EXPECT_NEAR(targetPosition, currentPosition.at(0), TOL);
}

/////////////////////////////////////////////////
// Tests that the JointPositionController commands joints in
// nested models.
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(
       JointPositionNestedModels))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller_nested_models.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  // model2 contains two joints with the same name, one is nested.
  const std::string modelName = "model2";
  const std::string jointName = "rotor_joint";
  const std::string scopedJointName = "model21::rotor_joint";

  test::Relay testSystem;
  std::vector<double> joint2Position;
  std::vector<double> joint21Position;
  testSystem.OnPreUpdate(
      [&](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
      {
        // Create a JointPosition component if it doesn't exist.
        _ecm.Each<components::Joint, components::Name>(
            [&](const gz::sim::Entity &_joint,
                const components::Joint *,
                const components::Name *_name) -> bool
            {
              if (_name->Data() == jointName)
              {
                _ecm.CreateComponent(_joint, components::JointPosition());
              }
              return true;
            });
      });

  testSystem.OnPostUpdate([&](const sim::UpdateInfo &,
                              const sim::EntityComponentManager &_ecm)
      {
        // Retrieve the parent model.
        Entity model{kNullEntity};
        _ecm.Each<components::Model,
                  components::Name>(
            [&](const gz::sim::Entity &_entity,
                const components::Model *,
                const components::Name *_name) -> bool
            {
              if (_name->Data() == modelName)
              {
                model = _entity;
              }
              return true;
            });

        // joint2
        {
          auto entities = entitiesFromScopedName(jointName, _ecm, model);
          Entity joint = *entities.begin();
          auto posComp = _ecm.Component<components::JointPosition>(joint);
          joint2Position = posComp->Data();
        }

        // joint21
        {
          auto entities = entitiesFromScopedName(scopedJointName, _ecm, model);
          Entity joint = *entities.begin();
          auto posComp = _ecm.Component<components::JointPosition>(joint);
          joint21Position = posComp->Data();
        }
      });

  server.AddSystem(testSystem.systemPtr);

  // joint pos starts at 0
  const std::size_t initIters = 1;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, joint2Position.at(0), TOL);
  EXPECT_NEAR(0, joint21Position.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model21/cmd_rotor");

  const double targetPosition{1.0};
  msgs::Double msg;
  msg.set_data(targetPosition);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);

  // joint2 should not move
  EXPECT_NEAR(0, joint2Position.at(0), TOL);

  // joint21 should be at target position
  EXPECT_NEAR(targetPosition, joint21Position.at(0), TOL);
}

/////////////////////////////////////////////////
// Tests that the JointPositionController respects the maximum command
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointPositonVelocityCommandWithMax))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "joint_position_controller_velocity.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j2";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));
        // Create a JointPosition component if it doesn't exist. This signals
        // physics system to populate the component
        if (nullptr == _ecm.Component<components::JointPosition>(joint))
        {
          _ecm.CreateComponent(joint, components::JointPosition());
        }
      });

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
                const components::Joint *,
                const components::Name *_name,
                const components::JointPosition *_position) -> bool
            {
              if(_name->Data() == jointName)
                currentPosition = _position->Data();
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // joint pos starts at 0
  const std::size_t initIters = 1;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, currentPosition.at(0), TOL);

  // joint moves to initial_position at -2.0
  const std::size_t initPosIters = 2;
  server.Run(true, initPosIters, false);
  double expectedInitialPosition = -2.0;
  EXPECT_NEAR(expectedInitialPosition, currentPosition.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_position_controller_test_with_max/joint/j2/0/cmd_pos");

  const double targetPosition{2.0};
  msgs::Double msg;
  msg.set_data(targetPosition);

  int sleep{0};
  int maxSleep{30};
  for (; !pub.HasConnections() && sleep < maxSleep; ++sleep) {
    std::this_thread::sleep_for(100ms);
  }

  pub.Publish(msg);

  // Wait for the message to be published
  std::this_thread::sleep_for(1ms);

  const std::size_t testInitialIters = 1;
  server.Run(true, testInitialIters , false);

  // We should not have reached our target yet.
  EXPECT_GT(fabs(currentPosition.at(0) - targetPosition), TOL);

  // Eventually reach target
  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);
  EXPECT_NEAR(currentPosition.at(0), targetPosition, TOL);
}

/// \brief Exercise the loaded controller with prescribed feedback, without
/// physics changing the position or clearing the force command between steps.
class JointPositionDerivativeTest
    : public JointPositionControllerTestFixture
{
  /// \brief Load and configure the plugin with optional additional parameters.
  protected: void Initialize(const std::string &_options = "")
  {
    const std::string sdfString = R"(
      <sdf version='1.10'>
        <world name='default'>
          <model name='derivative_test'>
            <link name='link'/>
            <joint name='joint' type='universal'>
              <parent>world</parent>
              <child>link</child>
              <axis><xyz>1 0 0</xyz></axis>
              <axis2><xyz>0 1 0</xyz></axis2>
            </joint>
            <plugin filename='gz-sim-joint-position-controller-system'
                    name='gz::sim::systems::JointPositionController'>
              <joint_name>joint</joint_name>
              <joint_index>1</joint_index>
              <p_gain>2</p_gain>
              <i_gain>0</i_gain>
              <d_gain>3</d_gain>
              <cmd_min>-10</cmd_min>
              <cmd_max>10</cmd_max>
              <initial_position>1</initial_position>
              <topic>/derivative_test/cmd_pos</topic>)" + _options + R"(
            </plugin>
          </model>
        </world>
      </sdf>)";

    Entity model;
    sdf::Plugin pluginSdf;
    test::LoadModelContext(sdfString, this->ecm, this->eventMgr,
        model, pluginSdf);
    this->joint = this->ecm.EntityByComponents(components::Joint(),
        components::Name("joint"));
    ASSERT_NE(kNullEntity, this->joint);

    auto plugin = this->loader.LoadPlugin(pluginSdf);
    ASSERT_TRUE(plugin.has_value());
    this->system = plugin.value();
    auto configure = this->system->QueryInterface<ISystemConfigure>();
    ASSERT_NE(nullptr, configure);
    configure->Configure(model, pluginSdf.Element(), this->ecm, this->eventMgr);
    auto parameters =
        this->system->QueryInterface<ISystemConfigureParameters>();
    ASSERT_NE(nullptr, parameters);
    parameters->ConfigureParameters(this->registry, this->ecm);
    ASSERT_NE(nullptr, this->system->QueryInterface<ISystemPreUpdate>());

    // Deliberately give the other axis a different position.
    this->ecm.SetComponentData<components::JointPosition>(this->joint, {5, 0});
    this->info.dt = std::chrono::milliseconds(1);
    this->info.paused = false;
  }

  /// \brief Run one controller update with the current feedback.
  protected: void Step()
  {
    this->system->QueryInterface<ISystemPreUpdate>()->PreUpdate(
        this->info, this->ecm);
  }

  /// \brief Check the force produced by the controller.
  protected: void ExpectForce(double _expected)
  {
    auto force = this->ecm.Component<components::JointForceCmd>(this->joint);
    ASSERT_NE(nullptr, force);
    ASSERT_EQ(1u, force->Data().size());
    EXPECT_DOUBLE_EQ(_expected, force->Data()[0]);
  }

  protected: EntityComponentManager ecm;
  protected: EventManager eventMgr;
  protected: transport::parameters::ParametersRegistry registry{
      "/derivative_test/parameters"};
  protected: SystemLoader loader;
  protected: SystemPluginPtr system;
  protected: Entity joint{kNullEntity};
  protected: UpdateInfo info;
};

/////////////////////////////////////////////////
TEST_F(JointPositionDerivativeTest, DerivativeOnErrorByDefault)
{
  ASSERT_NO_FATAL_FAILURE(this->Initialize());
  this->Step();
  this->ExpectForce(10);
  this->Step();
  this->ExpectForce(2);
  EXPECT_EQ(nullptr,
      this->ecm.Component<components::JointVelocity>(this->joint));
}

/////////////////////////////////////////////////
TEST_F(JointPositionDerivativeTest, DerivativeOnErrorExplicit)
{
  ASSERT_NO_FATAL_FAILURE(this->Initialize(
      "<derivative_on_measurement>false</derivative_on_measurement>"));
  this->Step();
  this->ExpectForce(10);
  this->Step();
  this->ExpectForce(2);
}

/////////////////////////////////////////////////
TEST_F(JointPositionDerivativeTest, MeasurementAvoidsTargetKick)
{
  using namespace std::chrono_literals;
  ASSERT_NO_FATAL_FAILURE(this->Initialize(
      "<derivative_on_measurement>true</derivative_on_measurement>"));
  this->ecm.SetComponentData<components::JointVelocity>(this->joint, {100, 0});
  this->Step();
  this->ExpectForce(2);

  transport::Node node;
  auto pub = node.Advertise<msgs::Double>("/derivative_test/cmd_pos");
  for (int i = 0; i < 500 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(10ms);
  ASSERT_TRUE(pub.HasConnections());
  msgs::Double msg;
  double previousForce = 2.0;
  for (double target : {2.0, -1.0, 0.5})
  {
    msg.set_data(target);
    ASSERT_TRUE(pub.Publish(msg));
    const double expectedForce = 2.0 * target;

    // Check every update, including command delivery, for a D-term spike.
    bool received = false;
    for (int i = 0; i < 500 && !received; ++i)
    {
      this->Step();
      auto force = this->ecm.Component<components::JointForceCmd>(this->joint);
      ASSERT_NE(nullptr, force);
      ASSERT_EQ(1u, force->Data().size());
      EXPECT_TRUE(math::equal(force->Data()[0], previousForce) ||
          math::equal(force->Data()[0], expectedForce));
      received = math::equal(force->Data()[0], expectedForce);
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(received);

    // Hold each target for 20 physics steps, as in a 50 Hz policy at 1 kHz.
    for (int i = 0; i < 20; ++i)
    {
      this->Step();
      this->ExpectForce(expectedForce);
    }
    previousForce = expectedForce;
  }
}

/////////////////////////////////////////////////
TEST_F(JointPositionDerivativeTest, MeasurementDampingAndCommandLimits)
{
  ASSERT_NO_FATAL_FAILURE(this->Initialize(
      "<derivative_on_measurement>true</derivative_on_measurement>"));
  // Position stays fixed so finite differences cannot produce these forces.
  // The selected axis provides negative velocity feedback in both directions.
  for (const auto &[velocity, force] :
      std::vector<std::pair<double, double>>{
          {2, -4}, {-2, 8}, {100, -10}, {-100, 10}})
  {
    this->ecm.SetComponentData<components::JointVelocity>(
        this->joint, {100, velocity});
    this->Step();
    this->ExpectForce(force);
  }
}

/////////////////////////////////////////////////
TEST_F(JointPositionDerivativeTest, MeasurementWaitsForVelocityAxis)
{
  ASSERT_NO_FATAL_FAILURE(this->Initialize(
      "<derivative_on_measurement>true</derivative_on_measurement>"));
  this->info.paused = true;
  this->Step();
  EXPECT_EQ(nullptr,
      this->ecm.Component<components::JointVelocity>(this->joint));
  this->info.paused = false;
  this->Step();
  auto velocity = this->ecm.Component<components::JointVelocity>(this->joint);
  ASSERT_NE(nullptr, velocity);
  EXPECT_TRUE(velocity->Data().empty());
  EXPECT_EQ(nullptr,
      this->ecm.Component<components::JointForceCmd>(this->joint));
  this->Step();
  EXPECT_EQ(nullptr,
      this->ecm.Component<components::JointForceCmd>(this->joint));

  this->ecm.SetComponentData<components::JointVelocity>(this->joint, {100});
  this->Step();
  EXPECT_EQ(nullptr,
      this->ecm.Component<components::JointForceCmd>(this->joint));
  this->ecm.SetComponentData<components::JointVelocity>(this->joint, {100, 2});
  this->Step();
  this->ExpectForce(-4);
}

/////////////////////////////////////////////////
TEST_F(JointPositionDerivativeTest, MeasurementPreservesIntegralAndOffset)
{
  ASSERT_NO_FATAL_FAILURE(this->Initialize(
      "<derivative_on_measurement>true</derivative_on_measurement>"
      "<cmd_offset>0.5</cmd_offset>"));
  this->ecm.SetComponentData<components::JointVelocity>(this->joint, {100, 0});

  msgs::Double gain;
  gain.set_data(5);
  auto result = this->registry.SetParameter("JointPositionController." +
      scopedName(this->joint, this->ecm, ".", false) + ".i_gain", gain);
  ASSERT_EQ(transport::parameters::ParameterResultType::Success,
      result.ResultType());

  // Each 1 ms step integrates the position error, while offset stays constant.
  for (int i = 1; i <= 3; ++i)
  {
    this->Step();
    this->ExpectForce(2.5 + 0.005 * i);
  }
}

/////////////////////////////////////////////////
TEST_F(JointPositionDerivativeTest, VelocityCommandsIgnoreDerivativeOption)
{
  ASSERT_NO_FATAL_FAILURE(this->Initialize(
      "<derivative_on_measurement>true</derivative_on_measurement>"
      "<use_velocity_commands>true</use_velocity_commands>"));
  this->Step();
  auto velocity =
      this->ecm.Component<components::JointVelocityCmd>(this->joint);
  ASSERT_NE(nullptr, velocity);
  ASSERT_EQ(1u, velocity->Data().size());
  EXPECT_DOUBLE_EQ(10, velocity->Data()[0]);
  EXPECT_EQ(nullptr,
      this->ecm.Component<components::JointVelocity>(this->joint));
  EXPECT_EQ(nullptr,
      this->ecm.Component<components::JointForceCmd>(this->joint));
}

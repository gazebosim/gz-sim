/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class SensorIntegrationTest : public InternalFixture<::testing::Test>
{
};

//////////////////////////////////////////////////
TEST_F(SensorIntegrationTest, Valid)
{
  EntityComponentManager ecm;

  // No ID
  {
    Sensor sensor;
    EXPECT_FALSE(sensor.Valid(ecm));
  }

  // Missing sensor component
  {
    auto id = ecm.CreateEntity();
    Sensor sensor(id);
    EXPECT_FALSE(sensor.Valid(ecm));
  }

  // Valid
  {
    auto id = ecm.CreateEntity();
    ecm.CreateComponent<components::Sensor>(id, components::Sensor());

    Sensor sensor(id);
    EXPECT_TRUE(sensor.Valid(ecm));
  }
}

//////////////////////////////////////////////////
TEST_F(SensorIntegrationTest, Name)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Sensor>(id, components::Sensor());

  Sensor sensor(id);

  // No name
  EXPECT_EQ(std::nullopt, sensor.Name(ecm));

  // Add name
  ecm.CreateComponent<components::Name>(id, components::Name("sensor_name"));
  EXPECT_EQ("sensor_name", sensor.Name(ecm));
}

//////////////////////////////////////////////////
TEST_F(SensorIntegrationTest, Pose)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Sensor>(id, components::Sensor());

  Sensor sensor(id);

  // No pose
  EXPECT_EQ(std::nullopt, sensor.Pose(ecm));

  // Add pose
  math::Pose3d pose(1, 2, 3, 0.1, 0.2, 0.3);
  ecm.CreateComponent<components::Pose>(id,
      components::Pose(pose));
  EXPECT_EQ(pose, sensor.Pose(ecm));
}

//////////////////////////////////////////////////
TEST_F(SensorIntegrationTest, Topic)
{
  EntityComponentManager ecm;

  auto eSensor = ecm.CreateEntity();
  ecm.CreateComponent(eSensor, components::Sensor());

  Sensor sensor(eSensor);

  std::string topic = "sensor_topic";
  ecm.CreateComponent<components::SensorTopic>(eSensor,
      components::SensorTopic(topic));

  EXPECT_EQ(topic, sensor.Topic(ecm));
}

//////////////////////////////////////////////////
TEST_F(SensorIntegrationTest, Parent)
{
  EntityComponentManager ecm;

  {
    // Link as parent
    auto eLink = ecm.CreateEntity();
    ecm.CreateComponent(eLink, components::Link());
    auto eSensor = ecm.CreateEntity();
    ecm.CreateComponent(eSensor, components::Sensor());

    Sensor sensor(eSensor);
    EXPECT_EQ(eSensor, sensor.Entity());
    EXPECT_FALSE(sensor.Parent(ecm).has_value());

    ecm.CreateComponent<components::ParentEntity>(eSensor,
        components::ParentEntity(eLink));
    ASSERT_TRUE(sensor.Valid(ecm));

    // Check parent link
    EXPECT_EQ(eLink, ecm.ParentEntity(eSensor));
    auto parentLink = sensor.Parent(ecm);
    EXPECT_EQ(eLink, parentLink);
  }

  {
    // Joint as parent
    auto eJoint = ecm.CreateEntity();
    ecm.CreateComponent(eJoint, components::Joint());
    auto eSensor = ecm.CreateEntity();
    ecm.CreateComponent(eSensor, components::Sensor());

    Sensor sensor(eSensor);
    EXPECT_EQ(eSensor, sensor.Entity());
    EXPECT_FALSE(sensor.Parent(ecm).has_value());

    ecm.CreateComponent<components::ParentEntity>(eSensor,
        components::ParentEntity(eJoint));
    ASSERT_TRUE(sensor.Valid(ecm));

    // Check parent joint
    EXPECT_EQ(eJoint, ecm.ParentEntity(eSensor));
    auto parentJoint = sensor.Parent(ecm);
    EXPECT_EQ(eJoint, parentJoint);
  }
}

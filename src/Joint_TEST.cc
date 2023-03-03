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

#include <cstddef>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Joint.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Sensor.hh"

/////////////////////////////////////////////////
TEST(JointTest, Constructor)
{
  ignition::gazebo::Joint jointNull;
  EXPECT_EQ(ignition::gazebo::kNullEntity, jointNull.Entity());

  ignition::gazebo::Entity id(3);
  ignition::gazebo::Joint joint(id);

  EXPECT_EQ(id, joint.Entity());
}

/////////////////////////////////////////////////
TEST(JointTest, CopyConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Joint joint(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  ignition::gazebo::Joint jointCopy(joint); // NOLINT
  EXPECT_EQ(joint.Entity(), jointCopy.Entity());
}

/////////////////////////////////////////////////
TEST(JointTest, CopyAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Joint joint(id);

  ignition::gazebo::Joint jointCopy;
  jointCopy = joint;
  EXPECT_EQ(joint.Entity(), jointCopy.Entity());
}

/////////////////////////////////////////////////
TEST(JointTest, MoveConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Joint joint(id);

  ignition::gazebo::Joint jointMoved(std::move(joint));
  EXPECT_EQ(id, jointMoved.Entity());
}

/////////////////////////////////////////////////
TEST(JointTest, MoveAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Joint joint(id);

  ignition::gazebo::Joint jointMoved;
  jointMoved = std::move(joint);
  EXPECT_EQ(id, jointMoved.Entity());
}

/////////////////////////////////////////////////
TEST(JointTest, Sensors)
{
  // jointA
  //  - sensorAA
  //  - sensorAB
  //
  // jointC

  ignition::gazebo::EntityComponentManager ecm;

  // Joint A
  auto jointAEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointAEntity, ignition::gazebo::components::Joint());
  ecm.CreateComponent(jointAEntity,
      ignition::gazebo::components::Name("jointA_name"));

  // Sensor AA - Child of Joint A
  auto sensorAAEntity = ecm.CreateEntity();
  ecm.CreateComponent(sensorAAEntity, ignition::gazebo::components::Sensor());
  ecm.CreateComponent(sensorAAEntity,
      ignition::gazebo::components::Name("sensorAA_name"));
  ecm.CreateComponent(sensorAAEntity,
      ignition::gazebo::components::ParentEntity(jointAEntity));

  // Sensor AB - Child of Joint A
  auto sensorABEntity = ecm.CreateEntity();
  ecm.CreateComponent(sensorABEntity, ignition::gazebo::components::Sensor());
  ecm.CreateComponent(sensorABEntity,
      ignition::gazebo::components::Name("sensorAB_name"));
  ecm.CreateComponent(sensorABEntity,
      ignition::gazebo::components::ParentEntity(jointAEntity));

  // Joint C
  auto jointCEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointCEntity, ignition::gazebo::components::Joint());
  ecm.CreateComponent(jointCEntity,
      ignition::gazebo::components::Name("jointC_name"));

  std::size_t foundSensors = 0;

  ignition::gazebo::Joint jointA(jointAEntity);
  auto sensors = jointA.Sensors(ecm);
  EXPECT_EQ(2u, sensors.size());
  for (const auto &sensor : sensors)
  {
    if (sensor == sensorAAEntity || sensor == sensorABEntity)
      foundSensors++;
  }
  EXPECT_EQ(foundSensors, sensors.size());

  ignition::gazebo::Joint jointC(jointCEntity);
  EXPECT_EQ(0u, jointC.Sensors(ecm).size());
}

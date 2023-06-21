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

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Joint.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"

/////////////////////////////////////////////////
TEST(JointTest, Constructor)
{
  gz::sim::Joint jointNull;
  EXPECT_EQ(gz::sim::kNullEntity, jointNull.Entity());

  gz::sim::Entity id(3);
  gz::sim::Joint joint(id);

  EXPECT_EQ(id, joint.Entity());
}

/////////////////////////////////////////////////
TEST(JointTest, CopyConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Joint joint(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  gz::sim::Joint jointCopy(joint); // NOLINT
  EXPECT_EQ(joint.Entity(), jointCopy.Entity());
}

/////////////////////////////////////////////////
TEST(JointTest, CopyAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Joint joint(id);

  gz::sim::Joint jointCopy;
  jointCopy = joint;
  EXPECT_EQ(joint.Entity(), jointCopy.Entity());
}

/////////////////////////////////////////////////
TEST(JointTest, MoveConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Joint joint(id);

  gz::sim::Joint jointMoved(std::move(joint));
  EXPECT_EQ(id, jointMoved.Entity());
}

/////////////////////////////////////////////////
TEST(JointTest, MoveAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Joint joint(id);

  gz::sim::Joint jointMoved;
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

  gz::sim::EntityComponentManager ecm;

  // Joint A
  auto jointAEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointAEntity, gz::sim::components::Joint());
  ecm.CreateComponent(jointAEntity,
      gz::sim::components::Name("jointA_name"));

  // Sensor AA - Child of Joint A
  auto sensorAAEntity = ecm.CreateEntity();
  ecm.CreateComponent(sensorAAEntity, gz::sim::components::Sensor());
  ecm.CreateComponent(sensorAAEntity,
      gz::sim::components::Name("sensorAA_name"));
  ecm.CreateComponent(sensorAAEntity,
      gz::sim::components::ParentEntity(jointAEntity));

  // Sensor AB - Child of Joint A
  auto sensorABEntity = ecm.CreateEntity();
  ecm.CreateComponent(sensorABEntity, gz::sim::components::Sensor());
  ecm.CreateComponent(sensorABEntity,
      gz::sim::components::Name("sensorAB_name"));
  ecm.CreateComponent(sensorABEntity,
      gz::sim::components::ParentEntity(jointAEntity));

  // Joint C
  auto jointCEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointCEntity, gz::sim::components::Joint());
  ecm.CreateComponent(jointCEntity,
      gz::sim::components::Name("jointC_name"));

  std::size_t foundSensors = 0;

  gz::sim::Joint jointA(jointAEntity);
  auto sensors = jointA.Sensors(ecm);
  EXPECT_EQ(2u, sensors.size());
  for (const auto &sensor : sensors)
  {
    if (sensor == sensorAAEntity || sensor == sensorABEntity)
      foundSensors++;
  }
  EXPECT_EQ(foundSensors, sensors.size());

  gz::sim::Joint jointC(jointCEntity);
  EXPECT_EQ(0u, jointC.Sensors(ecm).size());
}

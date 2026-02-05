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
#include "gz/sim/components/JointAxis.hh"
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

/////////////////////////////////////////////////
TEST(JointTest, MaxVelocityLimitsMultiAxis)
{
  gz::sim::EntityComponentManager ecm;

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gz::sim::components::Joint());

  gz::sim::components::JointAxis axis1;
  axis1.Data().SetMaxVelocity(5.0);
  ecm.CreateComponent(jointEntity, axis1);

  gz::sim::components::JointAxis2 axis2;
  axis2.Data().SetMaxVelocity(2.0);
  ecm.CreateComponent(jointEntity, axis2);

  gz::sim::Joint joint(jointEntity);

  auto limits = joint.MaxVelocityLimits(ecm);

  ASSERT_TRUE(limits.has_value());
  ASSERT_EQ(limits->size(), 2u);

  EXPECT_DOUBLE_EQ((*limits)[0], 5.0);
  EXPECT_DOUBLE_EQ((*limits)[1], 2.0);
}

/////////////////////////////////////////////////
TEST(JointTest, MaxVelocityLimitsSingleAxis)
{
  gz::sim::EntityComponentManager ecm;

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gz::sim::components::Joint());

  gz::sim::components::JointAxis axis;
  axis.Data().SetMaxVelocity(3.0);
  ecm.CreateComponent(jointEntity, axis);

  gz::sim::Joint joint(jointEntity);

  auto limits = joint.MaxVelocityLimits(ecm);

  ASSERT_TRUE(limits.has_value());
  ASSERT_EQ(limits->size(), 1u);
  EXPECT_DOUBLE_EQ((*limits)[0],  3.0);
}

/////////////////////////////////////////////////
TEST(JointTest, MaxVelocityLimitsNoAxis)
{
  gz::sim::EntityComponentManager ecm;

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gz::sim::components::Joint());

  gz::sim::Joint joint(jointEntity);

  auto limits = joint.MaxVelocityLimits(ecm);

  EXPECT_FALSE(limits.has_value());
}

/////////////////////////////////////////////////
TEST(JointTest, EffortLimitsMultiAxis)
{
  gz::sim::EntityComponentManager ecm;

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gz::sim::components::Joint());

  gz::sim::components::JointAxis axis1;
  axis1.Data().SetEffort(5.0);
  ecm.CreateComponent(jointEntity, axis1);

  gz::sim::components::JointAxis2 axis2;
  axis2.Data().SetEffort(2.0);
  ecm.CreateComponent(jointEntity, axis2);

  gz::sim::Joint joint(jointEntity);

  auto limits = joint.EffortLimits(ecm);

  ASSERT_TRUE(limits.has_value());
  ASSERT_EQ(limits->size(), 2u);

  EXPECT_DOUBLE_EQ((*limits)[0], 5.0);
  EXPECT_DOUBLE_EQ((*limits)[1], 2.0);
}

/////////////////////////////////////////////////
TEST(JointTest, EffortLimitsSingleAxis)
{
  gz::sim::EntityComponentManager ecm;

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gz::sim::components::Joint());

  gz::sim::components::JointAxis axis;
  axis.Data().SetEffort(3.0);
  ecm.CreateComponent(jointEntity, axis);

  gz::sim::Joint joint(jointEntity);

  auto limits = joint.EffortLimits(ecm);

  ASSERT_TRUE(limits.has_value());
  ASSERT_EQ(limits->size(), 1u);
  EXPECT_DOUBLE_EQ((*limits)[0], 3.0);
}

/////////////////////////////////////////////////
TEST(JointTest, EffortLimitsNoAxis)
{
  gz::sim::EntityComponentManager ecm;

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gz::sim::components::Joint());

  gz::sim::Joint joint(jointEntity);

  auto limits = joint.EffortLimits(ecm);

  EXPECT_FALSE(limits.has_value());
}

/////////////////////////////////////////////////
TEST(JointTest, PositionLimitsMultiAxis)
{
  gz::sim::EntityComponentManager ecm;

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gz::sim::components::Joint());

  gz::sim::components::JointAxis axis1;
  axis1.Data().SetLower(-1.0);
  axis1.Data().SetUpper(2.0);
  ecm.CreateComponent(jointEntity, axis1);

  gz::sim::components::JointAxis2 axis2;
  axis2.Data().SetLower(-3.0);
  axis2.Data().SetUpper(4.0);
  ecm.CreateComponent(jointEntity, axis2);

  gz::sim::Joint joint(jointEntity);

  auto limits = joint.PositionLimits(ecm);

  ASSERT_TRUE(limits.has_value());
  ASSERT_EQ(limits->size(), 2u);

  EXPECT_DOUBLE_EQ((*limits)[0].X(), -1.0);
  EXPECT_DOUBLE_EQ((*limits)[0].Y(), 2.0);
  EXPECT_DOUBLE_EQ((*limits)[1].X(), -3.0);
  EXPECT_DOUBLE_EQ((*limits)[1].Y(), 4.0);
}

/////////////////////////////////////////////////
TEST(JointTest, PositionLimitsSingleAxis)
{
  gz::sim::EntityComponentManager ecm;

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gz::sim::components::Joint());

  gz::sim::components::JointAxis axis;
  axis.Data().SetLower(-0.5);
  axis.Data().SetUpper(0.7);
  ecm.CreateComponent(jointEntity, axis);

  gz::sim::Joint joint(jointEntity);

  auto limits = joint.PositionLimits(ecm);

  ASSERT_TRUE(limits.has_value());
  ASSERT_EQ(limits->size(), 1u);
  EXPECT_DOUBLE_EQ((*limits)[0].X(), -0.5);
  EXPECT_DOUBLE_EQ((*limits)[0].Y(), 0.7);
}

/////////////////////////////////////////////////
TEST(JointTest, PositionLimitsNoAxis)
{
  gz::sim::EntityComponentManager ecm;

  auto jointEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointEntity, gz::sim::components::Joint());

  gz::sim::Joint joint(jointEntity);

  auto limits = joint.PositionLimits(ecm);

  EXPECT_FALSE(limits.has_value());
}

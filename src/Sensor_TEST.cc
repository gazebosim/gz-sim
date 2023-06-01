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

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Sensor.hh"

/////////////////////////////////////////////////
TEST(SensorTest, Constructor)
{
  ignition::gazebo::Sensor sensorNull;
  EXPECT_EQ(ignition::gazebo::kNullEntity, sensorNull.Entity());

  ignition::gazebo::Entity id(3);
  ignition::gazebo::Sensor sensor(id);

  EXPECT_EQ(id, sensor.Entity());
}

/////////////////////////////////////////////////
TEST(SensorTest, CopyConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Sensor sensor(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  ignition::gazebo::Sensor sensorCopy(sensor); // NOLINT
  EXPECT_EQ(sensor.Entity(), sensorCopy.Entity());
}

/////////////////////////////////////////////////
TEST(SensorTest, CopyAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Sensor sensor(id);

  ignition::gazebo::Sensor sensorCopy;
  sensorCopy = sensor;
  EXPECT_EQ(sensor.Entity(), sensorCopy.Entity());
}

/////////////////////////////////////////////////
TEST(SensorTest, MoveConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Sensor sensor(id);

  ignition::gazebo::Sensor sensorMoved(std::move(sensor));
  EXPECT_EQ(id, sensorMoved.Entity());
}

/////////////////////////////////////////////////
TEST(SensorTest, MoveAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Sensor sensor(id);

  ignition::gazebo::Sensor sensorMoved;
  sensorMoved = std::move(sensor);
  EXPECT_EQ(id, sensorMoved.Entity());
}


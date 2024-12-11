/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"

/////////////////////////////////////////////////
TEST(LinkTest, Constructor)
{
  gz::sim::Link linkNull;
  EXPECT_EQ(gz::sim::kNullEntity, linkNull.Entity());

  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  EXPECT_EQ(id, link.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, CopyConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  gz::sim::Link linkCopy(link); // NOLINT
  EXPECT_EQ(link.Entity(), linkCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, CopyAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  gz::sim::Link linkCopy;
  linkCopy = link;
  EXPECT_EQ(link.Entity(), linkCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, MoveConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  gz::sim::Link linkMoved(std::move(link));
  EXPECT_EQ(id, linkMoved.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, MoveAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  gz::sim::Link linkMoved;
  linkMoved = std::move(link);
  EXPECT_EQ(id, linkMoved.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, Sensors)
{
  // linkA
  //  - sensorAA
  //  - sensorAB
  //
  // linkC

  gz::sim::EntityComponentManager ecm;

  // Link A
  auto linkAEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkAEntity, gz::sim::components::Link());
  ecm.CreateComponent(linkAEntity,
      gz::sim::components::Name("linkA_name"));

  // Sensor AA - Child of Link A
  auto sensorAAEntity = ecm.CreateEntity();
  ecm.CreateComponent(sensorAAEntity, gz::sim::components::Sensor());
  ecm.CreateComponent(sensorAAEntity,
      gz::sim::components::Name("sensorAA_name"));
  ecm.CreateComponent(sensorAAEntity,
      gz::sim::components::ParentEntity(linkAEntity));

  // Sensor AB - Child of Link A
  auto sensorABEntity = ecm.CreateEntity();
  ecm.CreateComponent(sensorABEntity, gz::sim::components::Sensor());
  ecm.CreateComponent(sensorABEntity,
      gz::sim::components::Name("sensorAB_name"));
  ecm.CreateComponent(sensorABEntity,
      gz::sim::components::ParentEntity(linkAEntity));

  // Link C
  auto linkCEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkCEntity, gz::sim::components::Link());
  ecm.CreateComponent(linkCEntity,
      gz::sim::components::Name("linkC_name"));

  std::size_t foundSensors = 0;

  gz::sim::Link linkA(linkAEntity);
  auto sensors = linkA.Sensors(ecm);
  EXPECT_EQ(2u, sensors.size());
  for (const auto &sensor : sensors)
  {
    if (sensor == sensorAAEntity || sensor == sensorABEntity)
      foundSensors++;
  }
  EXPECT_EQ(foundSensors, sensors.size());

  EXPECT_EQ(sensorAAEntity, linkA.SensorByName(ecm, "sensorAA_name"));
  EXPECT_EQ(sensorABEntity, linkA.SensorByName(ecm, "sensorAB_name"));
  EXPECT_EQ(gz::sim::kNullEntity, linkA.SensorByName(ecm, "invalid"));

  gz::sim::Link linkC(linkCEntity);
  EXPECT_EQ(0u, linkC.Sensors(ecm).size());
  EXPECT_EQ(gz::sim::kNullEntity, linkC.SensorByName(ecm, "invalid"));
}

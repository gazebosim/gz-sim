/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <sdf/Atmosphere.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Atmosphere.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/MagneticField.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/SelfCollide.hh>
#include <ignition/gazebo/components/SourceFilePath.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/WindMode.hh>
#include <ignition/gazebo/components/World.hh>

#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class WorldIntegrationTest : public InternalFixture<::testing::Test>
{
};

//////////////////////////////////////////////////
TEST_F(WorldIntegrationTest, Valid)
{
  EntityComponentManager ecm;

  // No ID
  {
    World world;
    EXPECT_FALSE(world.Valid(ecm));
  }

  // Missing world component
  {
    auto id = ecm.CreateEntity();
    World world(id);
    EXPECT_FALSE(world.Valid(ecm));
  }

  // Valid
  {
    auto id = ecm.CreateEntity();
    ecm.CreateComponent<components::World>(id, components::World());

    World world(id);
    EXPECT_TRUE(world.Valid(ecm));
  }
}

//////////////////////////////////////////////////
TEST_F(WorldIntegrationTest, Name)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::World>(id, components::World());

  World world(id);

  EXPECT_EQ(std::nullopt, world.Name(ecm));

  ecm.CreateComponent<components::Name>(id, components::Name("world_name"));
  EXPECT_EQ("world_name", world.Name(ecm));
}

//////////////////////////////////////////////////
TEST_F(WorldIntegrationTest, Atmosphere)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::World>(id, components::World());

  World world(id);

  EXPECT_EQ(std::nullopt, world.Atmosphere(ecm));

  ecm.CreateComponent<components::Atmosphere>(id,
      components::Atmosphere(sdf::Atmosphere()));
  auto atmosphere = world.Atmosphere(ecm).value();
  EXPECT_EQ(math::Temperature(288.15), atmosphere.Temperature());
}

//////////////////////////////////////////////////
TEST_F(WorldIntegrationTest, Gravity)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::World>(id, components::World());

  World world(id);

  EXPECT_EQ(std::nullopt, world.Gravity(ecm));

  ecm.CreateComponent<components::Gravity>(id, components::Gravity({1, 2, 3}));
  EXPECT_EQ(math::Vector3d(1, 2, 3), world.Gravity(ecm));
}

//////////////////////////////////////////////////
TEST_F(WorldIntegrationTest, MagneticField)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::World>(id, components::World());

  World world(id);

  // No gravity
  EXPECT_EQ(std::nullopt, world.MagneticField(ecm));

  // Add gravity
  ecm.CreateComponent<components::MagneticField>(id,
      components::MagneticField({1, 2, 3}));
  EXPECT_EQ(math::Vector3d(1, 2, 3), world.MagneticField(ecm));
}

//////////////////////////////////////////////////
TEST_F(WorldIntegrationTest, ModelByName)
{
  EntityComponentManager ecm;

  // World
  auto eWorld = ecm.CreateEntity();
  World world(eWorld);
  EXPECT_EQ(eWorld, world.Entity());
  EXPECT_EQ(0u, world.ModelCount(ecm));

  // Model
  auto eModel = ecm.CreateEntity();
  ecm.CreateComponent<components::Model>(eModel, components::Model());
  ecm.CreateComponent<components::ParentEntity>(eModel,
      components::ParentEntity(eWorld));
  ecm.CreateComponent<components::Name>(eModel,
      components::Name("model_name"));

  // Check world
  EXPECT_EQ(eModel, world.ModelByName(ecm, "model_name"));
  EXPECT_EQ(1u, world.ModelCount(ecm));
}

//////////////////////////////////////////////////
TEST_F(WorldIntegrationTest, LightByName)
{
  EntityComponentManager ecm;

  // World
  auto eWorld = ecm.CreateEntity();
  World world(eWorld);
  EXPECT_EQ(eWorld, world.Entity());
  EXPECT_EQ(0u, world.LightCount(ecm));

  // Light
  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(eLight, components::Light());
  ecm.CreateComponent<components::ParentEntity>(eLight,
      components::ParentEntity(eWorld));
  ecm.CreateComponent<components::Name>(eLight,
      components::Name("light_name"));

  // Check world
  EXPECT_EQ(eLight, world.LightByName(ecm, "light_name"));
  EXPECT_EQ(1u, world.LightCount(ecm));
}

//////////////////////////////////////////////////
TEST_F(WorldIntegrationTest, ActorByName)
{
  EntityComponentManager ecm;

  // World
  auto eWorld = ecm.CreateEntity();
  World world(eWorld);
  EXPECT_EQ(eWorld, world.Entity());
  EXPECT_EQ(0u, world.ActorCount(ecm));

  // Actor
  auto eActor = ecm.CreateEntity();
  ecm.CreateComponent<components::Actor>(eActor, components::Actor());
  ecm.CreateComponent<components::ParentEntity>(eActor,
      components::ParentEntity(eWorld));
  ecm.CreateComponent<components::Name>(eActor,
      components::Name("actor_name"));

  // Check world
  EXPECT_EQ(eActor, world.ActorByName(ecm, "actor_name"));
  EXPECT_EQ(1u, world.ActorCount(ecm));
}


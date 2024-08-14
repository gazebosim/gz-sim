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

#include <string>
#include <memory>

#include <sdf/Root.hh>
#include <sdf/Model.hh>

#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "test_config.hh"

#include "gz/sim/components/Material.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class MaterialTest : public InternalFixture<::testing::Test>
{
  public: ::testing::AssertionResult StartServer(
    const gz::sim::ServerConfig &_serverConfig =
      gz::sim::ServerConfig())
  {
    this->relay = std::make_unique<test::Relay>();
    this->server = std::make_unique<Server>(_serverConfig);
    using namespace std::chrono_literals;
    this->server->SetUpdatePeriod(0ns);

    EXPECT_FALSE(this->server->Running());
    EXPECT_FALSE(*this->server->Running(0));
    // A pointer to the ecm. This will be valid once we run the mock system
    relay->OnPreUpdate(
        [this](const UpdateInfo &, EntityComponentManager &_ecm)
        {
          this->ecm = &_ecm;
        });

    this->server->AddSystem(this->relay->systemPtr);
    this->server->Run(true, 1, false);
    if (nullptr == this->ecm)
    {
      return ::testing::AssertionFailure()
        << "Failed to create EntityComponentManager";
    }

    this->creator =
        std::make_unique<SdfEntityCreator>(*this->ecm, this->dummyEventMgr);
    return ::testing::AssertionSuccess();
  }

  public: void SpawnModelSDF(const std::string &_sdfString)
  {
    ASSERT_NE(this->server, nullptr);
    ASSERT_NE(this->creator, nullptr);

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(_sdfString);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, root.Model());

    Entity modelEntity = this->creator->CreateEntities(root.Model());
    Entity worldEntity = this->ecm->EntityByComponents(components::World());
    this->creator->SetParent(modelEntity, worldEntity);
  }

  public: Model GetModel(const std::string &_name)
  {
    return Model(this->ecm->EntityByComponents(
        components::Model(), components::Name(_name)));
  }

  public: std::unique_ptr<test::Relay> relay;
  public: std::unique_ptr<Server> server;
  public: EntityComponentManager *ecm {nullptr};
  public: EventManager dummyEventMgr;
  public: std::unique_ptr<SdfEntityCreator> creator;
};

// Check for blue color parsed
TEST_F(MaterialTest, SolidColor)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.11">
      <model name="material_shapes">
        <pose>0 0 0.5 0 0 0</pose>
        <link name="box">
        <pose>0 -1.5 0 0 0 0</pose>
        <visual name="box_visual">
            <geometry>
            <box>
                <size>1 1 1</size>
            </box>
            </geometry>
            <material>
            <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Blue</name>
            </script>
            </material>
        </visual>
        </link>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("material_shapes");
  ASSERT_TRUE(model.Valid(*this->ecm));

  auto boxVisualEntity =
   this->ecm->EntityByComponents(components::Name("box_visual"));
  ASSERT_NE(kNullEntity, boxVisualEntity);

  auto boxVisualComp =
   this->ecm->Component<components::Material>(boxVisualEntity);
  EXPECT_EQ(math::Color(0.0f, 0.0f, 1.0f, 1.0f),
            boxVisualComp->Data().Ambient());
  EXPECT_EQ(math::Color(0.0f, 0.0f, 1.0f, 1.0f),
            boxVisualComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.1f, 0.1f, 0.1f, 1.0f),
            boxVisualComp->Data().Specular());
}

// Other than solid colors parsed black by default
TEST_F(MaterialTest, OtherColor)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.11">
      <model name="material_shapes">
        <pose>0 0 0.5 0 0 0</pose>
        <link name="box">
        <pose>0 -1.5 0 0 0 0</pose>
        <visual name="box_visual">
            <geometry>
            <box>
                <size>1 1 1</size>
            </box>
            </geometry>
            <material>
            <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/BlueGlow</name>
            </script>
            </material>
        </visual>
        </link>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("material_shapes");
  ASSERT_TRUE(model.Valid(*this->ecm));

  auto boxVisualEntity =
    this->ecm->EntityByComponents(components::Name("box_visual"));
  ASSERT_NE(kNullEntity, boxVisualEntity);

  // Default to black color
  auto boxVisualComp =
    this->ecm->Component<components::Material>(boxVisualEntity);
  EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Ambient());
  EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Specular());
}

// Warning for custom scripts not supported, default to black
TEST_F(MaterialTest, CustomScript)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.11">
      <model name="material_shapes">
        <pose>0 0 0.5 0 0 0</pose>
        <link name="box">
        <pose>0 -1.5 0 0 0 0</pose>
        <visual name="box_visual">
            <geometry>
            <box>
                <size>1 1 1</size>
            </box>
            </geometry>
            <material>
            <script>
                <uri>file://media/materials/scripts/custom.material</uri>
                <name>Blue</name>
            </script>
            </material>
        </visual>
        </link>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("material_shapes");
  ASSERT_TRUE(model.Valid(*this->ecm));

  auto boxVisualEntity =
    this->ecm->EntityByComponents(components::Name("box_visual"));
  ASSERT_NE(kNullEntity, boxVisualEntity);

  // Default to black color
  auto boxVisualComp =
    this->ecm->Component<components::Material>(boxVisualEntity);
  EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Ambient());
  EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Specular());
}

// Warning for invalid name, default to black
TEST_F(MaterialTest, InvalidColor)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.11">
      <model name="material_shapes">
        <pose>0 0 0.5 0 0 0</pose>
        <link name="box">
        <pose>0 -1.5 0 0 0 0</pose>
        <visual name="box_visual">
            <geometry>
            <box>
                <size>1 1 1</size>
            </box>
            </geometry>
            <material>
            <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Invalid</name>
            </script>
            </material>
        </visual>
        </link>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("material_shapes");
  ASSERT_TRUE(model.Valid(*this->ecm));

  auto boxVisualEntity =
    this->ecm->EntityByComponents(components::Name("box_visual"));
  ASSERT_NE(kNullEntity, boxVisualEntity);

  // Default to black color
  auto boxVisualComp =
    this->ecm->Component<components::Material>(boxVisualEntity);
  EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Ambient());
  EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Specular());
}

TEST_F(MaterialTest, WorldWithClassicMaterial)
{
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "classic_material.sdf"));

  std::cout << "Loading: " << serverConfig.SdfFile() << std::endl;
  this->StartServer(serverConfig);

  auto model = this->GetModel("box");
  ASSERT_TRUE(model.Valid(*this->ecm));

  auto boxVisualEntity =
    this->ecm->EntityByComponents(components::Name("box_visual"));
  ASSERT_NE(kNullEntity, boxVisualEntity);

  // Blue color
  auto boxVisualComp =
    this->ecm->Component<components::Material>(boxVisualEntity);
  EXPECT_EQ(math::Color(0.0f, 0.0f, 1.0f, 1.0f),
            boxVisualComp->Data().Ambient());
  EXPECT_EQ(math::Color(0.0f, 0.0f, 1.0f, 1.0f),
            boxVisualComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.1f, 0.1f, 0.1f, 1.0f),
            boxVisualComp->Data().Specular());
}

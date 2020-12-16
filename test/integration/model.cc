/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/LightCmd.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/SelfCollide.hh>
#include <ignition/gazebo/components/SourceFilePath.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/WindMode.hh>

#include <sdf/Light.hh>

using namespace ignition;
using namespace gazebo;

class ModelIntegrationTest : public ::testing::Test
{
  public: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }
};

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, Valid)
{
  EntityComponentManager ecm;

  // No ID
  {
    Model model;
    EXPECT_FALSE(model.Valid(ecm));
  }

  // Missing model component
  {
    auto id = ecm.CreateEntity();
    Model model(id);
    EXPECT_FALSE(model.Valid(ecm));
  }

  // Valid
  {
    auto id = ecm.CreateEntity();
    ecm.CreateComponent<components::Model>(id, components::Model());

    Model model(id);
    EXPECT_TRUE(model.Valid(ecm));
  }
}

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, Name)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Model>(id, components::Model());

  Model model(id);

  // No name
  EXPECT_TRUE(model.Name(ecm).empty());

  // Add name
  ecm.CreateComponent<components::Name>(id, components::Name("model_name"));
  EXPECT_EQ("model_name", model.Name(ecm));
}

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, Static)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Model>(id, components::Model());

  Model model(id);

  // Not static
  EXPECT_FALSE(model.Static(ecm));

  // Make static
  ecm.CreateComponent<components::Static>(id, components::Static(true));
  EXPECT_TRUE(model.Static(ecm));
}

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, SelfCollide)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Model>(id, components::Model());

  Model model(id);

  // Not self collide
  EXPECT_FALSE(model.SelfCollide(ecm));

  // Make self collide
  ecm.CreateComponent<components::SelfCollide>(id,
      components::SelfCollide(true));
  EXPECT_TRUE(model.SelfCollide(ecm));
}

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, WindMode)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Model>(id, components::Model());

  Model model(id);

  // Not static
  EXPECT_FALSE(model.WindMode(ecm));

  // Make static
  ecm.CreateComponent<components::WindMode>(id, components::WindMode(true));
  EXPECT_TRUE(model.WindMode(ecm));
}

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, SourceFilePath)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Model>(id, components::Model());

  Model model(id);

  // No path
  EXPECT_TRUE(model.SourceFilePath(ecm).empty());

  // Add path
  ecm.CreateComponent<components::SourceFilePath>(id,
      components::SourceFilePath("/tmp/path"));
  EXPECT_EQ("/tmp/path", model.SourceFilePath(ecm));
}

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, LinkByName)
{
  EntityComponentManager ecm;

  // Model
  auto eModel = ecm.CreateEntity();
  Model model(eModel);
  EXPECT_EQ(eModel, model.Entity());
  EXPECT_EQ(0u, model.LinkCount(ecm));

  // Link
  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent<components::Link>(eLink, components::Link());
  ecm.CreateComponent<components::ParentEntity>(eLink,
      components::ParentEntity(eModel));
  ecm.CreateComponent<components::Name>(eLink,
      components::Name("link_name"));

  // Check model
  EXPECT_EQ(eLink, model.LinkByName(ecm, "link_name"));
  EXPECT_EQ(1u, model.LinkCount(ecm));
}

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, JointByName)
{
  EntityComponentManager ecm;

  // Model
  auto eModel = ecm.CreateEntity();
  Model model(eModel);
  EXPECT_EQ(eModel, model.Entity());
  EXPECT_EQ(0u, model.JointCount(ecm));

  // Joint
  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent<components::Joint>(eJoint, components::Joint());
  ecm.CreateComponent<components::ParentEntity>(eJoint,
      components::ParentEntity(eModel));
  ecm.CreateComponent<components::Name>(eJoint,
      components::Name("joint_name"));

  // Check model
  EXPECT_EQ(eJoint, model.JointByName(ecm, "joint_name"));
  EXPECT_EQ(1u, model.JointCount(ecm));
}

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, SetWorldPoseCmd)
{
  EntityComponentManager ecm;

  // Model
  auto eModel = ecm.CreateEntity();
  Model model(eModel);

  auto worldPoseCmdComp = ecm.Component<components::WorldPoseCmd>(eModel);
  EXPECT_EQ(nullptr, worldPoseCmdComp);
  EXPECT_FALSE(ecm.HasOneTimeComponentChanges());

  model.SetWorldPoseCmd(ecm, math::Pose3d(1, 2, 3, 0, 0, 0));

  worldPoseCmdComp = ecm.Component<components::WorldPoseCmd>(eModel);
  ASSERT_NE(nullptr, worldPoseCmdComp);
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 0), worldPoseCmdComp->Data());
  EXPECT_TRUE(ecm.HasOneTimeComponentChanges());
}

//////////////////////////////////////////////////
TEST_F(ModelIntegrationTest, SetLightCmd)
{
  EntityComponentManager ecm;

  // Model
  auto eModel = ecm.CreateEntity();
  Model model(eModel);

  auto lightCmdComp = ecm.Component<components::LightCmd>(eModel);
  EXPECT_EQ(nullptr, lightCmdComp);
  EXPECT_FALSE(ecm.HasOneTimeComponentChanges());

  sdf::Light light;
  light.SetType(sdf::LightType::SPOT);
  light.SetName("spot");
  light.SetCastShadows(true);
  light.SetDiffuse(math::Color(0.8, 0.8, 0.8, 1));
  light.SetSpecular(math::Color(0.2, 0.2, 0.2, 1));
  light.SetDirection(math::Vector3d(0.5, 0.2, -0.9));
  light.SetAttenuationRange(1.0);
  light.SetLinearAttenuationFactor(0.5);
  light.SetConstantAttenuationFactor(0.3);
  light.SetQuadraticAttenuationFactor(4.0);
  light.SetSpotInnerAngle(math::Angle(0.1));
  light.SetSpotOuterAngle(math::Angle(0.2));
  light.SetSpotFalloff(0.001);

  model.SetLightCmd(ecm, light);

  lightCmdComp = ecm.Component<components::LightCmd>(eModel);
  ASSERT_NE(nullptr, lightCmdComp);
  EXPECT_EQ(sdf::LightType::SPOT, lightCmdComp->Data().Type());
  EXPECT_EQ("spot", lightCmdComp->Data().Name());
  EXPECT_TRUE(lightCmdComp->Data().CastShadows());
  EXPECT_EQ(math::Color(0.2, 0.2, 0.2, 1), lightCmdComp->Data().Specular());
  EXPECT_EQ(math::Color(0.8, 0.8, 0.8, 1), lightCmdComp->Data().Diffuse());
  EXPECT_EQ(math::Vector3d(0.5, 0.2, -0.9), lightCmdComp->Data().Direction());
  EXPECT_NEAR(1.0, lightCmdComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(0.5, lightCmdComp->Data().LinearAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.3, lightCmdComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(4.0, lightCmdComp->Data().QuadraticAttenuationFactor(), 0.1);
  EXPECT_EQ(math::Angle(0.1), lightCmdComp->Data().SpotInnerAngle());
  EXPECT_EQ(math::Angle(0.2), lightCmdComp->Data().SpotOuterAngle());
  EXPECT_NEAR(0.001, lightCmdComp->Data().SpotFalloff(), 0.0001);
  EXPECT_TRUE(ecm.HasOneTimeComponentChanges());
}

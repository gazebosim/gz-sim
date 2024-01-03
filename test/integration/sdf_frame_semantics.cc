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

#include <string>
#include <memory>

#include <sdf/Root.hh>
#include <sdf/Model.hh>

#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "test_config.hh"

#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class SdfFrameSemanticsTest : public InternalFixture<::testing::Test>
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

  public: sim::Model GetChildModel(Entity _parent, const std::string &_name)
  {
    return sim::Model(
      this->ecm->EntityByComponents(
        components::Model(),
        components::ParentEntity(_parent),
        components::Name(_name)));
  }

  public: math::Pose3d GetPose(Entity _entity)
  {
    auto poseComp = this->ecm->Component<components::Pose>(_entity);
    // Use EXPECT since we can't use ASSERT here
    EXPECT_NE(poseComp, nullptr);
    return poseComp->Data();
  }

  public: math::Pose3d GetWorldPose(Entity _entity)
  {
    auto poseComp = this->ecm->Component<components::WorldPose>(_entity);
    // Use EXPECT since we can't use ASSERT here
    EXPECT_NE(poseComp, nullptr);
    return poseComp->Data();
  }

  public: std::unique_ptr<test::Relay> relay;
  public: std::unique_ptr<Server> server;
  public: EntityComponentManager *ecm {nullptr};
  // We won't use the event manager but it's required to create an
  // SdfEntityCreator
  public: EventManager dummyEventMgr;
  public: std::unique_ptr<SdfEntityCreator> creator;
};

// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(SdfFrameSemanticsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LinkRelativeTo))
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>0.0 0 2 0 0 0</pose>
      <link name="L1">
        <pose>0.5 0 0 0 0 3.14159265358979</pose>
      </link>
      <link name="L2">
        <pose relative_to="L1">0.5 0 1 0 0 0</pose>
      </link>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_TRUE(model.Valid(*this->ecm));

  Entity link2 = model.LinkByName(*this->ecm, "L2");
  EXPECT_NE(link2, kNullEntity);

  // Expect the pose of L2 relative to model to be 0 0 1 0 0 pi
  math::Pose3d expRelPose(0, 0, 1, 0, 0, GZ_PI);
  math::Pose3d expWorldPose(0, 0, 3, 0, 0, GZ_PI);

  EXPECT_EQ(expRelPose, this->GetPose(link2));

  // Create WorldPose component to be populated by physics
  this->ecm->CreateComponent(link2, components::WorldPose());

  // Step once and check
  this->server->Run(true, 1, false);

  EXPECT_EQ(expRelPose, this->GetPose(link2));
  EXPECT_EQ(expWorldPose, this->GetWorldPose(link2));
}

TEST_F(SdfFrameSemanticsTest, JointRelativeTo)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <link name="L1">
        <pose>0.0 0 1 0 0 0</pose>
      </link>
      <link name="L2">
        <pose>0.0 0 2 0 0 0</pose>
      </link>
      <link name="L3">
        <pose>0.0 0 3 0 0 0</pose>
      </link>
      <joint name="J1" type="revolute">
        <parent>L1</parent>
        <child>L2</child>
      </joint>
      <joint name="J2" type="revolute">
        <pose relative_to="L2"/>
        <parent>L2</parent>
        <child>L3</child>
      </joint>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_TRUE(model.Valid(*this->ecm));

  Entity link2 = model.LinkByName(*this->ecm, "L2");
  EXPECT_NE(link2, kNullEntity);

  Entity joint1 = model.JointByName(*this->ecm, "J1");
  EXPECT_NE(joint1, kNullEntity);

  Entity joint2 = model.JointByName(*this->ecm, "J2");
  EXPECT_NE(joint2, kNullEntity);

  // Create WorldPose component to be populated by physics
  this->ecm->CreateComponent(joint1, components::WorldPose());
  this->ecm->CreateComponent(joint2, components::WorldPose());

  // Step once and check
  this->server->Run(true, 1, false);

  // Expect the pose of J1 relative to model to be the same as L2 (default
  // behavior)
  math::Pose3d expWorldPose(1, 0, 2, 0, 0, 0);

  // Expect the pose of J2 relative to model to be the same as L2 (non default
  // behavior due to "relative_to='L2'")
  // TODO(anyone) Enable the following expectations when a joint's WorldPose
  // can be computed by gz-physics.
  // EXPECT_EQ(expWorldPose, this->GetWorldPose(joint1));
  // EXPECT_EQ(expWorldPose, this->GetWorldPose(joint2));
}

TEST_F(SdfFrameSemanticsTest, VisualCollisionRelativeTo)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <link name="L1">
        <pose>0.0 0 1 0 0 0</pose>
      </link>
      <link name="L2">
        <pose>0.0 0 2 0 0 0</pose>
        <visual name="v1">
          <pose relative_to="L1"/>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </visual>
        <collision name="c1">
          <pose relative_to="L1"/>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </collision>
      </link>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_TRUE(model.Valid(*this->ecm));

  Entity link2 = model.LinkByName(*this->ecm, "L2");
  EXPECT_NE(link2, kNullEntity);

  Entity visual = this->ecm->EntityByComponents(
      components::ParentEntity(link2), components::Name("v1"),
      components::Visual());
  EXPECT_NE(visual, kNullEntity);

  Entity collision = this->ecm->EntityByComponents(
      components::ParentEntity(link2), components::Name("c1"),
      components::Collision());
  EXPECT_NE(collision, kNullEntity);

  // Expect the pose of v1 and relative to L2 (their parent link) to be the same
  // as the pose of L1 relative to L2
  math::Pose3d expPose(0, 0, -1, 0, 0, 0);
  EXPECT_EQ(expPose, this->GetPose(visual));
  EXPECT_EQ(expPose, this->GetPose(collision));

  // Step once and check, the pose should still be close to its initial pose
  this->server->Run(true, 1, false);

  EXPECT_EQ(expPose, this->GetPose(visual));
  EXPECT_EQ(expPose, this->GetPose(collision));
}

TEST_F(SdfFrameSemanticsTest, ExplicitFramesWithLinks)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <frame name="F1">
        <pose>0.0 0 1 0 0 0</pose>
      </frame>
      <link name="L1">
        <pose relative_to="F1"/>
      </link>

      <frame name="F2" attached_to="L1">
        <pose relative_to="__model__">0.0 0 0 0 0 0</pose>
      </frame>
      <link name="L2">
        <pose relative_to="F2"/>
      </link>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_TRUE(model.Valid(*this->ecm));

  Entity link1 = model.LinkByName(*this->ecm, "L1");
  EXPECT_NE(link1, kNullEntity);

  Entity link2 = model.LinkByName(*this->ecm, "L2");
  EXPECT_NE(link2, kNullEntity);

  // Expect the pose of L1 and relative to M to be the same
  // as the pose of F1 relative to M
  math::Pose3d link1ExpRelativePose(0, 0, 1, 0, 0, 0);

  EXPECT_EQ(link1ExpRelativePose, this->GetPose(link1));

  // Expect the pose of L2 and relative to M to be the same
  // as the pose of F2, which is at the origin of M
  math::Pose3d link2ExpRelativePose = math::Pose3d::Zero;

  EXPECT_EQ(link2ExpRelativePose, this->GetPose(link2));

  // Step once and check
  this->server->Run(true, 1, false);

  EXPECT_EQ(link1ExpRelativePose, this->GetPose(link1));
  EXPECT_EQ(link2ExpRelativePose, this->GetPose(link2));
}

TEST_F(SdfFrameSemanticsTest, ExplicitFramesWithJoints)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <frame name="F1">
        <pose>0.0 0 2 0 0 0</pose>
      </frame>
      <link name="L1">
        <pose relative_to="F1"/>
      </link>
      <link name="L2">
        <pose>0 1 0 0 0 0</pose>
      </link>
      <joint name="J1" type="revolute">
        <pose relative_to="F1"/>
        <parent>L1</parent>
        <child>L2</child>
      </joint>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_TRUE(model.Valid(*this->ecm));

  Entity joint1 = model.JointByName(*this->ecm, "J1");
  EXPECT_NE(joint1, kNullEntity);

  this->ecm->CreateComponent(joint1, components::WorldPose());
  // Step once and check
  this->server->Run(true, 1, false);

  // Expect the pose of J1 relative to model to be the same as F1 in world
  math::Pose3d expWorldPose(1, 0, 2, 0, 0, 0);
  // TODO(anyone) Enable the following expectation when a joint's WorldPose
  // can be computed by gz-physics.
  // EXPECT_EQ(expWorldPose, this->GetWorldPose(joint1));
}

TEST_F(SdfFrameSemanticsTest, ExplicitFramesWithVisualAndCollision)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <frame name="F1">
        <pose relative_to="L1">0 0 1 0 0 0</pose>
      </frame>
      <link name="L1">
        <pose>0 0 2 0 0 0</pose>
        <visual name="v1">
          <pose relative_to="F1"/>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </visual>
        <collision name="c1">
          <pose relative_to="F1"/>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </collision>
      </link>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_TRUE(model.Valid(*this->ecm));

  Entity link1 = model.LinkByName(*this->ecm, "L1");
  EXPECT_NE(link1, kNullEntity);

  Entity visual = this->ecm->EntityByComponents(
      components::ParentEntity(link1), components::Name("v1"),
      components::Visual());
  EXPECT_NE(visual, kNullEntity);

  Entity collision = this->ecm->EntityByComponents(
      components::ParentEntity(link1), components::Name("c1"),
      components::Collision());
  EXPECT_NE(collision, kNullEntity);

  // Expect the pose of v1 and relative to L1 (their parent link) to be the same
  // as the pose of F1 relative to L1
  math::Pose3d expPose(0, 0, 1, 0, 0, 0);
  EXPECT_EQ(expPose, this->GetPose(visual));
  EXPECT_EQ(expPose, this->GetPose(collision));

  // Step once and check, the pose should still be close to its initial pose
  this->server->Run(true, 1, false);

  EXPECT_EQ(expPose, this->GetPose(visual));
  EXPECT_EQ(expPose, this->GetPose(collision));
}

TEST_F(SdfFrameSemanticsTest, NestedModelsRelativeTo)
{
  const std::string modelSdf = R"sdf(
  <sdf version="1.8">
    <model name="M1">
      <link name="L1">
        <pose relative_to="M2::L2">1 0 0 0 0 0</pose>
      </link>
      <model name="M2">
        <frame name="F1">
          <pose>0 0 1 0 0 0</pose>
        </frame>
        <link name="L2">
          <pose relative_to="F1">0 1 0 0 0 0</pose>
        </link>
      </model>
    </model>
  </sdf>
  )sdf";

  ASSERT_TRUE(this->StartServer());
  this->SpawnModelSDF(modelSdf);

  auto model1 = this->GetModel("M1");
  ASSERT_TRUE(model1.Valid(*this->ecm));

  auto model2 = this->GetModel("M2");
  ASSERT_TRUE(model2.Valid(*this->ecm));

  Entity link1 = model1.LinkByName(*this->ecm, "L1");
  ASSERT_NE(link1, kNullEntity);

  Entity link2 = model2.LinkByName(*this->ecm, "L2");
  ASSERT_NE(link2, kNullEntity);

  const gz::math::Pose3d link1ExpectedPose(1, 1, 1, 0, 0, 0);
  const gz::math::Pose3d link2ExpectedPose(0, 1, 1, 0, 0, 0);

  EXPECT_EQ(link1ExpectedPose, this->GetPose(link1));
  EXPECT_EQ(link2ExpectedPose, this->GetPose(link2));

  // Step once and check, the pose should still be close to its initial pose
  this->server->Run(true, 1, false);

  EXPECT_EQ(link1ExpectedPose, this->GetPose(link1));
  EXPECT_EQ(link2ExpectedPose, this->GetPose(link2));
}

TEST_F(SdfFrameSemanticsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(IncludeNestedModelsRelativeToTPE))
{
  std::string path = std::string(PROJECT_SOURCE_PATH) + "/test/worlds/models";
  gz::common::setenv("GZ_SIM_RESOURCE_PATH", path.c_str());
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetResourceCache(path);
  serverConfig.SetPhysicsEngine("gz-physics-tpe-plugin");

  const std::string sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/include_nested_models.sdf";
  serverConfig.SetSdfFile(sdfFile);

  ASSERT_TRUE(this->StartServer(serverConfig));

  auto includeNestedModel = this->GetModel("include_nested_new_name");
  ASSERT_TRUE(includeNestedModel.Valid(*this->ecm));

  auto model00 = this->GetModel("model_00");
  ASSERT_TRUE(model00.Valid(*this->ecm));

  auto nestedModel =
    this->GetChildModel(includeNestedModel.Entity(), "nested_models_new_name");
  ASSERT_TRUE(nestedModel.Valid(*this->ecm));

  auto nestedModel00 = this->GetChildModel(nestedModel.Entity(), "model_00");
  ASSERT_TRUE(nestedModel00.Valid(*this->ecm));

  auto nestedModel01 = this->GetChildModel(nestedModel00.Entity(), "model_01");
  ASSERT_TRUE(nestedModel01.Valid(*this->ecm));

  Entity link00 = includeNestedModel.LinkByName(*this->ecm, "link_00");
  ASSERT_NE(link00, kNullEntity);

  gz::math::Pose3d link00ExpectedPose(30, 32, 34, 0, 0, 0);
  EXPECT_EQ(link00ExpectedPose, this->GetPose(link00));

  auto link01 = includeNestedModel.LinkByName(*this->ecm, "link_01");
  ASSERT_NE(link01, kNullEntity);

  gz::math::Pose3d link01ExpectedPose(20, 21, 22, 0, 0, 0);
  EXPECT_EQ(link01ExpectedPose, this->GetPose(link01));

  Entity nestedModelsLink00 = nestedModel00.LinkByName(*this->ecm, "link_00");
  ASSERT_NE(nestedModelsLink00, kNullEntity);

  gz::math::Pose3d nestedModelsLink00ExpectedPose(20, 21, 22, 0, 0, 0);
  EXPECT_EQ(nestedModelsLink00ExpectedPose, this->GetPose(nestedModelsLink00));

  auto nestedModelsLink01 = nestedModel01.LinkByName(*this->ecm, "link_01");
  ASSERT_NE(nestedModelsLink01, kNullEntity);

  gz::math::Pose3d nestedModelsLink01ExpectedPose(20, 21, 22, 0, 0, 0);
  EXPECT_EQ(nestedModelsLink01ExpectedPose, this->GetPose(nestedModelsLink01));

  this->server->Run(true, 1, false);

  EXPECT_EQ(link01ExpectedPose, this->GetPose(link01));
  EXPECT_EQ(link00ExpectedPose, this->GetPose(link00));
  EXPECT_EQ(nestedModelsLink00ExpectedPose, this->GetPose(nestedModelsLink00));
  EXPECT_EQ(nestedModelsLink01ExpectedPose, this->GetPose(nestedModelsLink01));
}

TEST_F(SdfFrameSemanticsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(IncludeNestedModelsRelativeToDartsim))
{
  std::string path = std::string(PROJECT_SOURCE_PATH) + "/test/worlds/models";
  gz::common::setenv("GZ_SIM_RESOURCE_PATH", path.c_str());
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetResourceCache(path);
  serverConfig.SetPhysicsEngine("gz-physics-dartsim-plugin");

  const std::string sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/include_nested_models.sdf";
  serverConfig.SetSdfFile(sdfFile);

  ASSERT_TRUE(this->StartServer(serverConfig));

  auto includeNestedModel = this->GetModel("include_nested_new_name");
  ASSERT_TRUE(includeNestedModel.Valid(*this->ecm));

  auto model00 = this->GetModel("model_00");
  ASSERT_TRUE(model00.Valid(*this->ecm));

  auto nestedModel =
    this->GetChildModel(includeNestedModel.Entity(), "nested_models_new_name");
  ASSERT_TRUE(nestedModel.Valid(*this->ecm));

  auto nestedModel00 = this->GetChildModel(nestedModel.Entity(), "model_00");
  ASSERT_TRUE(nestedModel00.Valid(*this->ecm));

  auto nestedModel01 = this->GetChildModel(nestedModel00.Entity(), "model_01");
  ASSERT_TRUE(nestedModel01.Valid(*this->ecm));

  Entity link00 = includeNestedModel.LinkByName(*this->ecm, "link_00");
  ASSERT_NE(link00, kNullEntity);

  gz::math::Pose3d link00ExpectedPose(30, 32, 34, 0, 0, 0);
  EXPECT_EQ(link00ExpectedPose, this->GetPose(link00));

  auto link01 = includeNestedModel.LinkByName(*this->ecm, "link_01");
  ASSERT_NE(link01, kNullEntity);

  gz::math::Pose3d link01ExpectedPose(20, 21, 22, 0, 0, 0);
  EXPECT_EQ(link01ExpectedPose, this->GetPose(link01));

  Entity nestedModelsLink00 = nestedModel00.LinkByName(*this->ecm, "link_00");
  ASSERT_NE(nestedModelsLink00, kNullEntity);

  gz::math::Pose3d nestedModelsLink00ExpectedPose(20, 21, 22, 0, 0, 0);
  EXPECT_EQ(nestedModelsLink00ExpectedPose, this->GetPose(nestedModelsLink00));

  auto nestedModelsLink01 = nestedModel01.LinkByName(*this->ecm, "link_01");
  ASSERT_NE(nestedModelsLink01, kNullEntity);

  gz::math::Pose3d nestedModelsLink01ExpectedPose(20, 21, 22, 0, 0, 0);
  EXPECT_EQ(nestedModelsLink01ExpectedPose, this->GetPose(nestedModelsLink01));

  this->server->Run(true, 1, false);

  EXPECT_EQ(link01ExpectedPose, this->GetPose(link01));
  EXPECT_EQ(link00ExpectedPose, this->GetPose(link00));
  EXPECT_EQ(nestedModelsLink00ExpectedPose, this->GetPose(nestedModelsLink00));
  EXPECT_EQ(nestedModelsLink01ExpectedPose, this->GetPose(nestedModelsLink01));
}

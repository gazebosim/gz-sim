/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/SupportedPhysicsFeatures.hh"
#include "gz/sim/components/SystemPluginInfo.hh"
#include "gz/sim/components/World.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "SystemManager.hh"

using namespace gz::sim;

constexpr System::PriorityType kPriority = 64;

/////////////////////////////////////////////////
class SystemWithConfigure:
  public System,
  public ISystemConfigure,
  public ISystemConfigureParameters
{
  // Documentation inherited
  public: void Configure(
                const Entity &,
                const std::shared_ptr<const sdf::Element> &,
                EntityComponentManager &,
                EventManager &) override { configured++; };

  public: void ConfigureParameters(
                gz::transport::parameters::ParametersRegistry &,
                EntityComponentManager &) override { configuredParameters++; }

  public: int configured = 0;
  public: int configuredParameters = 0;
};

/////////////////////////////////////////////////
class SystemWithUpdates:
  public System,
  public ISystemPreUpdate,
  public ISystemUpdate,
  public ISystemPostUpdate
{
  // Documentation inherited
  public: void PreUpdate(const UpdateInfo &,
                EntityComponentManager &) override {};

  // Documentation inherited
  public: void Update(const UpdateInfo &,
                EntityComponentManager &) override {};

  // Documentation inherited
  public: void PostUpdate(const UpdateInfo &,
                const EntityComponentManager &) override {};
};

/////////////////////////////////////////////////
class SystemWithPrioritizedUpdates:
  public SystemWithUpdates,
  public ISystemConfigurePriority
{
  // Documentation inherited
  public: System::PriorityType ConfigurePriority() override
                { return kPriority; }
};

/////////////////////////////////////////////////
class SystemWithPhysicsRequirements:
  public SystemWithUpdates,
  public ISystemPhysicsRequirements
{
  // Documentation inherited
  public: std::vector<std::string> PhysicsRequirements() override
  {
    this->queried++;
    return this->requirements;
  }

  public: std::vector<std::string> requirements;
  public: int queried = 0;
};

/////////////////////////////////////////////////
TEST(SystemManager, Constructor)
{
  auto loader = std::make_shared<SystemLoader>();
  SystemManager systemMgr(loader);

  ASSERT_EQ(0u, systemMgr.ActiveCount());
  ASSERT_EQ(0u, systemMgr.PendingCount());
  ASSERT_EQ(0u, systemMgr.TotalCount());

  ASSERT_EQ(0u, systemMgr.SystemsConfigure().size());
  ASSERT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  ASSERT_EQ(0u, systemMgr.SystemsUpdate().size());
  ASSERT_EQ(0u, systemMgr.SystemsPostUpdate().size());
}

/////////////////////////////////////////////////
TEST(SystemManager, AddSystemNoEcm)
{
  auto loader = std::make_shared<SystemLoader>();
  SystemManager systemMgr(loader);

  EXPECT_EQ(0u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(0u, systemMgr.TotalCount());
  EXPECT_EQ(0u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());

  auto configSystem = std::make_shared<SystemWithConfigure>();
  Entity configEntity{123u};
  systemMgr.AddSystem(configSystem, configEntity, nullptr);

  // SystemManager without an ECM/EventmManager will mean no config occurs
  EXPECT_EQ(0, configSystem->configured);
  EXPECT_EQ(0, configSystem->configuredParameters);

  EXPECT_EQ(0u, systemMgr.ActiveCount());
  EXPECT_EQ(1u, systemMgr.PendingCount());
  EXPECT_EQ(1u, systemMgr.TotalCount());
  EXPECT_EQ(0u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());
  EXPECT_EQ(1u, systemMgr.TotalByEntity(configEntity).size());

  systemMgr.ActivatePendingSystems();
  EXPECT_EQ(1u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(1u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());
  EXPECT_EQ(1u, systemMgr.TotalByEntity(configEntity).size());

  auto updateSystem = std::make_shared<SystemWithUpdates>();
  auto prioritizedSystem =
      std::make_shared<SystemWithPrioritizedUpdates>();
  Entity updateEntity{456u};
  systemMgr.AddSystem(updateSystem, updateEntity, nullptr);
  systemMgr.AddSystem(prioritizedSystem, updateEntity, nullptr);
  EXPECT_EQ(1u, systemMgr.ActiveCount());
  EXPECT_EQ(2u, systemMgr.PendingCount());
  EXPECT_EQ(3u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());
  EXPECT_EQ(2u, systemMgr.TotalByEntity(updateEntity).size());

  systemMgr.ActivatePendingSystems();
  EXPECT_EQ(3u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(3u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  // Expect PreUpdate and Update to contain two map entries:
  // 1. Priority {0} and a vector of length 1.
  // 2. Priority {kPriority} and a vector of length 1.
  EXPECT_EQ(2u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().count(0));
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().count(kPriority));
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().at(0).size());
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().at(kPriority).size());
  EXPECT_EQ(2u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().count(0));
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().count(kPriority));
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().at(0).size());
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().at(kPriority).size());
  EXPECT_EQ(2u, systemMgr.SystemsPostUpdate().size());
  EXPECT_EQ(2u, systemMgr.TotalByEntity(updateEntity).size());
}

/////////////////////////////////////////////////
TEST(SystemManager, AddSystemEcm)
{
  auto loader = std::make_shared<SystemLoader>();

  auto ecm = EntityComponentManager();
  auto eventManager = EventManager();

  auto paramRegistry = std::make_unique<
    gz::transport::parameters::ParametersRegistry>("SystemManager_TEST");
  SystemManager systemMgr(
    loader, &ecm, &eventManager, std::string(), paramRegistry.get());

  EXPECT_EQ(0u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(0u, systemMgr.TotalCount());
  EXPECT_EQ(0u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());

  auto configSystem = std::make_shared<SystemWithConfigure>();
  systemMgr.AddSystem(configSystem, kNullEntity, nullptr);

  // Configure called during AddSystem
  EXPECT_EQ(1, configSystem->configured);
  EXPECT_EQ(1, configSystem->configuredParameters);

  EXPECT_EQ(0u, systemMgr.ActiveCount());
  EXPECT_EQ(1u, systemMgr.PendingCount());
  EXPECT_EQ(1u, systemMgr.TotalCount());
  EXPECT_EQ(0u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());

  systemMgr.ActivatePendingSystems();
  EXPECT_EQ(1u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(1u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());

  auto updateSystem = std::make_shared<SystemWithUpdates>();
  auto prioritizedSystem =
      std::make_shared<SystemWithPrioritizedUpdates>();
  systemMgr.AddSystem(updateSystem, kNullEntity, nullptr);
  systemMgr.AddSystem(prioritizedSystem, kNullEntity, nullptr);
  EXPECT_EQ(1u, systemMgr.ActiveCount());
  EXPECT_EQ(2u, systemMgr.PendingCount());
  EXPECT_EQ(3u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());

  systemMgr.ActivatePendingSystems();
  EXPECT_EQ(3u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(3u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  // Expect PreUpdate and Update to contain two map entries:
  // 1. Priority {0} and a vector of length 1.
  // 2. Priority {kPriority} and a vector of length 1.
  EXPECT_EQ(2u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().count(0));
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().count(kPriority));
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().at(0).size());
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().at(kPriority).size());
  EXPECT_EQ(2u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().count(0));
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().count(kPriority));
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().at(0).size());
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().at(kPriority).size());
  EXPECT_EQ(2u, systemMgr.SystemsPostUpdate().size());
}

/////////////////////////////////////////////////
TEST(SystemManager, AddAndRemoveSystemEcm)
{
  auto loader = std::make_shared<SystemLoader>();

  auto ecm = EntityComponentManager();
  auto eventManager = EventManager();

  auto paramRegistry = std::make_unique<
    gz::transport::parameters::ParametersRegistry>("SystemManager_TEST");
  SystemManager systemMgr(
    loader, &ecm, &eventManager, std::string(), paramRegistry.get());

  EXPECT_EQ(0u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(0u, systemMgr.TotalCount());
  EXPECT_EQ(0u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());

  auto configSystem = std::make_shared<SystemWithConfigure>();
  systemMgr.AddSystem(configSystem, kNullEntity, nullptr);

  auto entity = ecm.CreateEntity();

  auto updateSystemWithChild = std::make_shared<SystemWithUpdates>();
  auto prioritizedSystemWithChild =
      std::make_shared<SystemWithPrioritizedUpdates>();
  systemMgr.AddSystem(updateSystemWithChild, entity, nullptr);
  systemMgr.AddSystem(prioritizedSystemWithChild, entity, nullptr);

  // Configure called during AddSystem
  EXPECT_EQ(1, configSystem->configured);
  EXPECT_EQ(1, configSystem->configuredParameters);

  EXPECT_EQ(0u, systemMgr.ActiveCount());
  EXPECT_EQ(3u, systemMgr.PendingCount());
  EXPECT_EQ(3u, systemMgr.TotalCount());
  EXPECT_EQ(0u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());

  systemMgr.ActivatePendingSystems();
  EXPECT_EQ(3u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(3u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  // Expect PreUpdate and Update to contain two map entries:
  // 1. Priority {0} and a vector of length 1.
  // 2. Priority {kPriority} and a vector of length 1.
  EXPECT_EQ(2u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().count(0));
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().count(kPriority));
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().at(0).size());
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().at(kPriority).size());
  EXPECT_EQ(2u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().count(0));
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().count(kPriority));
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().at(0).size());
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().at(kPriority).size());
  EXPECT_EQ(2u, systemMgr.SystemsPostUpdate().size());

  // Remove the entity
  ecm.RequestRemoveEntity(entity);
  bool needsCleanUp;
  systemMgr.ProcessRemovedEntities(ecm, needsCleanUp);

  EXPECT_TRUE(needsCleanUp);
  EXPECT_EQ(1u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(1u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());
}

/////////////////////////////////////////////////
TEST(SystemManager, AddSystemWithInfo)
{
  auto loader = std::make_shared<SystemLoader>();

  EntityComponentManager ecm;
  auto entity = ecm.CreateEntity();
  EXPECT_NE(kNullEntity, entity);

  auto eventManager = EventManager();

  SystemManager systemMgr(loader, &ecm, &eventManager);

  // No element, no SystemPluginInfo component
  auto configSystem = std::make_shared<SystemWithConfigure>();
  systemMgr.AddSystem(configSystem, entity, nullptr);

  // Element becomes SystemPluginInfo component
  auto pluginElem = std::make_shared<sdf::Element>();
  sdf::initFile("plugin.sdf", pluginElem);
  sdf::readString("<?xml version='1.0'?><sdf version='1.6'>"
      "  <plugin filename='plum' name='peach'>"
      "    <avocado>0.5</avocado>"
      "  </plugin>"
      "</sdf>", pluginElem);

  auto updateSystem = std::make_shared<SystemWithUpdates>();
  systemMgr.AddSystem(updateSystem, entity, pluginElem);

  int entityCount{0};
  ecm.Each<components::SystemPluginInfo>(
      [&](const Entity &_entity,
          const components::SystemPluginInfo *_systemInfoComp) -> bool
      {
        EXPECT_EQ(entity, _entity);

        EXPECT_NE(nullptr, _systemInfoComp);
        if (nullptr == _systemInfoComp)
          return true;

        auto pluginsMsg = _systemInfoComp->Data();
        EXPECT_EQ(1, pluginsMsg.plugins().size());
        if (1u != pluginsMsg.plugins().size())
          return true;

        auto pluginMsg = pluginsMsg.plugins(0);
        EXPECT_EQ("plum", pluginMsg.filename());
        EXPECT_EQ("peach", pluginMsg.name());
        EXPECT_NE(pluginMsg.innerxml().find("<avocado>0.5</avocado>"),
            std::string::npos);

        entityCount++;
        return true;
      });
  EXPECT_EQ(1, entityCount);
}

/////////////////////////////////////////////////
/// \brief Test fixture with a world, and two top-level models with a link.
class SystemManagerPhysicsRequirements : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    // world
    //  - modelA
    //    - linkA
    //  - modelB
    this->world = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->world, components::World());
    this->ecm.CreateComponent(this->world, components::Name("world"));

    this->modelA = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->modelA, components::Model());
    this->ecm.CreateComponent(this->modelA, components::Name("modelA"));
    this->ecm.CreateComponent(this->modelA,
        components::ParentEntity(this->world));

    this->linkA = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->linkA, components::Link());
    this->ecm.CreateComponent(this->linkA, components::Name("linkA"));
    this->ecm.CreateComponent(this->linkA,
        components::ParentEntity(this->modelA));

    this->modelB = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->modelB, components::Model());
    this->ecm.CreateComponent(this->modelB, components::Name("modelB"));
    this->ecm.CreateComponent(this->modelB,
        components::ParentEntity(this->world));
  }

  /// \brief Create a system manager that uses the ECM of the fixture.
  protected: std::unique_ptr<SystemManager> MakeManager()
  {
    return std::make_unique<SystemManager>(
        std::make_shared<SystemLoader>(), &this->ecm, &this->eventManager);
  }

  /// \brief Create a system that needs physics features.
  /// \param[in] _requirements Names of the features.
  protected: std::shared_ptr<SystemWithPhysicsRequirements> MakeSystem(
      const std::vector<std::string> &_requirements)
  {
    auto system = std::make_shared<SystemWithPhysicsRequirements>();
    system->requirements = _requirements;
    return system;
  }

  protected: EntityComponentManager ecm;
  protected: EventManager eventManager;
  protected: Entity world{kNullEntity};
  protected: Entity modelA{kNullEntity};
  protected: Entity linkA{kNullEntity};
  protected: Entity modelB{kNullEntity};
};

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirements, NoDeclaration)
{
  auto systemMgr = this->MakeManager();

  // Without a SupportedPhysicsFeatures component, every requirement is met.
  auto system = this->MakeSystem({"feature_a", "feature_b"});
  systemMgr->AddSystem(system, this->modelA, nullptr);

  // A system without requirements is not affected by a component that lists
  // nothing.
  this->ecm.CreateComponent(this->modelB,
      components::SupportedPhysicsFeatures());
  auto plain = std::make_shared<SystemWithUpdates>();
  systemMgr->AddSystem(plain, this->modelB, nullptr);
  auto empty = this->MakeSystem({});
  systemMgr->AddSystem(empty, this->modelB, nullptr);

  EXPECT_EQ(3u, systemMgr->PendingCount());
  ::testing::internal::CaptureStderr();
  EXPECT_EQ(3u, systemMgr->ActivatePendingSystems());
  EXPECT_EQ("", ::testing::internal::GetCapturedStderr());

  EXPECT_EQ(3u, systemMgr->ActiveCount());
  EXPECT_EQ(0u, systemMgr->PendingCount());
  EXPECT_EQ(1, system->queried);
  EXPECT_EQ(1, empty->queried);
}

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirements, MetOnModel)
{
  auto systemMgr = this->MakeManager();
  this->ecm.CreateComponent(this->modelA, components::SupportedPhysicsFeatures(
      {"feature_a", "feature_b", "feature_c"}));

  // On the model and on a link inside the model.
  auto onModel = this->MakeSystem({"feature_a", "feature_c"});
  auto onLink = this->MakeSystem({"feature_b"});
  systemMgr->AddSystem(onModel, this->modelA, nullptr);
  systemMgr->AddSystem(onLink, this->linkA, nullptr);

  ::testing::internal::CaptureStderr();
  EXPECT_EQ(2u, systemMgr->ActivatePendingSystems());
  EXPECT_EQ("", ::testing::internal::GetCapturedStderr());
  EXPECT_EQ(2u, systemMgr->ActiveCount());
  EXPECT_EQ(2u, systemMgr->SystemsUpdate().at(0).size());
}

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirements, NotMetOnModel)
{
  auto systemMgr = this->MakeManager();
  this->ecm.CreateComponent(this->modelA,
      components::SupportedPhysicsFeatures({"feature_a"}));

  // Another model with no declaration, and a system that does not need
  // anything, to see that only the right system is dropped.
  auto missing = this->MakeSystem({"feature_a", "feature_b"});
  auto onLink = this->MakeSystem({"feature_b"});
  auto fine = this->MakeSystem({"feature_a"});
  auto unaffected = this->MakeSystem({"feature_b"});
  auto plain = std::make_shared<SystemWithConfigure>();
  systemMgr->AddSystem(missing, this->modelA, nullptr);
  systemMgr->AddSystem(onLink, this->linkA, nullptr);
  systemMgr->AddSystem(fine, this->modelA, nullptr);
  systemMgr->AddSystem(unaffected, this->modelB, nullptr);
  systemMgr->AddSystem(plain, this->modelA, nullptr);
  EXPECT_EQ(5u, systemMgr->PendingCount());

  ::testing::internal::CaptureStderr();
  EXPECT_EQ(3u, systemMgr->ActivatePendingSystems());
  const auto output = ::testing::internal::GetCapturedStderr();

  // The dropped systems are gone, not pending.
  EXPECT_EQ(3u, systemMgr->ActiveCount());
  EXPECT_EQ(0u, systemMgr->PendingCount());
  EXPECT_EQ(3u, systemMgr->TotalCount());
  EXPECT_EQ(2u, systemMgr->SystemsUpdate().at(0).size());
  EXPECT_EQ(2u, systemMgr->SystemsPreUpdate().at(0).size());
  EXPECT_EQ(2u, systemMgr->SystemsPostUpdate().size());
  EXPECT_EQ(1u, systemMgr->SystemsConfigure().size());

  // The warning names the system, the model, and the missing feature.
  EXPECT_NE(std::string::npos, output.find("SystemWithPhysicsRequirements"))
      << output;
  EXPECT_NE(std::string::npos, output.find("model [modelA]")) << output;
  EXPECT_NE(std::string::npos, output.find("[feature_b]")) << output;
  EXPECT_EQ(std::string::npos, output.find("[feature_a]")) << output;
  EXPECT_EQ(std::string::npos, output.find("modelB")) << output;
}

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirements, WorldDefault)
{
  auto systemMgr = this->MakeManager();
  this->ecm.CreateComponent(this->world,
      components::SupportedPhysicsFeatures({"feature_a"}));
  // modelB overrides the default with its own, bigger, set.
  this->ecm.CreateComponent(this->modelB,
      components::SupportedPhysicsFeatures({"feature_a", "feature_b"}));

  auto onModelA = this->MakeSystem({"feature_b"});
  auto onModelB = this->MakeSystem({"feature_b"});
  auto onWorld = this->MakeSystem({"feature_b"});
  systemMgr->AddSystem(onModelA, this->modelA, nullptr);
  systemMgr->AddSystem(onModelB, this->modelB, nullptr);
  systemMgr->AddSystem(onWorld, this->world, nullptr);

  ::testing::internal::CaptureStderr();
  EXPECT_EQ(1u, systemMgr->ActivatePendingSystems());
  const auto output = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(1u, systemMgr->ActiveCount());
  EXPECT_NE(std::string::npos, output.find("model [modelA]")) << output;
  EXPECT_NE(std::string::npos, output.find("world [world]")) << output;

  // An empty set on the model overrides a world set that has the feature.
  auto systemMgr2 = this->MakeManager();
  this->ecm.Component<components::SupportedPhysicsFeatures>(this->modelB)
      ->Data().clear();
  this->ecm.Component<components::SupportedPhysicsFeatures>(this->world)
      ->Data() = {"feature_b"};
  auto emptyOnModel = this->MakeSystem({"feature_b"});
  auto fromWorld = this->MakeSystem({"feature_b"});
  systemMgr2->AddSystem(emptyOnModel, this->modelB, nullptr);
  systemMgr2->AddSystem(fromWorld, this->modelA, nullptr);
  ::testing::internal::CaptureStderr();
  EXPECT_EQ(1u, systemMgr2->ActivatePendingSystems());
  ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(1, fromWorld->queried);
  EXPECT_EQ(1u, systemMgr2->SystemsUpdate().at(0).size());
}

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirements, SeveralMissing)
{
  auto systemMgr = this->MakeManager();
  this->ecm.CreateComponent(this->modelA,
      components::SupportedPhysicsFeatures());

  auto system = this->MakeSystem({"feature_a", "feature_b"});
  systemMgr->AddSystem(system, this->modelA, nullptr);

  ::testing::internal::CaptureStderr();
  EXPECT_EQ(0u, systemMgr->ActivatePendingSystems());
  const auto output = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(0u, systemMgr->TotalCount());
  EXPECT_NE(std::string::npos, output.find("[feature_a]")) << output;
  EXPECT_NE(std::string::npos, output.find("[feature_b]")) << output;
}

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirements, NoEcmOrEntity)
{
  // Without an ECM there is nothing to check against.
  SystemManager noEcm(std::make_shared<SystemLoader>());
  auto system = this->MakeSystem({"feature_a"});
  noEcm.AddSystem(system, this->modelA, nullptr);
  EXPECT_EQ(1u, noEcm.ActivatePendingSystems());

  // A system that is not attached to an entity is not checked either.
  auto systemMgr = this->MakeManager();
  this->ecm.CreateComponent(this->world,
      components::SupportedPhysicsFeatures());
  auto detached = this->MakeSystem({"feature_a"});
  systemMgr->AddSystem(detached, kNullEntity, nullptr);
  EXPECT_EQ(1u, systemMgr->ActivatePendingSystems());
}

/////////////////////////////////////////////////
/// \brief Load a system plugin that has an SDF element.
class SystemManagerPhysicsRequirementsPlugin
  : public SystemManagerPhysicsRequirements
{
  // Documentation inherited
  protected: void SetUp() override
  {
    SystemManagerPhysicsRequirements::SetUp();

    this->loader = std::make_shared<SystemLoader>();
    this->loader->AddSystemPluginPath(gz::common::joinPaths(
        std::string(PROJECT_BINARY_PATH), "lib"));
  }

  /// \brief Load a test plugin with extra SDF in its element.
  /// \param[in] _manager Where to load it.
  /// \param[in] _entity Entity the plugin belongs to.
  /// \param[in] _filename Name of the plugin library.
  /// \param[in] _name Name of the plugin.
  /// \param[in] _content SDF children of the plugin element.
  protected: void LoadTestPlugin(SystemManager &_manager, Entity _entity,
      const std::string &_filename, const std::string &_name,
      const std::string &_content)
  {
    sdf::Root root;
    root.LoadSdfString(std::string("<?xml version='1.0'?>"
        "<sdf version='1.9'><world name='default'>"
        "<plugin filename='") + _filename + "' name='" + _name + "'>" +
        _content + "</plugin></world></sdf>");
    ASSERT_NE(nullptr, root.WorldByIndex(0));
    ASSERT_EQ(1u, root.WorldByIndex(0)->Plugins().size());
    _manager.LoadPlugin(_entity, root.WorldByIndex(0)->Plugins()[0]);
  }

  /// \brief Load the MockSystem test plugin, which has no requirements.
  protected: void LoadMock(SystemManager &_manager, Entity _entity,
      const std::string &_content)
  {
    this->LoadTestPlugin(_manager, _entity, "MockSystem",
        "gz::sim::MockSystem", _content);
  }

  /// \brief Load the TestPhysicsRequirementsSystem test plugin, which
  /// requires feature_a and feature_b through ISystemPhysicsRequirements.
  protected: void LoadRequirementsPlugin(SystemManager &_manager,
      Entity _entity, const std::string &_content)
  {
    this->LoadTestPlugin(_manager, _entity, "TestPhysicsRequirementsSystem",
        "gz::sim::TestPhysicsRequirementsSystem", _content);
  }

  protected: SystemLoaderPtr loader;
};

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirementsPlugin, SdfRequirements)
{
  SystemManager systemMgr(this->loader, &this->ecm, &this->eventManager);
  this->ecm.CreateComponent(this->modelA,
      components::SupportedPhysicsFeatures({"feature_a"}));

  // The plugin has no requirements of its own, the SDF adds them.
  this->LoadMock(systemMgr, this->modelA,
      "<gz:physics_requirements>feature_a feature_b</gz:physics_requirements>");
  this->LoadMock(systemMgr, this->modelA,
      "<gz:physics_requirements>feature_a</gz:physics_requirements>");
  this->LoadMock(systemMgr, this->modelA, "");
  ASSERT_EQ(3u, systemMgr.PendingCount());

  ::testing::internal::CaptureStderr();
  EXPECT_EQ(2u, systemMgr.ActivatePendingSystems());
  const auto output = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(2u, systemMgr.ActiveCount());
  EXPECT_NE(std::string::npos, output.find("gz::sim::MockSystem"))
      << output;
  EXPECT_NE(std::string::npos, output.find("model [modelA]")) << output;
  EXPECT_NE(std::string::npos, output.find("[feature_b]")) << output;

  // A reset reloads the systems that were active, and not the one that was
  // dropped.
  UpdateInfo info;
  systemMgr.Reset(info, this->ecm);
  EXPECT_EQ(2u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
}

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirementsPlugin, InterfaceFromPlugin)
{
  // The plugin lists ISystemPhysicsRequirements in GZ_ADD_PLUGIN, so the
  // manager finds the interface without a dynamic_cast.
  SystemManager systemMgr(this->loader, &this->ecm, &this->eventManager);
  this->ecm.CreateComponent(this->modelA,
      components::SupportedPhysicsFeatures({"feature_a"}));
  this->ecm.CreateComponent(this->modelB,
      components::SupportedPhysicsFeatures({"feature_a", "feature_b"}));

  this->LoadRequirementsPlugin(systemMgr, this->modelA, "");
  this->LoadRequirementsPlugin(systemMgr, this->modelB, "");
  ASSERT_EQ(2u, systemMgr.PendingCount());

  ::testing::internal::CaptureStderr();
  EXPECT_EQ(1u, systemMgr.ActivatePendingSystems());
  const auto output = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(1u, systemMgr.ActiveCount());
  EXPECT_NE(std::string::npos,
      output.find("gz::sim::TestPhysicsRequirementsSystem")) << output;
  EXPECT_NE(std::string::npos, output.find("model [modelA]")) << output;
  EXPECT_NE(std::string::npos, output.find("[feature_b]")) << output;
  EXPECT_EQ(std::string::npos, output.find("[feature_a]")) << output;
  EXPECT_EQ(std::string::npos, output.find("modelB")) << output;
}

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirementsPlugin, SdfReplacesInterface)
{
  // modelA supports feature_a only. The plugin declares feature_a and
  // feature_b, so it is dropped unless the SDF replaces its list.
  SystemManager systemMgr(this->loader, &this->ecm, &this->eventManager);
  this->ecm.CreateComponent(this->modelA,
      components::SupportedPhysicsFeatures({"feature_a"}));

  this->LoadRequirementsPlugin(systemMgr, this->modelA,
      "<gz:physics_requirements>feature_a</gz:physics_requirements>");
  ::testing::internal::CaptureStderr();
  EXPECT_EQ(1u, systemMgr.ActivatePendingSystems());
  EXPECT_EQ("", ::testing::internal::GetCapturedStderr());

  // The replacement does not add to the declared list. It can also ask for
  // something the declared list does not.
  this->LoadRequirementsPlugin(systemMgr, this->modelA,
      "<gz:physics_requirements>feature_c</gz:physics_requirements>");
  ::testing::internal::CaptureStderr();
  EXPECT_EQ(0u, systemMgr.ActivatePendingSystems());
  const auto output = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(std::string::npos, output.find("[feature_c]")) << output;
  EXPECT_EQ(std::string::npos, output.find("[feature_b]")) << output;
  EXPECT_EQ(1u, systemMgr.ActiveCount());
}

/////////////////////////////////////////////////
TEST_F(SystemManagerPhysicsRequirementsPlugin, EmptySdfClearsInterface)
{
  // The provider supports nothing, and the plugin declares requirements. An
  // empty element removes them.
  SystemManager systemMgr(this->loader, &this->ecm, &this->eventManager);
  this->ecm.CreateComponent(this->modelA,
      components::SupportedPhysicsFeatures());

  this->LoadRequirementsPlugin(systemMgr, this->modelA,
      "<gz:physics_requirements/>");
  this->LoadRequirementsPlugin(systemMgr, this->modelA,
      "<gz:physics_requirements></gz:physics_requirements>");
  ASSERT_EQ(2u, systemMgr.PendingCount());

  ::testing::internal::CaptureStderr();
  EXPECT_EQ(2u, systemMgr.ActivatePendingSystems());
  EXPECT_EQ("", ::testing::internal::GetCapturedStderr());
  EXPECT_EQ(2u, systemMgr.ActiveCount());

  // Without the element, the declared requirements apply.
  this->LoadRequirementsPlugin(systemMgr, this->modelA, "");
  ::testing::internal::CaptureStderr();
  EXPECT_EQ(0u, systemMgr.ActivatePendingSystems());
  ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(2u, systemMgr.ActiveCount());
}

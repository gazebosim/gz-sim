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

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/components/SystemPluginInfo.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "SystemManager.hh"

using namespace gz::sim;

/////////////////////////////////////////////////
class SystemWithConfigure:
  public System,
  public ISystemConfigure
{
  // Documentation inherited
  public: void Configure(
                const Entity &,
                const std::shared_ptr<const sdf::Element> &,
                EntityComponentManager &,
                EventManager &) override { configured++; };

  public: int configured = 0;
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
  Entity updateEntity{456u};
  systemMgr.AddSystem(updateSystem, updateEntity, nullptr);
  EXPECT_EQ(1u, systemMgr.ActiveCount());
  EXPECT_EQ(1u, systemMgr.PendingCount());
  EXPECT_EQ(2u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());
  EXPECT_EQ(1u, systemMgr.TotalByEntity(updateEntity).size());

  systemMgr.ActivatePendingSystems();
  EXPECT_EQ(2u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(2u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsPostUpdate().size());
  EXPECT_EQ(1u, systemMgr.TotalByEntity(updateEntity).size());
}

/////////////////////////////////////////////////
TEST(SystemManager, AddSystemEcm)
{
  auto loader = std::make_shared<SystemLoader>();

  auto ecm = EntityComponentManager();
  auto eventManager = EventManager();

  SystemManager systemMgr(loader, &ecm, &eventManager);

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
  systemMgr.AddSystem(updateSystem, kNullEntity, nullptr);
  EXPECT_EQ(1u, systemMgr.ActiveCount());
  EXPECT_EQ(1u, systemMgr.PendingCount());
  EXPECT_EQ(2u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(0u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(0u, systemMgr.SystemsPostUpdate().size());

  systemMgr.ActivatePendingSystems();
  EXPECT_EQ(2u, systemMgr.ActiveCount());
  EXPECT_EQ(0u, systemMgr.PendingCount());
  EXPECT_EQ(2u, systemMgr.TotalCount());
  EXPECT_EQ(1u, systemMgr.SystemsConfigure().size());
  EXPECT_EQ(1u, systemMgr.SystemsPreUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsUpdate().size());
  EXPECT_EQ(1u, systemMgr.SystemsPostUpdate().size());
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


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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/server_control.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <csignal>
#include <vector>
#include <gz/common/StringUtils.hh>
#include <gz/common/Util.hh>
#include <gz/math/Rand.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>
#include <sdf/Mesh.hh>

#include "gz/sim/components/AxisAlignedBox.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/Util.hh"
#include "test_config.hh"

#include "plugins/MockSystem.hh"
#include "../test/helpers/Relay.hh"
#include "../test/helpers/EnvTestFixture.hh"
#include "../test/helpers/Util.hh"

using namespace gz;
using namespace gz::sim;
using namespace std::chrono_literals;

/////////////////////////////////////////////////
class ServerFixture : public InternalFixture<::testing::TestWithParam<int>>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(DefaultServerConfig))
{
  ServerConfig serverConfig;
  EXPECT_TRUE(serverConfig.SdfFile().empty());
  EXPECT_TRUE(serverConfig.SdfString().empty());
  EXPECT_FALSE(serverConfig.UpdateRate());
  EXPECT_FALSE(serverConfig.UseLevels());
  EXPECT_FALSE(serverConfig.UseDistributedSimulation());
  EXPECT_EQ(0u, serverConfig.NetworkSecondaries());
  EXPECT_TRUE(serverConfig.NetworkRole().empty());
  EXPECT_FALSE(serverConfig.UseLogRecord());
  EXPECT_FALSE(serverConfig.LogRecordPath().empty());
  EXPECT_TRUE(serverConfig.LogPlaybackPath().empty());
  EXPECT_FALSE(serverConfig.LogRecordResources());
  EXPECT_TRUE(serverConfig.LogRecordCompressPath().empty());
  EXPECT_EQ(0u, serverConfig.Seed());
  EXPECT_EQ(123ms, serverConfig.UpdatePeriod().value_or(123ms));
  EXPECT_TRUE(serverConfig.ResourceCache().empty());
  EXPECT_TRUE(serverConfig.PhysicsEngine().empty());
  EXPECT_TRUE(serverConfig.Plugins().empty());
  EXPECT_TRUE(serverConfig.LogRecordTopics().empty());

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(std::nullopt, server.Running(1));
  EXPECT_TRUE(*server.Paused());
  EXPECT_EQ(0u, *server.IterationCount());

  EXPECT_EQ(3u, *server.EntityCount());
  EXPECT_TRUE(server.HasEntity("default"));

  EXPECT_EQ(3u, *server.SystemCount());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, UpdateRate)
{
  sim::ServerConfig serverConfig;
  serverConfig.SetUpdateRate(1000.0);
  EXPECT_DOUBLE_EQ(1000.0, *serverConfig.UpdateRate());
  serverConfig.SetUpdateRate(-1000.0);
  EXPECT_DOUBLE_EQ(1000.0, *serverConfig.UpdateRate());
  serverConfig.SetUpdateRate(0.0);
  EXPECT_DOUBLE_EQ(1000.0, *serverConfig.UpdateRate());
  EXPECT_EQ(1ms, serverConfig.UpdatePeriod());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, ServerConfigPluginInfo)
{
  ServerConfig::PluginInfo pluginInfo;
  pluginInfo.SetEntityName("an_entity");
  pluginInfo.SetEntityType("model");

  sdf::Plugin sdfPlugin;
  sdfPlugin.SetFilename("filename");
  sdfPlugin.SetName("interface");
  pluginInfo.SetPlugin(sdfPlugin);

  ServerConfig serverConfig;
  serverConfig.AddPlugin(pluginInfo);

  const std::list<ServerConfig::PluginInfo> &plugins = serverConfig.Plugins();
  ASSERT_FALSE(plugins.empty());

  EXPECT_EQ("an_entity", plugins.front().EntityName());
  EXPECT_EQ("model", plugins.front().EntityType());

  // Test operator=
  {
    ServerConfig::PluginInfo info;
    info = plugins.front();

    EXPECT_EQ(info.EntityName(), plugins.front().EntityName());
    EXPECT_EQ(info.EntityType(), plugins.front().EntityType());
    EXPECT_EQ(info.Plugin().Name(), plugins.front().Plugin().Name());
    EXPECT_EQ(info.Plugin().Filename(), plugins.front().Plugin().Filename());
    EXPECT_EQ(info.Plugin().Element(), plugins.front().Plugin().Element());
  }

  // Test copy constructor
  {
    ServerConfig::PluginInfo info(plugins.front());

    EXPECT_EQ(info.EntityName(), plugins.front().EntityName());
    EXPECT_EQ(info.EntityType(), plugins.front().EntityType());
    EXPECT_EQ(info.Plugin().Name(), plugins.front().Plugin().Name());
    EXPECT_EQ(info.Plugin().Filename(), plugins.front().Plugin().Filename());
    EXPECT_EQ(info.Plugin().Element(), plugins.front().Plugin().Element());
  }

  // Test server config copy constructor
  {
    const ServerConfig &cfg(serverConfig);
    const std::list<ServerConfig::PluginInfo> &cfgPlugins = cfg.Plugins();
    ASSERT_FALSE(cfgPlugins.empty());

    EXPECT_EQ(cfgPlugins.front().EntityName(), plugins.front().EntityName());
    EXPECT_EQ(cfgPlugins.front().EntityType(), plugins.front().EntityType());
    EXPECT_EQ(cfgPlugins.front().Plugin().Filename(),
        plugins.front().Plugin().Filename());
    EXPECT_EQ(cfgPlugins.front().Plugin().Name(),
        plugins.front().Plugin().Name());
    EXPECT_EQ(cfgPlugins.front().Plugin().Element(),
        plugins.front().Plugin().Element());
  }
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, ServerConfigSdfPluginInfo)
{
  ServerConfig::PluginInfo pluginInfo;
  pluginInfo.SetEntityName("an_entity");
  pluginInfo.SetEntityType("model");
  sdf::Plugin plugin;
  plugin.SetFilename("filename");
  plugin.SetName("interface");
  pluginInfo.SetPlugin(plugin);

  gz::sim::ServerConfig serverConfig;
  serverConfig.AddPlugin(pluginInfo);

  const std::list<ServerConfig::PluginInfo> &plugins = serverConfig.Plugins();
  ASSERT_FALSE(plugins.empty());

  EXPECT_EQ("an_entity", plugins.front().EntityName());
  EXPECT_EQ("model", plugins.front().EntityType());
  EXPECT_EQ("filename", plugins.front().Plugin().Filename());
  EXPECT_EQ("interface", plugins.front().Plugin().Name());
  EXPECT_EQ(nullptr, plugins.front().Plugin().Element());
  EXPECT_TRUE(plugins.front().Plugin().Contents().empty());

  // Test operator=
  {
    ServerConfig::PluginInfo info;
    info = plugins.front();

    EXPECT_EQ(info.EntityName(), plugins.front().EntityName());
    EXPECT_EQ(info.EntityType(), plugins.front().EntityType());
    EXPECT_EQ(info.Plugin().Filename(), plugins.front().Plugin().Filename());
    EXPECT_EQ(info.Plugin().Name(), plugins.front().Plugin().Name());
    EXPECT_EQ(info.Plugin().ToElement()->ToString(""),
        plugins.front().Plugin().ToElement()->ToString(""));
  }

  // Test copy constructor
  {
    ServerConfig::PluginInfo info(plugins.front());

    EXPECT_EQ(info.EntityName(), plugins.front().EntityName());
    EXPECT_EQ(info.EntityType(), plugins.front().EntityType());
    EXPECT_EQ(info.Plugin().Filename(), plugins.front().Plugin().Filename());
    EXPECT_EQ(info.Plugin().Name(), plugins.front().Plugin().Name());
    EXPECT_EQ(info.Plugin().ToElement()->ToString(""),
        plugins.front().Plugin().ToElement()->ToString(""));
  }

  // Test server config copy constructor
  {
    const ServerConfig &cfg(serverConfig);
    const std::list<ServerConfig::PluginInfo> &cfgPlugins = cfg.Plugins();
    ASSERT_FALSE(cfgPlugins.empty());

    EXPECT_EQ(cfgPlugins.front().EntityName(), plugins.front().EntityName());
    EXPECT_EQ(cfgPlugins.front().EntityType(), plugins.front().EntityType());
    EXPECT_EQ(cfgPlugins.front().Plugin().Filename(),
        plugins.front().Plugin().Filename());
    EXPECT_EQ(cfgPlugins.front().Plugin().Name(),
        plugins.front().Plugin().Name());
    EXPECT_EQ(cfgPlugins.front().Plugin().ToElement()->ToString(""),
        plugins.front().Plugin().ToElement()->ToString(""));
  }
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(ServerConfigRealPlugin))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetUpdateRate(10000);
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sdf::ElementPtr sdf(new sdf::Element);
  sdf->SetName("plugin");
  sdf->AddAttribute("name", "string",
      "gz::sim::TestModelSystem", true);
  sdf->AddAttribute("filename", "string", "libTestModelSystem.so", true);

  sdf::ElementPtr child(new sdf::Element);
  child->SetParent(sdf);
  child->SetName("model_key");
  child->AddValue("string", "987", "1");

  ServerConfig::PluginInfo pluginInfo;
  pluginInfo.SetEntityName("box");
  pluginInfo.SetEntityType("model");
  sdf::Plugin plugin;
  plugin.SetFilename("TestModelSystem");
  plugin.SetName("gz::sim::TestModelSystem");
  pluginInfo.SetPlugin(plugin);

  serverConfig.AddPlugin(pluginInfo);

  sim::Server server(serverConfig);

  // The simulation runner should not be running.
  EXPECT_FALSE(*server.Running(0));

  // Run the server
  EXPECT_TRUE(server.Run(false, 0, false));
  EXPECT_FALSE(*server.Paused());

  // The TestModelSystem should have created a service. Call the service to
  // make sure the TestModelSystem was successfully loaded.
  transport::Node node;
  msgs::StringMsg rep;
  bool result{false};
  bool executed{false};
  const std::string service = "/test/service";
  ASSERT_TRUE(test::waitForService(node, service));
  gzdbg << "Requesting " << service << std::endl;
  executed = node.Request(service, 1000, rep, result);
  EXPECT_TRUE(executed);
  EXPECT_TRUE(result);
  EXPECT_EQ("TestModelSystem", rep.data());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ServerConfigSensorPlugin))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "air_pressure.sdf"));

  sdf::ElementPtr sdf(new sdf::Element);
  sdf->SetName("plugin");
  sdf->AddAttribute("name", "string",
      "gz::sim::TestSensorSystem", true);
  sdf->AddAttribute("filename", "string", "libTestSensorSystem.so", true);

  ServerConfig::PluginInfo pluginInfo;
  pluginInfo.SetEntityName(
      "air_pressure_sensor::air_pressure_model::link::air_pressure_sensor");
  pluginInfo.SetEntityType("sensor");
  sdf::Plugin plugin;
  plugin.Load(sdf);
  plugin.SetFilename("TestSensorSystem");
  plugin.SetName("gz::sim::TestSensorSystem");
  pluginInfo.SetPlugin(plugin);

  serverConfig.AddPlugin(pluginInfo);

  gzdbg << "Create server" << std::endl;
  sim::Server server(serverConfig);

  // The simulation runner should not be running.
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(3u, *server.SystemCount());

  // Run the server
  gzdbg << "Run server" << std::endl;
  EXPECT_TRUE(server.Run(false, 0, false));
  EXPECT_FALSE(*server.Paused());

  // The TestSensorSystem should have created a service. Call the service to
  // make sure the TestSensorSystem was successfully loaded.
  gzdbg << "Request service" << std::endl;
  transport::Node node;
  msgs::StringMsg rep;
  bool result{false};
  bool executed{false};
  const std::string service ="/test/service/sensor";
  ASSERT_TRUE(test::waitForService(node, service));
  gzdbg << "Requesting " << service << std::endl;
  executed = node.Request(service, 1000, rep, result);
  EXPECT_TRUE(executed);
  EXPECT_TRUE(result);
  EXPECT_EQ("TestSensorSystem", rep.data());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(SdfServerConfig))
{
  ServerConfig serverConfig;

  serverConfig.SetSdfString(TestWorldSansPhysics::World());
  EXPECT_TRUE(serverConfig.SdfFile().empty());
  EXPECT_FALSE(serverConfig.SdfString().empty());

  // Setting the SDF file should override the string.
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));
  EXPECT_FALSE(serverConfig.SdfFile().empty());
  EXPECT_TRUE(serverConfig.SdfString().empty());

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_TRUE(*server.Paused());
  EXPECT_EQ(0u, *server.IterationCount());
  EXPECT_EQ(25u, *server.EntityCount());
  EXPECT_EQ(3u, *server.SystemCount());

  EXPECT_TRUE(server.HasEntity("box"));
  EXPECT_FALSE(server.HasEntity("box", 1));
  EXPECT_TRUE(server.HasEntity("sphere"));
  EXPECT_TRUE(server.HasEntity("cylinder"));
  EXPECT_TRUE(server.HasEntity("capsule"));
  EXPECT_TRUE(server.HasEntity("ellipsoid"));
  EXPECT_FALSE(server.HasEntity("bad", 0));
  EXPECT_FALSE(server.HasEntity("bad", 1));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(SdfRootServerConfig))
{
  gz::sim::ServerConfig serverConfig;

  serverConfig.SetSdfString(TestWorldSansPhysics::World());
  EXPECT_TRUE(serverConfig.SdfFile().empty());
  EXPECT_FALSE(serverConfig.SdfString().empty());

  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "air_pressure.sdf"));
  EXPECT_FALSE(serverConfig.SdfFile().empty());
  EXPECT_TRUE(serverConfig.SdfString().empty());

  sdf::Root root;
  root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  // Setting the SDF Root should override the string and file.
  serverConfig.SetSdfRoot(root);

  EXPECT_TRUE(serverConfig.SdfRoot());
  EXPECT_TRUE(serverConfig.SdfFile().empty());
  EXPECT_TRUE(serverConfig.SdfString().empty());

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_TRUE(*server.Paused());
  EXPECT_EQ(0u, *server.IterationCount());
  EXPECT_EQ(25u, *server.EntityCount());
  EXPECT_EQ(3u, *server.SystemCount());

  EXPECT_TRUE(server.HasEntity("box"));
  EXPECT_FALSE(server.HasEntity("box", 1));
  EXPECT_TRUE(server.HasEntity("sphere"));
  EXPECT_TRUE(server.HasEntity("cylinder"));
  EXPECT_TRUE(server.HasEntity("capsule"));
  EXPECT_TRUE(server.HasEntity("ellipsoid"));
  EXPECT_FALSE(server.HasEntity("bad", 0));
  EXPECT_FALSE(server.HasEntity("bad", 1));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(ServerConfigLogRecord))
{
  auto logPath = common::joinPaths(PROJECT_BINARY_PATH, "test_log_path");
  auto logFile = common::joinPaths(logPath, "state.tlog");
  auto compressedFile = logPath + ".zip";

  gzdbg << "Log path [" << logPath << "]" << std::endl;

  common::removeAll(logPath);
  common::removeAll(compressedFile);
  EXPECT_FALSE(common::exists(logFile));
  EXPECT_FALSE(common::exists(compressedFile));

  {
    sim::ServerConfig serverConfig;
    serverConfig.SetUseLogRecord(true);
    serverConfig.SetLogRecordPath(logPath);

    sim::Server server(serverConfig);

    EXPECT_EQ(0u, *server.IterationCount());
    EXPECT_EQ(3u, *server.EntityCount());
    EXPECT_EQ(4u, *server.SystemCount());

    EXPECT_TRUE(serverConfig.LogRecordTopics().empty());
    serverConfig.AddLogRecordTopic("test_topic1");
    EXPECT_EQ(1u, serverConfig.LogRecordTopics().size());
    serverConfig.AddLogRecordTopic("test_topic2");
    EXPECT_EQ(2u, serverConfig.LogRecordTopics().size());
    serverConfig.ClearLogRecordTopics();
    EXPECT_TRUE(serverConfig.LogRecordTopics().empty());
  }

  EXPECT_TRUE(common::exists(logFile));
  EXPECT_FALSE(common::exists(compressedFile));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ServerConfigLogRecordCompress))
{
  auto logPath = common::joinPaths(PROJECT_BINARY_PATH, "test_log_path");
  auto logFile = common::joinPaths(logPath, "state.tlog");
  auto compressedFile = logPath + ".zip";

  gzdbg << "Log path [" << logPath << "]" << std::endl;

  common::removeAll(logPath);
  common::removeAll(compressedFile);
  EXPECT_FALSE(common::exists(logFile));
  EXPECT_FALSE(common::exists(compressedFile));

  {
    sim::ServerConfig serverConfig;
    serverConfig.SetUseLogRecord(true);
    serverConfig.SetLogRecordPath(logPath);
    serverConfig.SetLogRecordCompressPath(compressedFile);

    sim::Server server(serverConfig);
    EXPECT_EQ(0u, *server.IterationCount());
    EXPECT_EQ(3u, *server.EntityCount());
    EXPECT_EQ(4u, *server.SystemCount());
  }

  EXPECT_FALSE(common::exists(logFile));
  EXPECT_TRUE(common::exists(compressedFile));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, SdfStringServerConfig)
{
  ServerConfig serverConfig;

  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));
  EXPECT_FALSE(serverConfig.SdfFile().empty());
  EXPECT_TRUE(serverConfig.SdfString().empty());

  // Setting the string should override the file.
  serverConfig.SetSdfString(TestWorldSansPhysics::World());
  EXPECT_TRUE(serverConfig.SdfFile().empty());
  EXPECT_FALSE(serverConfig.SdfString().empty());
  EXPECT_FALSE(serverConfig.SdfRoot());

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_TRUE(*server.Paused());
  EXPECT_EQ(0u, *server.IterationCount());
  EXPECT_EQ(3u, *server.EntityCount());
  EXPECT_EQ(2u, *server.SystemCount());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, RunBlocking)
{
  sim::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_TRUE(*server.Paused());
  EXPECT_EQ(0u, server.IterationCount());

  // Make the server run fast.
  server.SetUpdatePeriod(1ns);

  uint64_t expectedIters = 0;
  for (uint64_t i = 1; i < 10; ++i)
  {
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));
    server.Run(true, i, false);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    expectedIters += i;
    EXPECT_EQ(expectedIters, *server.IterationCount());
  }
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, RunNonBlockingPaused)
{
  sim::Server server;

  // The server should not be running.
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // The simulation runner should not be running.
  EXPECT_FALSE(*server.Running(0));

  // Invalid world index.
  EXPECT_EQ(std::nullopt, server.Running(1));

  EXPECT_TRUE(*server.Paused());
  EXPECT_EQ(0u, *server.IterationCount());

  // Make the server run fast.
  server.SetUpdatePeriod(1ns);

  EXPECT_TRUE(server.Run(false, 100, true));
  EXPECT_TRUE(*server.Paused());

  EXPECT_TRUE(server.Running());

  // Add a small sleep because the non-blocking Run call causes the
  // simulation runner to start asynchronously.
  GZ_SLEEP_MS(500);
  EXPECT_TRUE(*server.Running(0));

  EXPECT_EQ(0u, server.IterationCount());

  // Attempt to unpause an invalid world
  EXPECT_FALSE(server.SetPaused(false, 1));

  // Unpause the existing world
  EXPECT_TRUE(server.SetPaused(false, 0));

  EXPECT_FALSE(*server.Paused());
  EXPECT_TRUE(server.Running());

  while (*server.IterationCount() < 100)
    GZ_SLEEP_MS(100);

  EXPECT_EQ(100u, *server.IterationCount());
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, RunNonBlocking)
{
  sim::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(0u, *server.IterationCount());

  // Make the server run fast.
  server.SetUpdatePeriod(1ns);

  server.Run(false, 100, false);
  while (*server.IterationCount() < 100)
    GZ_SLEEP_MS(100);

  EXPECT_EQ(100u, *server.IterationCount());
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(RunOnceUnpaused))
{
  sim::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(0u, *server.IterationCount());

  // Load a system
  sim::SystemLoader systemLoader;
  sdf::Plugin sdfPlugin;
  sdfPlugin.SetName("gz::sim::MockSystem");
  sdfPlugin.SetFilename("MockSystem");
  auto mockSystemPlugin = systemLoader.LoadPlugin(sdfPlugin);
  ASSERT_TRUE(mockSystemPlugin.has_value());

  // Query the interface from the plugin
  auto system = mockSystemPlugin.value()->QueryInterface<sim::System>();
  EXPECT_NE(system, nullptr);
  auto mockSystem = dynamic_cast<sim::MockSystem*>(system);
  EXPECT_NE(mockSystem, nullptr);

  Entity entity = server.EntityByName("default").value();
  size_t configureCallCount = 0;
  auto configureCallback = [&sdfPlugin, &entity, &configureCallCount](
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &,
    EventManager &)
  {
    configureCallCount++;
    EXPECT_EQ(entity, _entity);
    EXPECT_EQ(sdfPlugin.ToElement()->ToString(""), _sdf->ToString(""));
  };

  mockSystem->configureCallback = configureCallback;
  const size_t systemCount = *server.SystemCount();
  EXPECT_TRUE(
    *server.AddSystem(
      mockSystemPlugin.value(), entity, sdfPlugin.ToElement()
  ));

  // Add pointer
  auto mockSystemPtr = std::make_shared<MockSystem>();
  mockSystemPtr->configureCallback = configureCallback;
  EXPECT_TRUE(*server.AddSystem(mockSystemPtr, entity, sdfPlugin.ToElement()));

  // Add an sdf::Plugin
  EXPECT_TRUE(*server.AddSystem(sdfPlugin, entity));

  // Fail if plugin is invalid
  sdf::Plugin invalidPlugin("foo_plugin", "foo::systems::FooPlugin");
  EXPECT_FALSE(*server.AddSystem(invalidPlugin, entity));

  // Check that it was loaded
  EXPECT_EQ(systemCount + 3, *server.SystemCount());
  EXPECT_EQ(2u, configureCallCount);

  // No steps should have been executed
  EXPECT_EQ(0u, mockSystem->preUpdateCallCount);
  EXPECT_EQ(0u, mockSystem->updateCallCount);
  EXPECT_EQ(0u, mockSystem->postUpdateCallCount);

  // Make the server run fast
  server.SetUpdatePeriod(1ns);

  while (*server.IterationCount() < 100)
    server.RunOnce(false);

  // Check that the server provides the correct information
  EXPECT_EQ(*server.IterationCount(), 100u);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Check that the system has been called correctly
  EXPECT_EQ(100u, mockSystem->preUpdateCallCount);
  EXPECT_EQ(100u, mockSystem->updateCallCount);
  EXPECT_EQ(100u, mockSystem->postUpdateCallCount);
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(RunOncePaused))
{
  sim::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(0u, *server.IterationCount());

  // Load a system
  sim::SystemLoader systemLoader;
  sdf::Plugin sdfPlugin;
  sdfPlugin.SetName("gz::sim::MockSystem");
  sdfPlugin.SetFilename("MockSystem");
  auto mockSystemPlugin = systemLoader.LoadPlugin(sdfPlugin);
  ASSERT_TRUE(mockSystemPlugin.has_value());

  // Check that it was loaded
  const size_t systemCount = *server.SystemCount();
  EXPECT_TRUE(*server.AddSystem(mockSystemPlugin.value()));
  EXPECT_EQ(systemCount + 1, *server.SystemCount());

  // Query the interface from the plugin
  auto system = mockSystemPlugin.value()->QueryInterface<sim::System>();
  EXPECT_NE(system, nullptr);
  auto mockSystem = dynamic_cast<sim::MockSystem*>(system);
  EXPECT_NE(mockSystem, nullptr);

  // No steps should have been executed
  EXPECT_EQ(0u, mockSystem->preUpdateCallCount);
  EXPECT_EQ(0u, mockSystem->updateCallCount);
  EXPECT_EQ(0u, mockSystem->postUpdateCallCount);

  // Make the server run fast
  server.SetUpdatePeriod(1ns);

  while (*server.IterationCount() < 100)
    server.RunOnce(true);

  // Check that the server provides the correct information
  EXPECT_EQ(*server.IterationCount(), 100u);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Check that the system has been called correctly
  EXPECT_EQ(100u, mockSystem->preUpdateCallCount);
  EXPECT_EQ(100u, mockSystem->updateCallCount);
  EXPECT_EQ(100u, mockSystem->postUpdateCallCount);
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, RunNonBlockingMultiple)
{
  ServerConfig serverConfig;
  serverConfig.SetSdfString(TestWorldSansPhysics::World());
  sim::Server server(serverConfig);

  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(0u, *server.IterationCount());

  EXPECT_TRUE(server.Run(false, 100, false));
  EXPECT_FALSE(server.Run(false, 100, false));

  while (*server.IterationCount() < 100)
    GZ_SLEEP_MS(100);

  EXPECT_EQ(100u, *server.IterationCount());
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, SigInt)
{
  sim::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Run forever, non-blocking.
  server.Run(false, 0, false);

  GZ_SLEEP_MS(500);

  EXPECT_TRUE(server.Running());
  EXPECT_TRUE(*server.Running(0));

  std::raise(SIGTERM);

  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, ServerControlStop)
{
  // Test that the server correctly reacts to requests on /server_control
  // service with `stop` set to either false or true.

  sim::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Run forever, non-blocking.
  server.Run(false, 0, false);

  GZ_SLEEP_MS(500);

  EXPECT_TRUE(server.Running());
  EXPECT_TRUE(*server.Running(0));

  transport::Node node;
  msgs::ServerControl req;
  msgs::Boolean res;
  bool result{false};
  bool executed{false};

  const std::string service = "/server_control";
  ASSERT_TRUE(test::waitForService(node, service));
  // first, call with stop = false; the server should keep running
  gzdbg << "Requesting " << service << std::endl;
  executed = node.Request(service, req, 1000, res, result);
  EXPECT_TRUE(executed);
  EXPECT_TRUE(result);
  EXPECT_FALSE(res.data());

  GZ_SLEEP_MS(500);

  EXPECT_TRUE(server.Running());
  EXPECT_TRUE(*server.Running(0));

  // now call with stop = true; the server should stop
  req.set_stop(true);

  gzdbg << "Requesting " << service << std::endl;
  executed = node.Request(service, req, 1000, res, result);

  EXPECT_TRUE(executed);
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  GZ_SLEEP_MS(500);

  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(AddSystemWhileRunning))
{
  ServerConfig serverConfig;

  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  server.SetUpdatePeriod(1us);

  // Run the server to test whether we can add systems while system is running
  server.Run(false, 0, false);

  GZ_SLEEP_MS(500);

  EXPECT_TRUE(server.Running());
  EXPECT_TRUE(*server.Running(0));

  EXPECT_EQ(3u, *server.SystemCount());

  // Add system from plugin
  sim::SystemLoader systemLoader;
  sdf::Plugin sdfPlugin;
  sdfPlugin.SetName("gz::sim::MockSystem");
  sdfPlugin.SetFilename("MockSystem");
  auto mockSystemPlugin = systemLoader.LoadPlugin(sdfPlugin);
  ASSERT_TRUE(mockSystemPlugin.has_value());

  auto result = server.AddSystem(mockSystemPlugin.value());
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result.value());
  EXPECT_EQ(3u, *server.SystemCount());

  // Add system pointer
  auto mockSystem = std::make_shared<MockSystem>();
  result = server.AddSystem(mockSystem);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result.value());
  EXPECT_EQ(3u, *server.SystemCount());

  // Stop the server
  std::raise(SIGTERM);

  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(AddSystemAfterLoad))
{
  ServerConfig serverConfig;

  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Add system from plugin
  sim::SystemLoader systemLoader;
  sdf::Plugin sdfPlugin;
  sdfPlugin.SetName("gz::sim::MockSystem");
  sdfPlugin.SetFilename("MockSystem");
  auto mockSystemPlugin = systemLoader.LoadPlugin(sdfPlugin);
  ASSERT_TRUE(mockSystemPlugin.has_value());

  auto system = mockSystemPlugin.value()->QueryInterface<sim::System>();
  EXPECT_NE(system, nullptr);
  auto mockSystem = dynamic_cast<sim::MockSystem*>(system);
  ASSERT_NE(mockSystem, nullptr);

  EXPECT_EQ(3u, *server.SystemCount());
  EXPECT_EQ(0u, mockSystem->configureCallCount);

  EXPECT_TRUE(*server.AddSystem(mockSystemPlugin.value()));

  EXPECT_EQ(4u, *server.SystemCount());
  EXPECT_EQ(1u, mockSystem->configureCallCount);

  // Add system pointer
  auto mockSystemLocal = std::make_shared<MockSystem>();
  EXPECT_EQ(0u, mockSystemLocal->configureCallCount);

  EXPECT_TRUE(server.AddSystem(mockSystemLocal));
  EXPECT_EQ(5u, *server.SystemCount());
  EXPECT_EQ(1u, mockSystemLocal->configureCallCount);

  // Check that update callbacks are called
  server.SetUpdatePeriod(1us);
  EXPECT_EQ(0u, mockSystem->preUpdateCallCount);
  EXPECT_EQ(0u, mockSystem->updateCallCount);
  EXPECT_EQ(0u, mockSystem->postUpdateCallCount);
  EXPECT_EQ(0u, mockSystemLocal->preUpdateCallCount);
  EXPECT_EQ(0u, mockSystemLocal->updateCallCount);
  EXPECT_EQ(0u, mockSystemLocal->postUpdateCallCount);
  server.Run(true, 1, false);
  EXPECT_EQ(1u, mockSystem->preUpdateCallCount);
  EXPECT_EQ(1u, mockSystem->updateCallCount);
  EXPECT_EQ(1u, mockSystem->postUpdateCallCount);
  EXPECT_EQ(1u, mockSystemLocal->preUpdateCallCount);
  EXPECT_EQ(1u, mockSystemLocal->updateCallCount);
  EXPECT_EQ(1u, mockSystemLocal->postUpdateCallCount);

  // Add to inexistent world
  auto result = server.AddSystem(mockSystemPlugin.value(), 100);
  EXPECT_FALSE(result.has_value());

  result = server.AddSystem(mockSystemLocal, 100);
  EXPECT_FALSE(result.has_value());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, Seed)
{
  ServerConfig serverConfig;
  EXPECT_EQ(0u, serverConfig.Seed());
  unsigned int mySeed = 12345u;
  serverConfig.SetSeed(mySeed);
  EXPECT_EQ(mySeed, serverConfig.Seed());
  EXPECT_EQ(mySeed, math::Rand::Seed());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(ResourcePath))
{
  common::setenv("GZ_SIM_RESOURCE_PATH",
      (common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds:") +
       common::joinPaths(PROJECT_SOURCE_PATH,
           "test", "worlds", "models")).c_str());

  ServerConfig serverConfig;
  serverConfig.SetSdfFile("resource_paths.sdf");
  sim::Server server(serverConfig);

  test::Relay testSystem;
  unsigned int preUpdates{0};
  testSystem.OnPreUpdate(
    [&preUpdates](const sim::UpdateInfo &,
    sim::EntityComponentManager &_ecm)
    {
      // Create AABB so it is populated
      unsigned int eachCount{0};
      _ecm.Each<components::Model>(
        [&](const Entity &_entity, components::Model *) -> bool
        {
          auto bboxComp = _ecm.Component<components::AxisAlignedBox>(_entity);
          EXPECT_EQ(bboxComp, nullptr);
          _ecm.CreateComponent(_entity, components::AxisAlignedBox());
          eachCount++;
          return true;
        });
      EXPECT_EQ(1u, eachCount);
      preUpdates++;
    });

  unsigned int postUpdates{0};
  testSystem.OnPostUpdate([&postUpdates](const sim::UpdateInfo &,
    const sim::EntityComponentManager &_ecm)
    {
      // Check geometry components
      unsigned int eachCount{0};
      _ecm.Each<components::Geometry>(
        [&eachCount](const Entity &, const components::Geometry *_geom)
        -> bool
        {
          auto mesh = _geom->Data().MeshShape();

          // ASSERT would fail at compile with
          // "void value not ignored as it ought to be"
          EXPECT_NE(nullptr, mesh);

          if (mesh)
          {
            // StoreResolvedURIs is set to true so expect full path
            EXPECT_NE(std::string::npos,
                mesh->Uri().find("scheme_resource_uri/meshes/box.dae"));
            EXPECT_FALSE(common::isRelativePath(mesh->Uri()));
            EXPECT_TRUE(common::isFile(mesh->Uri()));
          }

          eachCount++;
          return true;
        });
      EXPECT_EQ(2u, eachCount);

      // Check physics system loaded meshes and got their BB correct
      eachCount = 0;
      _ecm.Each<components::AxisAlignedBox>(
        [&](const Entity &,
            const components::AxisAlignedBox *_box)->bool
        {
          auto box = _box->Data();
          EXPECT_EQ(box, math::AxisAlignedBox(-0.4, -0.4, 0.6, 0.4, 0.4, 1.4));
          eachCount++;
          return true;
        });
      EXPECT_EQ(1u, eachCount);

      postUpdates++;
    });
  server.AddSystem(testSystem.systemPtr);

  EXPECT_FALSE(*server.Running(0));

  EXPECT_TRUE(server.Run(true /*blocking*/, 1, false /*paused*/));
  EXPECT_EQ(1u, preUpdates);
  EXPECT_EQ(1u, postUpdates);

  EXPECT_EQ(7u, *server.EntityCount());
  EXPECT_TRUE(server.HasEntity("scheme_resource_uri"));
  EXPECT_TRUE(server.HasEntity("the_link"));
  EXPECT_TRUE(server.HasEntity("the_visual"));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, GetResourcePaths)
{
  common::setenv("GZ_SIM_RESOURCE_PATH",
      std::string("/tmp/some/path") +
      common::SystemPaths::Delimiter() +
      std::string("/home/user/another_path"));

  ServerConfig serverConfig;
  sim::Server server(serverConfig);

  EXPECT_FALSE(*server.Running(0));

  transport::Node node;
  msgs::StringMsg_V res;
  bool result{false};
  bool executed{false};
  const std::string service = "/gazebo/resource_paths/get";
  ASSERT_TRUE(test::waitForService(node, service));
  gzdbg << "Requesting " << service << std::endl;
  executed = node.Request(service, 1000, res, result);
  EXPECT_TRUE(executed);
  EXPECT_TRUE(result);
  EXPECT_EQ(2, res.data_size());
  EXPECT_EQ("/tmp/some/path", res.data(0));
  EXPECT_EQ("/home/user/another_path", res.data(1));
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, AddResourcePaths)
{
  common::setenv("GZ_SIM_RESOURCE_PATH",
      std::string("/tmp/some/path") +
      common::SystemPaths::Delimiter() +
      std::string("/home/user/another_path"));
  common::setenv("SDF_PATH", "");
  common::setenv("GZ_FILE_PATH", "");

  ServerConfig serverConfig;
  sim::Server server(serverConfig);

  EXPECT_FALSE(*server.Running(0));

  transport::Node node;

  // Subscribe to path updates
  bool receivedMsg{false};
  auto resourceCb = std::function<void(const msgs::StringMsg_V &)>(
      [&receivedMsg](const auto &_msg)
      {
        receivedMsg = true;
        EXPECT_EQ(5, _msg.data_size());
        EXPECT_EQ("/tmp/some/path", _msg.data(0));
        EXPECT_EQ("/home/user/another_path", _msg.data(1));
        EXPECT_EQ("/tmp/new_path", _msg.data(2));
        EXPECT_EQ("/tmp/more", _msg.data(3));
        EXPECT_EQ("/tmp/even_more", _msg.data(4));
      });
  node.Subscribe("/gazebo/resource_paths", resourceCb);

  // Add path
  msgs::StringMsg_V req;
  req.add_data("/tmp/new_path");
  req.add_data(std::string("/tmp/more") +
               common::SystemPaths::Delimiter() +
               std::string("/tmp/even_more"));
  req.add_data("/tmp/some/path");
  const std::string service = "/gazebo/resource_paths/add";
  ASSERT_TRUE(test::waitForService(node, service));
  bool executed = node.Request(service, req);
  EXPECT_TRUE(executed);

  int sleep{0};
  int maxSleep{30};
  while (!receivedMsg && sleep < maxSleep)
  {
    GZ_SLEEP_MS(50);
    sleep++;
  }
  EXPECT_TRUE(receivedMsg);

  // Check environment variables
  for (auto env : {"GZ_SIM_RESOURCE_PATH", "SDF_PATH", "GZ_FILE_PATH"})
  {
    char *pathCStr = std::getenv(env);

    auto paths = common::Split(pathCStr, common::SystemPaths::Delimiter());
    paths.erase(std::remove_if(paths.begin(), paths.end(),
        [](std::string const &_path)
        {
          return _path.empty();
        }),
        paths.end());

    EXPECT_EQ(5u, paths.size());
    EXPECT_EQ("/tmp/some/path", paths[0]);
    EXPECT_EQ("/home/user/another_path", paths[1]);
    EXPECT_EQ("/tmp/new_path", paths[2]);
    EXPECT_EQ("/tmp/more", paths[3]);
    EXPECT_EQ("/tmp/even_more", paths[4]);
  }
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, ResolveResourcePaths)
{
  common::setenv("GZ_SIM_RESOURCE_PATH", "");
  common::setenv("SDF_PATH", "");
  common::setenv("GZ_FILE_PATH", "");

  ServerConfig serverConfig;
  Server server(serverConfig);

  EXPECT_FALSE(*server.Running(0));

  auto test = std::function<void(const std::string _uri,
      const std::string &_expected, bool _found)>(
        [&](const std::string &_uri, const std::string &_expected, bool _found)
        {
          transport::Node node;
          msgs::StringMsg req, res;
          bool result{false};
          bool executed{false};

          req.set_data(_uri);
          const std::string service ="/gazebo/resource_paths/resolve";
          ASSERT_TRUE(test::waitForService(node, service));
          gzdbg << "Requesting " << service << std::endl;
          executed = node.Request(service, req, 1000, res, result);
          EXPECT_TRUE(executed);
          EXPECT_EQ(_found, result);
          EXPECT_EQ(_expected, res.data()) << "Expected[" << _expected
            << "] Received[" << res.data() << "]";
        });

  // Make sure the resource path is clear
  common::setenv("GZ_SIM_RESOURCE_PATH", "");

  // A valid path should be returned as an absolute path
  test(PROJECT_SOURCE_PATH, common::absPath(PROJECT_SOURCE_PATH), true);

  // An absolute path, with the file:// prefix, should return the absolute path
  test(std::string("file://") +
      PROJECT_SOURCE_PATH, PROJECT_SOURCE_PATH, true);

  // A non-absolute path with no RESOURCE_PATH should not find the resource
  test(common::joinPaths("test", "worlds", "plugins.sdf"), "", false);

  // Try again, this time with a RESOURCE_PATH
  common::setenv("GZ_SIM_RESOURCE_PATH", PROJECT_SOURCE_PATH);
  test(common::joinPaths("test", "worlds", "plugins.sdf"),
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "plugins.sdf"),
      true);
  // With the file:// prefix should also work
  test(std::string("file://") +
      common::joinPaths("test", "worlds", "plugins.sdf"),
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "plugins.sdf"),
      true);

  // The model:// URI should not resolve
  test("model://include_nested/model.sdf", "", false);
  common::setenv("GZ_SIM_RESOURCE_PATH",
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "models"));
  // The model:// URI should now resolve because the RESOURCE_PATH has been
  // updated.
  test("model://include_nested/model.sdf",
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "models",
        "include_nested", "model.sdf"), true);
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, Stop)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetUpdateRate(10000);
  serverConfig.SetSdfFile(common::joinPaths((PROJECT_SOURCE_PATH),
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);

  std::mutex testMutex;
  std::condition_variable testCv;
  test::Relay testSystem;
  unsigned int preUpdates{0};
  testSystem.OnPreUpdate(
    [&](const sim::UpdateInfo &,
    sim::EntityComponentManager &)
    {
      std::scoped_lock localLock(testMutex);
      ++preUpdates;
      testCv.notify_one();
      return true;
    });

  server.AddSystem(testSystem.systemPtr);
  // The simulation runner should not be running.
  EXPECT_FALSE(*server.Running(0));
  EXPECT_FALSE(server.Running());

  // Run the server.
  std::unique_lock<std::mutex> testLock(testMutex);
  EXPECT_TRUE(server.Run(false, 0, false));
  EXPECT_TRUE(server.Running());
  testCv.wait(testLock, [&]() -> bool {return preUpdates > 0;});
  EXPECT_TRUE(*server.Running(0));

  // Stop the server
  server.Stop();
  EXPECT_FALSE(*server.Running(0));
  EXPECT_FALSE(server.Running());
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_SUITE_P(ServerRepeat, ServerFixture, ::testing::Range(1, 2));

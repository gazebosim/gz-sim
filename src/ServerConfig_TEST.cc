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

#include <gz/common/Console.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/Util.hh>
#include <test_config.hh>

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
TEST(ParsePluginsFromString, Valid)
{
  std::string config = R"(
  <server_config>
    <plugins>
      <plugin
        entity_name="default"
        entity_type="world"
        filename="TestWorldSystem"
        name="gz::sim::TestWorldSystem">
        <world_key>0.123</world_key>
      </plugin>
      <plugin
        entity_name="box"
        entity_type="model"
        filename="TestModelSystem"
        name="gz::sim::TestModelSystem">
        <model_key>987</model_key>
      </plugin>
      <plugin
        entity_name="default::box::link_1::camera"
        entity_type="sensor"
        filename="TestSensorSystem"
        name="gz::sim::TestSensorSystem">
        <sensor_key>456</sensor_key>
      </plugin>
    </plugins>
  </server_config>)";

  auto plugins = parsePluginsFromString(config);
  ASSERT_EQ(3u, plugins.size());

  auto plugin = plugins.begin();

  EXPECT_EQ("default", plugin->EntityName());
  EXPECT_EQ("world", plugin->EntityType());
  EXPECT_EQ("TestWorldSystem", plugin->Plugin().Filename());
  EXPECT_EQ("gz::sim::TestWorldSystem", plugin->Plugin().Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("box", plugin->EntityName());
  EXPECT_EQ("model", plugin->EntityType());
  EXPECT_EQ("TestModelSystem", plugin->Plugin().Filename());
  EXPECT_EQ("gz::sim::TestModelSystem", plugin->Plugin().Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("default::box::link_1::camera", plugin->EntityName());
  EXPECT_EQ("sensor", plugin->EntityType());
  EXPECT_EQ("TestSensorSystem", plugin->Plugin().Filename());
  EXPECT_EQ("gz::sim::TestSensorSystem", plugin->Plugin().Name());
}

//////////////////////////////////////////////////
TEST(ParsePluginsFromString, Invalid)
{
  std::string config = R"(
  <server_config>
    <plugin
      entity_name="default"
      entity_type="world"
      filename="TestWorldSystem"
      name="gz::sim::TestWorldSystem">
      <world_key>0.123</world_key>
    </plugin>
  </server_config>)";

  auto plugins = parsePluginsFromString(config);
  ASSERT_EQ(0u, plugins.size());

  auto plugins2 = parsePluginsFromString("");
  ASSERT_EQ(0u, plugins2.size());
}

//////////////////////////////////////////////////
TEST(ParsePluginsFromFile, Valid)
{
  auto config = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "server_valid.config");

  auto plugins = parsePluginsFromFile(config);
  ASSERT_EQ(3u, plugins.size());

  auto plugin = plugins.begin();

  EXPECT_EQ("default", plugin->EntityName());
  EXPECT_EQ("world", plugin->EntityType());
  EXPECT_EQ("TestWorldSystem", plugin->Plugin().Filename());
  EXPECT_EQ("gz::sim::TestWorldSystem", plugin->Plugin().Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("box", plugin->EntityName());
  EXPECT_EQ("model", plugin->EntityType());
  EXPECT_EQ("TestModelSystem", plugin->Plugin().Filename());
  EXPECT_EQ("gz::sim::TestModelSystem", plugin->Plugin().Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("default::box::link_1::camera", plugin->EntityName());
  EXPECT_EQ("sensor", plugin->EntityType());
  EXPECT_EQ("TestSensorSystem", plugin->Plugin().Filename());
  EXPECT_EQ("gz::sim::TestSensorSystem", plugin->Plugin().Name());
}

//////////////////////////////////////////////////
TEST(ParsePluginsFromFile, Invalid)
{
  auto config = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "server_invalid.config");

  // Valid file without valid content
  auto plugins = parsePluginsFromFile(config);
  ASSERT_EQ(0u, plugins.size());

  // Invalid file
  auto plugins2 = parsePluginsFromFile("/foo/bar/baz");
  ASSERT_EQ(0u, plugins2.size());
}

//////////////////////////////////////////////////
TEST(ParsePluginsFromFile, DefaultConfig)
{
  // Note: This test validates that that the default
  // configuration always parses.
  // If more systems are added, then the number needs
  // to be adjusted below.
  auto config = common::joinPaths(PROJECT_SOURCE_PATH,
    "include", "gz", "sim", "server.config");

  auto plugins = parsePluginsFromFile(config);
  ASSERT_EQ(3u, plugins.size());
}

//////////////////////////////////////////////////
TEST(ParsePluginsFromFile, PlaybackConfig)
{
  // Note: This test validates that that the default
  // configuration always parses.
  // If more systems are added, then the number needs
  // to be adjusted below.
  auto config = common::joinPaths(PROJECT_SOURCE_PATH,
    "include", "gz", "sim", "playback_server.config");

  auto plugins = parsePluginsFromFile(config);
  ASSERT_EQ(2u, plugins.size());
}

//////////////////////////////////////////////////
TEST(LoadPluginInfo, FromEmptyEnv)
{
  // Set environment to something that doesn't exist
  ASSERT_TRUE(common::setenv(kServerConfigPathEnv, "foo"));
  auto plugins = loadPluginInfo();

  EXPECT_EQ(0u, plugins.size());
  EXPECT_TRUE(common::unsetenv(kServerConfigPathEnv));
}

//////////////////////////////////////////////////
TEST(LoadPluginInfo, FromValidEnv)
{
  auto validPath = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "server_valid2.config");

  ASSERT_TRUE(common::setenv(kServerConfigPathEnv, validPath));

  auto plugins = loadPluginInfo();
  ASSERT_EQ(2u, plugins.size());

  auto plugin = plugins.begin();

  EXPECT_EQ("*", plugin->EntityName());
  EXPECT_EQ("world", plugin->EntityType());
  EXPECT_EQ("TestWorldSystem", plugin->Plugin().Filename());
  EXPECT_EQ("gz::sim::TestWorldSystem", plugin->Plugin().Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("box", plugin->EntityName());
  EXPECT_EQ("model", plugin->EntityType());
  EXPECT_EQ("TestModelSystem", plugin->Plugin().Filename());
  EXPECT_EQ("gz::sim::TestModelSystem", plugin->Plugin().Name());

  EXPECT_TRUE(common::unsetenv(kServerConfigPathEnv));
}

//////////////////////////////////////////////////
TEST(ServerConfig, GenerateRecordPlugin)
{
  ServerConfig config;
  config.SetUseLogRecord(true);
  config.SetLogRecordPath("foo/bar");
  config.SetLogRecordResources(true);
  auto period =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(0.04));
  config.SetLogRecordPeriod(period);
  EXPECT_EQ(period, config.LogRecordPeriod());

  auto plugin = config.LogRecordPlugin();
  EXPECT_EQ(plugin.EntityName(), "*");
  EXPECT_EQ(plugin.EntityType(), "world");
  EXPECT_EQ(plugin.Plugin().Name(), "gz::sim::systems::LogRecord");
}

//////////////////////////////////////////////////
TEST(ServerConfig, SdfRoot)
{
  ServerConfig config;
  EXPECT_FALSE(config.SdfRoot());
  EXPECT_TRUE(config.SdfFile().empty());
  EXPECT_TRUE(config.SdfString().empty());
  EXPECT_EQ(ServerConfig::SourceType::kNone, config.Source());

  config.SetSdfString("string");
  EXPECT_FALSE(config.SdfRoot());
  EXPECT_TRUE(config.SdfFile().empty());
  EXPECT_FALSE(config.SdfString().empty());
  EXPECT_EQ(ServerConfig::SourceType::kSdfString, config.Source());

  config.SetSdfFile("file");
  EXPECT_FALSE(config.SdfRoot());
  EXPECT_FALSE(config.SdfFile().empty());
  EXPECT_TRUE(config.SdfString().empty());
  EXPECT_EQ(ServerConfig::SourceType::kSdfFile, config.Source());

  sdf::Root root;
  config.SetSdfRoot(root);
  EXPECT_TRUE(config.SdfRoot());
  EXPECT_TRUE(config.SdfFile().empty());
  EXPECT_TRUE(config.SdfString().empty());
  EXPECT_EQ(ServerConfig::SourceType::kSdfRoot, config.Source());
}

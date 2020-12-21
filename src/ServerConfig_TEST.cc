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

#include <ignition/common/Console.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/test_config.hh>

using namespace ignition;
using namespace gazebo;

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
        name="ignition::gazebo::TestWorldSystem">
        <world_key>0.123</world_key>
      </plugin>
      <plugin
        entity_name="box"
        entity_type="model"
        filename="TestModelSystem"
        name="ignition::gazebo::TestModelSystem">
        <model_key>987</model_key>
      </plugin>
      <plugin
        entity_name="default::box::link_1::camera"
        entity_type="sensor"
        filename="TestSensorSystem"
        name="ignition::gazebo::TestSensorSystem">
        <sensor_key>456</sensor_key>
      </plugin>
    </plugins>
  </server_config>)";

  auto plugins = parsePluginsFromString(config);
  ASSERT_EQ(3u, plugins.size());

  auto plugin = plugins.begin();

  EXPECT_EQ("default", plugin->EntityName());
  EXPECT_EQ("world", plugin->EntityType());
  EXPECT_EQ("TestWorldSystem", plugin->Filename());
  EXPECT_EQ("ignition::gazebo::TestWorldSystem", plugin->Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("box", plugin->EntityName());
  EXPECT_EQ("model", plugin->EntityType());
  EXPECT_EQ("TestModelSystem", plugin->Filename());
  EXPECT_EQ("ignition::gazebo::TestModelSystem", plugin->Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("default::box::link_1::camera", plugin->EntityName());
  EXPECT_EQ("sensor", plugin->EntityType());
  EXPECT_EQ("TestSensorSystem", plugin->Filename());
  EXPECT_EQ("ignition::gazebo::TestSensorSystem", plugin->Name());
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
      name="ignition::gazebo::TestWorldSystem">
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
  EXPECT_EQ("TestWorldSystem", plugin->Filename());
  EXPECT_EQ("ignition::gazebo::TestWorldSystem", plugin->Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("box", plugin->EntityName());
  EXPECT_EQ("model", plugin->EntityType());
  EXPECT_EQ("TestModelSystem", plugin->Filename());
  EXPECT_EQ("ignition::gazebo::TestModelSystem", plugin->Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("default::box::link_1::camera", plugin->EntityName());
  EXPECT_EQ("sensor", plugin->EntityType());
  EXPECT_EQ("TestSensorSystem", plugin->Filename());
  EXPECT_EQ("ignition::gazebo::TestSensorSystem", plugin->Name());
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
    "include", "ignition", "gazebo", "server.config");

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
    "include", "ignition", "gazebo", "playback_server.config");

  auto plugins = parsePluginsFromFile(config);
  ASSERT_EQ(2u, plugins.size());
}

//////////////////////////////////////////////////
TEST(LoadPluginInfo, FromEmptyEnv)
{
  // Set environment to something that doesn't exist
  ASSERT_TRUE(common::setenv(gazebo::kServerConfigPathEnv, "foo"));
  auto plugins = loadPluginInfo();

  EXPECT_EQ(0u, plugins.size());
  EXPECT_TRUE(common::unsetenv(gazebo::kServerConfigPathEnv));
}

//////////////////////////////////////////////////
TEST(LoadPluginInfo, FromValidEnv)
{
  auto validPath = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "server_valid2.config");

  ASSERT_TRUE(common::setenv(gazebo::kServerConfigPathEnv, validPath));

  auto plugins = loadPluginInfo();
  ASSERT_EQ(2u, plugins.size());

  auto plugin = plugins.begin();

  EXPECT_EQ("*", plugin->EntityName());
  EXPECT_EQ("world", plugin->EntityType());
  EXPECT_EQ("TestWorldSystem", plugin->Filename());
  EXPECT_EQ("ignition::gazebo::TestWorldSystem", plugin->Name());

  plugin = std::next(plugin, 1);

  EXPECT_EQ("box", plugin->EntityName());
  EXPECT_EQ("model", plugin->EntityType());
  EXPECT_EQ("TestModelSystem", plugin->Filename());
  EXPECT_EQ("ignition::gazebo::TestModelSystem", plugin->Name());

  EXPECT_TRUE(common::unsetenv(gazebo::kServerConfigPathEnv));
}

//////////////////////////////////////////////////
TEST(ServerConfig, GenerateRecordPlugin)
{
  ServerConfig config;
  config.SetUseLogRecord(true);
  config.SetLogRecordPath("foo/bar");
  config.SetLogRecordResources(true);

  auto plugin = config.LogRecordPlugin();
  EXPECT_EQ(plugin.EntityName(), "*");
  EXPECT_EQ(plugin.EntityType(), "world");
  EXPECT_EQ(plugin.Name(), "ignition::gazebo::systems::LogRecord");
}


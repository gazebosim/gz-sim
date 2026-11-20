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

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/TempDirectory.hh>
#include <gz/common/Util.hh>
#include <gz/sim/InstallationDirectories.hh>
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
/// \brief Points the home directory and the installation prefix to
/// temporary directories, with an installed server.config, and restores the
/// environment afterwards.
class LoadPluginInfoDefaultConfig : public ::testing::Test
{
  public: void SetUp() override
  {
    this->hadHome = gz::common::env(GZ_HOMEDIR, this->origHome);
    this->hadPrefix =
        gz::common::env("GZ_SIM_INSTALL_PREFIX", this->origPrefix);
    this->hadServerConfigEnv =
        gz::common::env(kServerConfigPathEnv, this->origServerConfigEnv);

    this->temp = std::make_unique<common::TempDirectory>(
        "server_config", "gz", true);
    ASSERT_TRUE(this->temp->Valid());

    // Installed config
    this->prefix = common::joinPaths(this->temp->Path(), "prefix");
    ASSERT_TRUE(common::setenv("GZ_SIM_INSTALL_PREFIX", this->prefix));
    if (getInstallPrefix() != this->prefix)
    {
      // For example, Bazel builds use a fixed install prefix.
      GTEST_SKIP() << "The installation prefix can't be overridden";
    }
    this->installedConfig = common::joinPaths(
        getServerConfigPath(), "server.config");
    ASSERT_TRUE(common::createDirectories(getServerConfigPath()));
    ASSERT_TRUE(common::copyFile(
        common::joinPaths(PROJECT_SOURCE_PATH, "include", "gz", "sim",
                          "server.config"),
        this->installedConfig));

    ASSERT_TRUE(common::unsetenv(kServerConfigPathEnv));
  }

  public: void TearDown() override
  {
    if (this->hadHome)
      common::setenv(GZ_HOMEDIR, this->origHome);
    else
      common::unsetenv(GZ_HOMEDIR);
    if (this->hadPrefix)
      common::setenv("GZ_SIM_INSTALL_PREFIX", this->origPrefix);
    else
      common::unsetenv("GZ_SIM_INSTALL_PREFIX");
    if (this->hadServerConfigEnv)
      common::setenv(kServerConfigPathEnv, this->origServerConfigEnv);
  }

  /// \brief Directory where the default config is copied for a given home
  public: static std::string ConfigDir(const std::string &_home)
  {
    return common::joinPaths(_home, ".gz", "sim", GZ_SIM_MAJOR_VERSION_STR);
  }

  public: std::unique_ptr<common::TempDirectory> temp;
  public: std::string prefix;
  public: std::string installedConfig;
  public: std::string origHome;
  public: std::string origPrefix;
  public: std::string origServerConfigEnv;
  public: bool hadHome{false};
  public: bool hadPrefix{false};
  public: bool hadServerConfigEnv{false};
};

//////////////////////////////////////////////////
TEST_F(LoadPluginInfoDefaultConfig, CopiesInstalledConfig)
{
  const auto expected = parsePluginsFromFile(this->installedConfig).size();
  ASSERT_LT(0u, expected);

  const auto home = common::joinPaths(this->temp->Path(), "home");
  ASSERT_TRUE(common::createDirectories(home));
  ASSERT_TRUE(common::setenv(GZ_HOMEDIR, home));

  EXPECT_EQ(expected, loadPluginInfo().size());

  // The installed config was copied, and nothing else was left behind.
  const auto configDir = ConfigDir(home);
  const auto copied = common::joinPaths(configDir, "server.config");
  EXPECT_TRUE(common::exists(copied));
  EXPECT_EQ(expected, parsePluginsFromFile(copied).size());

  std::vector<std::string> files;
  for (common::DirIter dir(configDir), end; dir != end; ++dir)
    files.push_back(common::basename(*dir));
  EXPECT_EQ(std::vector<std::string>{"server.config"}, files);
}

//////////////////////////////////////////////////
// Servers that start at the same time with a home directory that has no
// default config must all load the installed systems, and not read a config
// file that another server is still copying.
TEST_F(LoadPluginInfoDefaultConfig, ConcurrentFirstUse)
{
  const auto expected = parsePluginsFromFile(this->installedConfig).size();
  ASSERT_LT(0u, expected);

  constexpr int kThreads = 8;
  constexpr int kRounds = 100;

  for (int round = 0; round < kRounds; ++round)
  {
    const auto home = common::joinPaths(
        this->temp->Path(), "home" + std::to_string(round));
    ASSERT_TRUE(common::createDirectories(home));
    ASSERT_TRUE(common::setenv(GZ_HOMEDIR, home));

    std::atomic<bool> start{false};
    std::atomic<int> failures{0};
    std::vector<std::thread> threads;
    for (int i = 0; i < kThreads; ++i)
    {
      threads.emplace_back([&]()
      {
        while (!start)
          std::this_thread::yield();
        if (loadPluginInfo().size() != expected)
          ++failures;
      });
    }
    start = true;
    for (auto &thread : threads)
      thread.join();

    ASSERT_EQ(0, failures) << "round " << round;
  }
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

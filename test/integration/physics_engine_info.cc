/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include <optional>
#include <string>

#include <gz/math/SemanticVersion.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/PhysicsEngineInfo.hh"
#include "gz/sim/components/PhysicsEnginePlugin.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief World with the Physics system and a falling box
const char kWorld[] = R"(<?xml version="1.0"?>
<sdf version="1.12">
  <world name="default">
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
    </plugin>
    <model name="box">
      <pose>0 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>)";

/// \brief Engine components of the world entity, as seen by other systems
struct EngineComponents
{
  /// \brief PhysicsEnginePlugin, the plugin that was requested
  std::optional<std::string> plugin;

  /// \brief PhysicsEngineInfo, the engine that was loaded
  std::optional<std::string> info;
};

/// \brief Check that _info is _name, a space and a version that isn't 0.0.0
/// \param[in] _name Engine name.
/// \param[in] _info Content of the PhysicsEngineInfo component.
void ExpectNameAndVersion(const std::string &_name, const std::string &_info)
{
  const std::string prefix = _name + " ";
  ASSERT_EQ(prefix, _info.substr(0, prefix.size())) << _info;

  const std::string versionStr = _info.substr(prefix.size());
  const math::SemanticVersion version(versionStr);
  EXPECT_NE(math::SemanticVersion(), version) << _info;
  EXPECT_EQ(versionStr, version.Version()) << _info;
}

/// \brief Configuration of kWorld
/// \param[in] _engineLib Physics engine plugin to request, empty for the
/// default one.
ServerConfig Config(const std::string &_engineLib)
{
  ServerConfig config;
  config.SetSdfString(kWorld);
  if (!_engineLib.empty())
    config.SetPhysicsEngine(_engineLib);
  return config;
}

/// \brief Fixture that keeps the engine components of the world up to date
/// \param[in] _config Server configuration.
/// \param[out] _components Updated after every iteration. It must outlive the
/// returned fixture.
std::unique_ptr<TestFixture> Fixture(const ServerConfig &_config,
    EngineComponents &_components)
{
  auto fixture = std::make_unique<TestFixture>(_config);
  fixture->OnPostUpdate([&_components](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    const Entity world = worldEntity(_ecm);
    _components.plugin =
        _ecm.ComponentData<components::PhysicsEnginePlugin>(world);
    _components.info =
        _ecm.ComponentData<components::PhysicsEngineInfo>(world);
  }).Finalize();
  return fixture;
}
}  // namespace

/// \brief Test the physics engine information kept in the world entity
class PhysicsEngineInfoTest : public InternalFixture<::testing::Test>
{
};

/// \brief Same, for each engine. The parameter is the engine name, as the
/// engine reports it.
class PhysicsEngineInfoEnginesTest
  : public InternalFixture<::testing::TestWithParam<std::string>>
{
};

/////////////////////////////////////////////////
TEST_P(PhysicsEngineInfoEnginesTest, NameAndVersionOnWorldEntity)
{
  const std::string engineLib = "gz-physics-" + GetParam() + "-plugin";

  EngineComponents components;
  auto fixture = Fixture(Config(engineLib), components);
  fixture->Server()->Run(true, 1, false);

  // What was requested is left as it was, it must stay a loadable plugin name
  ASSERT_TRUE(components.plugin.has_value());
  EXPECT_EQ(engineLib, *components.plugin);

  ASSERT_TRUE(components.info.has_value());
  ExpectNameAndVersion(GetParam(), *components.info);
}

/////////////////////////////////////////////////
TEST_P(PhysicsEngineInfoEnginesTest, SurvivesWorldReset)
{
  const std::string engineLib = "gz-physics-" + GetParam() + "-plugin";

  EngineComponents components;
  auto fixture = Fixture(Config(engineLib), components);
  fixture->Server()->Run(true, 10, false);
  ASSERT_TRUE(components.info.has_value());
  const std::string infoBeforeReset = *components.info;

  fixture->Server()->ResetAll();
  components = {};
  fixture->Server()->Run(true, 10, false);

  ASSERT_TRUE(components.plugin.has_value());
  EXPECT_EQ(engineLib, *components.plugin);
  ASSERT_TRUE(components.info.has_value());
  EXPECT_EQ(infoBeforeReset, *components.info);
}

INSTANTIATE_TEST_SUITE_P(Engines, PhysicsEngineInfoEnginesTest,
    ::testing::Values("dartsim", "tpe", "bullet-featherstone"),
    [](const ::testing::TestParamInfo<std::string> &_info)
    {
      std::string name = _info.param;
      for (char &c : name)
      {
        if (c == '-')
          c = '_';
      }
      return name;
    });

/////////////////////////////////////////////////
TEST_F(PhysicsEngineInfoTest, DefaultEngine)
{
  EngineComponents components;
  auto fixture = Fixture(Config(""), components);
  fixture->Server()->Run(true, 1, false);

  ASSERT_TRUE(components.plugin.has_value());
  EXPECT_EQ("gz-physics-dartsim-plugin", *components.plugin);
  ASSERT_TRUE(components.info.has_value());
  ExpectNameAndVersion("dartsim", *components.info);
}

/////////////////////////////////////////////////
TEST_F(PhysicsEngineInfoTest, NoInfoWhenEngineFailsToLoad)
{
  const std::string engineLib = "gz-physics-does-not-exist-plugin";

  EngineComponents components;
  auto fixture = Fixture(Config(engineLib), components);
  fixture->Server()->Run(true, 1, false);

  // The component inspector falls back to the requested plugin in this case
  ASSERT_TRUE(components.plugin.has_value());
  EXPECT_EQ(engineLib, *components.plugin);
  EXPECT_FALSE(components.info.has_value());
}

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
 */

#include <gtest/gtest.h>

#include <chrono>

#include <gz/common/Util.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

using namespace gz;
using namespace sim;

/// \brief Test fixture for the AdvancedLiftDrag system.
class AdvancedLiftDragTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(AdvancedLiftDragTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ForceAtZeroAngleOfAttack))
{
  using namespace std::chrono_literals;

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH, "test",
      "worlds", "advanced_lift_drag.sdf"));
  Server server(serverConfig);
  server.SetUpdatePeriod(0ns);

  constexpr unsigned int testIterations = 5;
  // At zero alpha, both flat-plate terms vanish and sigma = 0.0122378065.
  // CL = (1 - sigma) * CL0 and CD = (1 - sigma) * (CD0 + CL^2 / (pi*AR*eff)).
  // With q * area = 81.8788 N, lift is along +Z and drag is along -X.
  constexpr double expectedLift = 12.2835658158;
  constexpr double expectedDrag = -2.4373223442;
  constexpr double tolerance = 1e-5;

  bool sampled = false;
  test::Relay relay;
  // Read the wrenches after the SDF systems run in PreUpdate and before
  // physics consumes them in Update.
  relay.OnPreUpdate([&](const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
  {
    if (_info.iterations != testIterations)
      return;

    const char *modelNames[] = {"level", "tilted"};
    for (const auto *modelName : modelNames)
    {
      SCOPED_TRACE(modelName);
      const auto model = _ecm.EntityByComponents(components::Model(),
          components::Name(modelName));
      ASSERT_NE(kNullEntity, model);
      const auto link = Model(model).LinkByName(_ecm, "base_link");
      ASSERT_NE(kNullEntity, link);

      const auto *wrench =
          _ecm.Component<components::ExternalWorldWrenchCmd>(link);
      ASSERT_NE(nullptr, wrench);

      const auto &force = wrench->Data().force();
      EXPECT_NEAR(expectedDrag, force.x(), tolerance);
      EXPECT_NEAR(0.0, force.y(), tolerance);
      EXPECT_NEAR(expectedLift, force.z(), tolerance);
    }
    sampled = true;
  });

  ASSERT_TRUE(server.AddSystem(relay.systemPtr).value_or(false));
  ASSERT_TRUE(server.Run(true, testIterations, false));
  EXPECT_TRUE(sampled);
}

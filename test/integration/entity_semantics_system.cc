/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#include <gz/common/Util.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "../helpers/EnvTestFixture.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/components/SemanticCategory.hh"
#include "gz/sim/components/SemanticDescription.hh"
#include "gz/sim/components/SemanticTags.hh"
#include "test_config.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test EntitySemantics system
class EntitySemanticsTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(EntitySemanticsTest, CanSetCategoriesAndTags)
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
                                        "test", "worlds",
                                        "entity_semantics.sdf"));

  bool testsExecuted = false;

  // Put the checks in an extra lambda as opposed to doing it OnPostUpdate so we
  // can call ASSERT_*
  auto runChecks = [](const sim::EntityComponentManager &_ecm)
  {
    auto robots = _ecm.EntitiesByComponents(components::SemanticCategory(1));
    ASSERT_EQ(1u, robots.size());

    auto robotDescription =
        _ecm.ComponentData<components::SemanticDescription>(robots[0]);
    ASSERT_TRUE(robotDescription .has_value());
    EXPECT_EQ("Food delivery mobile robot", *robotDescription);

    auto robotTags = _ecm.ComponentData<components::SemanticTags>(robots[0]);
    ASSERT_TRUE(robotTags.has_value());
    ASSERT_EQ(2u, robotTags->size());
    EXPECT_EQ("mobile", (*robotTags)[0]);
    EXPECT_EQ("diff_drive", (*robotTags)[1]);

    auto dynamicObjects =
        _ecm.EntitiesByComponents(components::SemanticCategory(4));
    EXPECT_EQ(2u, dynamicObjects.size());

    auto staticObjects =
        _ecm.EntitiesByComponents(components::SemanticCategory(5));
    EXPECT_EQ(1u, staticObjects.size());
  };

  fixture.OnPostUpdate(
      [&](const sim::UpdateInfo &, const sim::EntityComponentManager &_ecm)
      {
        runChecks(_ecm);
        testsExecuted = true;
        return true;
      });

  fixture.Finalize();
  fixture.Server()->Run(true, 1, false);
  EXPECT_TRUE(testsExecuted);
}

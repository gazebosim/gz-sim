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

#include <gz/common/Util.hh>

#include "../helpers/EnvTestFixture.hh"
#include "gz/sim/Server.hh"

using namespace gz;
using namespace sim;

/// \brief Test loading system from static plugin registry
class LoadSystemStaticRegistryTest : public InternalFixture<::testing::Test> {};

TEST_F(LoadSystemStaticRegistryTest, LoadWorks)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfString(R"(
    <?xml version="1.0"?>
    <sdf version="1.12">
      <world name="default">
        <model name="test_model1">
          <static>true</static>
          <link name="link_1" />
          <plugin filename="static://gz::sim::TestModelSystem"
                  name="gz::sim::TestModelSystem">
            <model_key>98765</model_key>
          </plugin>
        </model>
        <model name="test_model2">
          <static>true</static>
          <link name="link_1" />
          <!-- Plugin filename with registered alias -->
          <plugin filename="static://StaticTestModelSystem"
                  name="gz::sim::TestModelSystem">
            <model_key>54321</model_key>
          </plugin>
        </model>
      </world>
    </sdf>)");

  Server server(serverConfig);

  // Verify that server was initialized correctly
  auto iterationCount = server.IterationCount();
  ASSERT_NE(iterationCount, std::nullopt);
  ASSERT_EQ(*iterationCount, 0);

  std::optional<Entity> model1Id = server.EntityByName("test_model1");
  ASSERT_NE(model1Id, std::nullopt);
  std::optional<Entity> model2Id = server.EntityByName("test_model2");
  ASSERT_NE(model2Id, std::nullopt);

  server.RunOnce();

  // Verify that the System was instantiated for both models by checking the ECM
  // for the configured `ModelPluginComponent`.
  auto ecm = server.EcmScope();
  ASSERT_TRUE(ecm.Valid());
  const std::string modelComponentName{"ModelPluginComponent"};
  auto modelComponentId = common::hash64(modelComponentName);
  EXPECT_TRUE(ecm->HasComponentType(modelComponentId));
  EXPECT_TRUE(ecm->EntityHasComponentType(*model1Id, modelComponentId));
  EXPECT_TRUE(ecm->EntityHasComponentType(*model2Id, modelComponentId));
}

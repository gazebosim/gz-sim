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

#include <chrono>
#include <string>

#include <gz/sim/ServerConfig.hh>

#include "helpers/TestFixture.hh"

using namespace gz;
using namespace std::literals::chrono_literals;

//////////////////////////////////////////////////
TEST(TestFixtureHelperTest, StepDurationStopsWhenServerExited)
{
  const std::string badSdf = R"(
    <sdf version="1.12">
      <world name="test">
        <model name="bad_model_no_link" />
      </world>
    </sdf>)";

  sim::ServerConfig serverConfig;
  serverConfig.SetSdfString(badSdf);
  serverConfig.SetWaitForAssets(true);
  serverConfig.SetBehaviorOnSdfErrors(
      sim::ServerConfig::SdfErrorBehavior::EXIT_IMMEDIATELY);

  TestFixture fixture(serverConfig);
  auto *server = fixture.Simulator();
  ASSERT_NE(nullptr, server);
  EXPECT_EQ(sim::Server::Status::EXITED, server->GetStatus());

  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(0u, fixture.Step(100ms));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
}

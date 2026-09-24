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

#include <chrono>

#include <gtest/gtest.h>

#include <gz/common/Filesystem.hh>
#include <gz/sim/Server.hh>

#include "helpers/TestFixture.hh"

using namespace std::chrono_literals;

/////////////////////////////////////////////////
TEST(TestFixture, StepDurationStopsWhenServerExits)
{
  const auto missingWorld = gz::common::joinPaths(
      PROJECT_BINARY_PATH, "missing_test_fixture_world.sdf");
  TestFixture fixture(missingWorld);

  ASSERT_EQ(gz::sim::Server::Status::EXITED,
      fixture.Simulator()->GetStatus());
  EXPECT_EQ(0u, fixture.Step(1s));
}

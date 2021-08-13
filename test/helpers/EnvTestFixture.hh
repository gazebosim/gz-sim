/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_TEST_HELPERS_ENVTESTFIXTURE_HH_
#define IGNITION_GAZEBO_TEST_HELPERS_ENVTESTFIXTURE_HH_

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include "ignition/gazebo/test_config.hh"

using namespace ignition;

/// \brief Common test setup for various tests
template <typename TestType>
class InternalFixture : public TestType
{
  // Documentation inherited
  protected: void SetUp() override
  {
    // Augment the system plugin path.  In SetUp to avoid test order issues.
    common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           common::joinPaths(std::string(PROJECT_BINARY_PATH), "lib").c_str());

    common::Console::SetVerbosity(4);

    // Change environment variable so that test files aren't written to $HOME
    common::env(IGN_HOMEDIR, this->realHome);
    EXPECT_TRUE(common::setenv(IGN_HOMEDIR, this->kFakeHome.c_str()));
  }

  // Documentation inherited
  protected: void TearDown() override
  {
    // Restore $HOME
    EXPECT_TRUE(common::setenv(IGN_HOMEDIR, this->realHome.c_str()));
  }

  /// \brief Directory to act as $HOME for tests
  public: const std::string kFakeHome = common::joinPaths(PROJECT_BINARY_PATH,
      "test", "fake_home");

  /// \brief Store user's real $HOME to set it back at the end of tests.
  public: std::string realHome;
};
#endif

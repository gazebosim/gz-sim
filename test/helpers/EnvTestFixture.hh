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
#ifndef IGNITION_GAZEBO_TEST_HELPERS_ENVTESTFIXTURE_HH_
#define IGNITION_GAZEBO_TEST_HELPERS_ENVTESTFIXTURE_HH_

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include "ignition/gazebo/test_config.hh"

class ServerFixture : public ::testing::TestWithParam<int>
{
  protected: void SetUp() override
  {
    // Augment the system plugin path.  In SetUp to avoid test order issues.
    ignition::common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str());

    ignition::common::Console::SetVerbosity(4);
  }
};
#endif

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
#ifndef GZ_SIM_TEST_HELPERS_CUSTOMCACHEENV_HH_
#define GZ_SIM_TEST_HELPERS_CUSTOMCACHEENV_HH_

#include <gtest/gtest.h>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include "test_config.hh"

namespace gz
{
namespace sim
{
namespace test
{
/// \brief Helper class to be used in internal tests. It sets up a unique test
/// directory inside the cmake build directory (PROJECT_BINARY_PATH). The name
/// of the directory is specified in the constructor argument, but if another
/// directory with the same name is found, a new one with a numeric suffix will
/// be created ensuring that the new name is unique. This class can
/// be used, for example, for setting custom cache locations for tests that
/// download Fuel models.
///
/// An instance of the Environment has to be added to gtest using the
/// `::testing::AddGlobalTestEnvironment` function.
///
/// ## Usage
///
///  // In the test file
///  int main(int _argc, char **_argv)
///  {
///    ::testing::InitGoogleTest(&_argc, _argv);
///    ::testing::AddGlobalTestEnvironment(
///      new gz::sim::test::UniqueTestDirectoryEnv("custom_dir_name"));
///     return RUN_ALL_TESTS();
///  }
/// gtest is responsible for the instance, so there is no need to delete it.
class UniqueTestDirectoryEnv : public ::testing::Environment
{
  /// \brief Constructor
  /// \param[in] _dirName Directory name based on which a unique path is created
  public: explicit UniqueTestDirectoryEnv(const std::string &_dirName)
  {
    // We don't assert here because the assertion in the constructor won't be
    // caught by gtest.
    if (_dirName.empty())
    {
      gzerr << "_dirName cannot be empty\n";
    }
    else
    {
      UniqueTestDirectoryEnv::Path() = common::uniqueDirectoryPath(
          common::joinPaths(PROJECT_BINARY_PATH, _dirName));
    }
  }

  public: void SetUp() override
  {
    ASSERT_FALSE(UniqueTestDirectoryEnv::Path().empty())
        << "UniqueTestDirectoryEnv is not configured properly";

    common::createDirectory(UniqueTestDirectoryEnv::Path());
    ASSERT_TRUE(common::exists(UniqueTestDirectoryEnv::Path()));
  }

  public: void TearDown() override
  {
    if (!UniqueTestDirectoryEnv::Path().empty())
    {
      common::removeAll(UniqueTestDirectoryEnv::Path());
      UniqueTestDirectoryEnv::Path() = "";
    }
  }

  public: static std::string &Path()
  {
    static std::string dirPath = "";
    return dirPath;
  }
};
}
}
}
#endif

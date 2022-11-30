/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/TempDirectory.hh>
#include <gz/common/Util.hh>
#include <gz/math/SemanticVersion.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"

// File copied from
// https://github.com/gazebosim/gz-gui/raw/ign-gui3/test/integration/ExamplesBuild_TEST.cc

using namespace gz;

auto kExampleTypes = std::vector<std::string>{"plugin", "standalone"};

//////////////////////////////////////////////////
struct ExampleEntry
{
  /// Type of example (eg plugin/standalone)
  std::string type;

  /// Example plugin directory name
  std::string base;

  /// Full path to the source directory of the example
  std::string sourceDir;

  /// Used to pretty print with gtest
  /// \param[in] _os stream to print to
  /// \param[in] _example Entry to print to the stream
  friend std::ostream& operator<<(std::ostream &_os,
                                  const ExampleEntry &_entry)
  {
    return _os << "[" << _entry.type << ", " << _entry.base << "]";
  }
};

//////////////////////////////////////////////////
/// Filter examples that are known to not build or require
/// specific configurations
/// \param[in] _entry Example entry to check
/// \return true if example entry should be built, false otherwise
bool FilterEntry(const ExampleEntry &_entry)
{
  math::SemanticVersion cmakeVersion{std::string(CMAKE_VERSION)};
  if (cmakeVersion < math::SemanticVersion(3, 11, 0) &&
      (_entry.base == "custom_sensor_system" ||
       _entry.base == "gtest_setup"))
  {
    gzdbg << "Skipping [" << _entry.base
           << "] test, which requires CMake version "
           << ">= 3.11.0. Currently using CMake "
           << cmakeVersion
           << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
/// Generate a list of examples to be built.
std::vector<ExampleEntry> GetExamples()
{
  std::vector<ExampleEntry> examples;
  for (auto type : kExampleTypes)
  {
    auto examplesDir =
      common::joinPaths(PROJECT_SOURCE_PATH, "/examples/", type);

    // Iterate over directory
    gz::common::DirIter endIter;
    for (gz::common::DirIter dirIter(examplesDir);
        dirIter != endIter; ++dirIter)
    {
      auto base = gz::common::basename(*dirIter);
      auto sourceDir = common::joinPaths(examplesDir, base);
      examples.push_back({ type, base, sourceDir });
    }
  }
  return examples;
}

//////////////////////////////////////////////////
class ExamplesBuild
  : public InternalFixture<::testing::TestWithParam<ExampleEntry>>
{
  /// \brief Build code in a temporary build folder.
  /// \param[in] _type Type of example to build (plugins, standalone).
  public: void Build(const ExampleEntry &_type);
};

//////////////////////////////////////////////////
void ExamplesBuild::Build(const ExampleEntry &_entry)
{
  common::Console::SetVerbosity(4);

  if (!FilterEntry(_entry))
  {
    GTEST_SKIP();
  }

  // Path to examples of the given type
  ASSERT_TRUE(gz::common::exists(_entry.sourceDir));

  gzdbg << "Source: " << _entry.sourceDir << std::endl;

  // Create a temp build directory
  common::TempDirectory tmpBuildDir;
  ASSERT_TRUE(tmpBuildDir.Valid());
  gzdbg << "Build directory: " << tmpBuildDir.Path() << std::endl;

  char cmd[1024];

  // cd build && cmake source
  snprintf(cmd, sizeof(cmd), "cd %s && cmake %s",
    tmpBuildDir.Path().c_str(), _entry.sourceDir.c_str());

  ASSERT_EQ(system(cmd), 0) << _entry.sourceDir;
  ASSERT_EQ(system("make"), 0);
}

//////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_P(ExamplesBuild, GZ_UTILS_TEST_DISABLED_ON_WIN32(Build))
{
  Build(GetParam());
}

//////////////////////////////////////////////////
INSTANTIATE_TEST_SUITE_P(Plugins, ExamplesBuild,
    ::testing::ValuesIn(GetExamples()),
    [](const ::testing::TestParamInfo<ExamplesBuild::ParamType>& param) {
      return param.param.type + "_" + param.param.base;
    });

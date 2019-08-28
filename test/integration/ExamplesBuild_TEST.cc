/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <cstring>
#include <string>

#include "test_config.h"  // NOLINT(build/include)

// Helper functions copied from
// https://bitbucket.org/ignitionrobotics/ign-common/raw/default/src/Filesystem_TEST.cc

#ifndef _WIN32
#include <dirent.h>  // NOLINT(build/include_order)
#include <fcntl.h>  // NOLINT(build/include_order)
#include <limits.h>  // NOLINT(build/include_order)
#include <stdlib.h>  // NOLINT(build/include_order)
#include <sys/stat.h>  // NOLINT(build/include_order)
#include <sys/types.h>  // NOLINT(build/include_order)
#include <unistd.h>  // NOLINT(build/include_order)

/////////////////////////////////////////////////
bool createAndSwitchToTempDir(std::string &_newTempPath)
{
  std::string tmppath;
  const char *tmp = std::getenv("TMPDIR");
  if (tmp)
  {
    tmppath = std::string(tmp);
  }
  else
  {
    tmppath = std::string("/tmp");
  }

  tmppath += "/XXXXXX";

  char *dtemp = mkdtemp(const_cast<char *>(tmppath.c_str()));
  if (dtemp == nullptr)
  {
    return false;
  }
  if (chdir(dtemp) < 0)
  {
    return false;
  }

  char resolved[PATH_MAX];
  if (realpath(dtemp, resolved) == nullptr)
  {
    return false;
  }

  _newTempPath = std::string(resolved);

  return true;
}

#else
#include <windows.h>  // NOLINT(build/include_order)
#include <winnt.h>  // NOLINT(build/include_order)
#include <cstdint>

/////////////////////////////////////////////////
bool createAndSwitchToTempDir(std::string &_newTempPath)
{
  char tempPath[MAX_PATH + 1];
  DWORD pathLen = ::GetTempPathA(MAX_PATH, tempPath);
  if (pathLen >= MAX_PATH || pathLen <= 0)
  {
    return false;
  }
  std::string pathToCreate(tempPath);
  srand(static_cast<uint32_t>(time(nullptr)));

  for (int count = 0; count < 50; ++count)
  {
    // Try creating a new temporary directory with a randomly generated name.
    // If the one we chose exists, keep trying another path name until we reach
    // some limit.
    std::string newDirName;
    newDirName.append(std::to_string(::GetCurrentProcessId()));
    newDirName.push_back('_');
    // On Windows, rand_r() doesn't exist as an alternative to rand(), so the
    // cpplint warning is spurious.  This program is not multi-threaded, so
    // it is safe to suppress the threadsafe_fn warning here.
    newDirName.append(
       std::to_string(rand()    // NOLINT(runtime/threadsafe_fn)
                      % ((int16_t)0x7fff)));

    pathToCreate += newDirName;
    if (::CreateDirectoryA(pathToCreate.c_str(), nullptr))
    {
      _newTempPath = pathToCreate;
      return ::SetCurrentDirectoryA(_newTempPath.c_str()) != 0;
    }
  }

  return false;
}

#endif

//////////////////////////////////////////////////
void removeAll(const std::string &_path)
{
  DIR *dir = opendir(_path.c_str());
  if (dir)
  {
    struct dirent *p;
    while ((p=readdir(dir)))
    {
      // Skip special files.
      if (!std::strcmp(p->d_name, ".") || !std::strcmp(p->d_name, ".."))
        continue;

      removeAll(_path + "/" + p->d_name);
    }
  }
  closedir(dir);

  bool removed;
#ifdef _WIN32
  removed = RemoveDirectory(_path.c_str());
#else
  removed = (rmdir(_path.c_str()) == 0);
  if (!removed)
  {
    // A sym link would end up here
    removed = (std::remove(_path.c_str()) == 0);
  }
#endif
  ASSERT_TRUE(removed);
}

//////////////////////////////////////////////////
TEST(ExamplesBuild, Build)
{
  // Path to examples of the given type
  std::string examplesDir = std::string(PROJECT_SOURCE_PATH) + "/examples/";

  // Create a temp build directory
  std::string tmpBuildDir;
  ASSERT_TRUE(createAndSwitchToTempDir(tmpBuildDir));
  std::cout << "Build directory: " << tmpBuildDir<< std::endl;

  char cmd[1024];

  // cd build && cmake source
  snprintf(cmd, sizeof(cmd), "cd %s && cmake %s && make",
    tmpBuildDir.c_str(), examplesDir.c_str());
  ASSERT_EQ(system(cmd), 0);

  // Remove temp dir
  removeAll(tmpBuildDir);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

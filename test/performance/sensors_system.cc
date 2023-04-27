/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

// The following is needed to enable the GetMemInfo function for OSX
#ifdef __MACH__
# include <mach/mach.h>
#endif  // __MACH__

#include <gtest/gtest.h>

#include <gz/common/Console.hh>
#include <gz/rendering/RenderingIface.hh>

#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

/////////////////////////////////////////////////
void getMemInfo(double &_resident, double &_share)
{
#ifdef __linux__
  int totalSize, residentPages, sharePages;
  totalSize = residentPages = sharePages = 0;

  std::ifstream buffer("/proc/self/statm");
  buffer >> totalSize >> residentPages >> sharePages;
  buffer.close();

  // in case x86-64 is configured to use 2MB pages
  int64_t pageSizeKb = sysconf(_SC_PAGE_SIZE) / 1024;

  _resident = residentPages * pageSizeKb;
  _share = sharePages * pageSizeKb;
#elif __MACH__
  // /proc is only available on Linux
  // for OSX, use task_info to get resident and virtual memory
  struct task_basic_info t_info;
  mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;
  if (KERN_SUCCESS != task_info(mach_task_self(),
                                TASK_BASIC_INFO,
                                (task_info_t)&t_info,
                                &t_info_count))
  {
    gzerr << "failure calling task_info\n";
    return;
  }
  _resident = static_cast<double>(t_info.resident_size/1024);
  _share = static_cast<double>(t_info.virtual_size/1024);
#else
  gzerr << "Unsupported architecture\n";
  return;
#endif
}

/////////////////////////////////////////////////
class SensorsSystemFixture : public InternalFixture<::testing::Test>
{
};


/////////////////////////////////////////////////
// Test repeatedly launching sim with sensors system and check for
// memory leaks
TEST_F(SensorsSystemFixture, GZ_UTILS_TEST_DISABLED_ON_MAC(MemLeak))
{
  gz::common::Console::SetVerbosity(4);

  // max memory change allowed
  const double resMaxPercentChange = 1.0;
  const double shareMaxPercentChange = 1.0;

  std::vector<double> residentMemV;
  std::vector<double> shareMemV;

  for (unsigned int i = 0; i < 5; ++i)
  {
    // get resident and shared memory usage
    double residentMem = 0;
    double shareMem = 0;
    getMemInfo(residentMem, shareMem);

    residentMemV.push_back(residentMem);
    shareMemV.push_back(shareMem);

    gz::sim::ServerConfig serverConfig;

    const std::string sdfFile = std::string(PROJECT_SOURCE_PATH) +
        "/test/worlds/sensors_system_mem_leak.sdf";

    serverConfig.SetSdfFile(sdfFile);

    gz::sim::Server server(serverConfig);
    server.Run(true, 100, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // Calculate the percent change between runs
  // Start from index 1 (instead of 0) since there is a big jump in memory usage
  // when the system is first loaded. Make sure subsequent runs do not continue
  // to increase memory usage
  for (unsigned int i = 1; i < residentMemV.size() - 1; ++i)
  {
    double resPercentChange =
        (residentMemV[i+1] - residentMemV[i]) / residentMemV[i];
    double sharePercentChange = (shareMemV[i+1] - shareMemV[i]) / shareMemV[i];

    gzdbg << "ResPercentChange[" << resPercentChange << "] "
      << "ResMaxPercentChange[" << resMaxPercentChange << "]" << std::endl;
    gzdbg << "SharePercentChange[" << sharePercentChange << "] "
      << "ShareMaxPercentChange[" << shareMaxPercentChange << "]" << std::endl;

    // check for memory leaks
    // \todo(anyone) there is still gradual slow increase in memory usage
    // between runs, likely from the sensors system / gz-rendering.
    // Use tighter max percentage change once more mem leaks are fixed
    EXPECT_LT(resPercentChange, resMaxPercentChange);
    EXPECT_LT(sharePercentChange, shareMaxPercentChange);
  }
}

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

#include <benchmark/benchmark.h>

#include <gz/common/Util.hh>
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"

#include "test_config.hh"

using namespace gz;
using namespace sim;
using namespace components;

void BM_RuntimeWorld(benchmark::State &_st, std::string physics_engine,
                     std::string world_sdf) {
  // world_sdf file should be in test/worlds/

  // const std::vector<std::string> kPhysicsPlugins = {
  //     "gz-physics-dartsim-plugin", "gz-physics-bullet-featherstone-plugin"};

  // const std::vector<std::string> worldFiles = {"shapes.sdf",
  //                                              "gpu_lidar_sensor.sdf"};

  // std::string world_file = worldFiles[_st.range(1)];
  // std::string physics_engine = kPhysicsPlugins[_st.range(0)];
  auto stabilizingSteps = _st.range(0);

  std::string path = std::string(PROJECT_SOURCE_PATH) + "/test/worlds/models";
  common::setenv("GZ_SIM_RESOURCE_PATH", path.c_str());
  ServerConfig serverConfig;
  serverConfig.SetWaitForAssets(true);
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
                                            "test/worlds/", world_sdf));
  serverConfig.SetPhysicsEngine(physics_engine);

  sim::Server server(serverConfig); // Add system from plugin

  for (auto _ : _st) {
    _st.PauseTiming();
    // wait for it to stabilize before timing?
    server.Run(true, stabilizingSteps, false);
    _st.ResumeTiming();
    server.Run(true, 1000, false);
  }
}

// NOLINTNEXTLINE
BENCHMARK_CAPTURE(BM_RuntimeWorld, bullet_shapes_sdf,
                  std::string("gz-physics-bullet-featherstone-plugin"),
                  std::string("shapes.sdf"))
    ->Arg(1)
    ->Arg(10)
    ->Arg(100)
    ->Arg(1000)
    ->Unit(benchmark::kMillisecond);

// NOLINTNEXTLINE
BENCHMARK_CAPTURE(BM_RuntimeWorld, bullet_gpu_lidar_sensor_sdf,
                  std::string("gz-physics-bullet-featherstone-plugin"),
                  std::string("gpu_lidar_sensor.sdf"))
    ->Arg(1)
    ->Arg(10)
    ->Arg(100)
    ->Arg(1000)
    ->Unit(benchmark::kMillisecond);

// NOLINTNEXTLINE
BENCHMARK_CAPTURE(BM_RuntimeWorld, bullet_breadcrumbs_sdf,
                  std::string("gz-physics-bullet-featherstone-plugin"),
                  std::string("breadcrumbs.sdf"))
    ->Arg(1)
    ->Arg(10)
    ->Arg(100)
    ->Arg(1000)
    ->Unit(benchmark::kMillisecond);

// // NOLINTNEXTLINE
// BENCHMARK_CAPTURE(BM_RuntimeWorld, bullet_3k_shapes_sdf,
//                   std::string("gz-physics-bullet-featherstone-plugin"),
//                   std::string("3k_shapes.sdf"))
//     ->Arg(1)
//     ->Arg(10)
//     ->Arg(100)
//     ->Arg(1000)
//     ->Unit(benchmark::kMillisecond);

// OSX needs the semicolon, Ubuntu complains that there's an extra ';'
#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif

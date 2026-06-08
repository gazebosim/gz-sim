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
#include <cstring>
#include <vector>

#include <gz/common/Util.hh>
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"

#include "test_config.hh"

using namespace gz;
using namespace sim;
using namespace components;

void BM_RuntimeWorld(benchmark::State &_st, const std::string &_physics_engine,
                     const std::string &_world_sdf) {
  auto stabilizingSteps = _st.range(0);

  std::string path = common::joinPaths(std::string(PROJECT_SOURCE_PATH), "/test/worlds/models");
  common::setenv("GZ_SIM_RESOURCE_PATH", path.c_str());
  ServerConfig serverConfig;
  serverConfig.SetWaitForAssets(true);
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
                                            "test/worlds/", _world_sdf));
  serverConfig.SetPhysicsEngine(_physics_engine);

  for (auto _ : _st) {
    _st.PauseTiming();
    sim::Server server(serverConfig); // Add system from plugin
    // Wait for simulation to stabilize before timing
    server.Run(true, stabilizingSteps, false);
    _st.ResumeTiming();

    server.Run(true, 1000, false);
  }
}

// NOLINTNEXTLINE
BENCHMARK_CAPTURE(BM_RuntimeWorld, bullet_shapes_sdf,
                  "gz-physics-bullet-featherstone-plugin",
                  "shapes.sdf")
    ->Arg(10000)
    ->Unit(benchmark::kMillisecond);

// NOLINTNEXTLINE
BENCHMARK_CAPTURE(BM_RuntimeWorld, bullet_gpu_lidar_sensor_sdf,
                  "gz-physics-bullet-featherstone-plugin",
                  "gpu_lidar_sensor.sdf")
    ->Arg(10000)
    ->Unit(benchmark::kMillisecond);

// NOLINTNEXTLINE
BENCHMARK_CAPTURE(BM_RuntimeWorld, bullet_breadcrumbs_sdf,
                  "gz-physics-bullet-featherstone-plugin",
                  "breadcrumbs.sdf")
    ->Arg(10000)
    ->Unit(benchmark::kMillisecond);

// NOLINTNEXTLINE
BENCHMARK_CAPTURE(BM_RuntimeWorld, bullet_3k_shapes_sdf,
                  "gz-physics-bullet-featherstone-plugin",
                  "3k_shapes.sdf")
    ->Arg(10000)
    ->Unit(benchmark::kMillisecond);

// OSX needs the semicolon, Ubuntu complains that there's an extra ';'
#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif

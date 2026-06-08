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

#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/EntityComponentManager.hh"

#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;

void BM_ScopedName(benchmark::State &_st) {

  // world
  // - modelA
  //   - modelB
  //     - linkC
  //       - sensorD

  // todo:
  // call on different entity types
  // call on different depth levels
  EntityComponentManager ecm;

  auto worldEntity = ecm.CreateEntity();
  ecm.CreateComponent(worldEntity, components::World());
  ecm.CreateComponent(worldEntity, components::Name("world_name"));

  // Model A
  auto modelAEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelAEntity, components::Model());
  ecm.CreateComponent(modelAEntity, components::Name("modelA_name"));
  ecm.CreateComponent(modelAEntity, components::ParentEntity(worldEntity));

  // Model B
  auto modelBEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelBEntity, components::Model());
  ecm.CreateComponent(modelBEntity, components::Name("modelB_name"));
  ecm.CreateComponent(modelBEntity, components::ParentEntity(modelAEntity));

  // Link C
  auto linkCEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkCEntity, components::Link());
  ecm.CreateComponent(linkCEntity, components::Name("linkC_name"));
  ecm.CreateComponent(linkCEntity, components::ParentEntity(modelBEntity));

  // Sensor D
  auto sensorDEntity = ecm.CreateEntity();
  ecm.CreateComponent(sensorDEntity, components::Sensor());
  ecm.CreateComponent(sensorDEntity, components::Name("sensorD_name"));
  ecm.CreateComponent(sensorDEntity, components::ParentEntity(linkCEntity));

  auto iterations = _st.range(0);
  for (auto _ : _st) {
    for (int ii = 0; ii < iterations; ++ii)
    {
      scopedName(sensorDEntity, ecm);
    }
  }
}

// NOLINTNEXTLINE
BENCHMARK(BM_ScopedName)
    ->Arg(1000)
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

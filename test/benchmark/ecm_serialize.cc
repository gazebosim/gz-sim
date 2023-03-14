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

#include <memory>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"

#include "gz/sim/components/Factory.hh"


namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
using IntComponent = components::Component<int, class IntComponentTag>;
GZ_SIM_REGISTER_COMPONENT("ign_gazebo_components.IntComponent",
    IntComponent)

using UIntComponent = components::Component<int, class IntComponentTag>;
GZ_SIM_REGISTER_COMPONENT("ign_gazebo_components.UIntComponent",
    UIntComponent)

using DoubleComponent = components::Component<double, class DoubleComponentTag>;
GZ_SIM_REGISTER_COMPONENT("ign_gazebo_components.DoubleComponent",
    DoubleComponent)

using StringComponent =
    components::Component<std::string, class StringComponentTag>;
GZ_SIM_REGISTER_COMPONENT("ign_gazebo_components.StringComponent",
    StringComponent)

using BoolComponent = components::Component<bool, class BoolComponentTag>;
GZ_SIM_REGISTER_COMPONENT("ign_gazebo_components.BoolComponent",
    BoolComponent)
}
}
}
}


using namespace gz;
using namespace sim;
using namespace components;

// NOLINTNEXTLINE
void BM_Serialize1Component(benchmark::State &_st)
{
  size_t serializedSize = 0;
  auto entityCount = _st.range(0);
  for (auto _: _st)
  {
    _st.PauseTiming();
    auto mgr = std::make_unique<EntityComponentManager>();
    for (int ii = 0; ii < entityCount; ++ii)
    {
      auto e = mgr->CreateEntity();
      mgr->CreateComponent(e, IntComponent(ii));
    }
    _st.ResumeTiming();

    auto stateMsg = mgr->State();
#if GOOGLE_PROTOBUF_VERSION >= 3004000
    serializedSize = stateMsg.ByteSizeLong();
#else
    serializedSize = stateMsg.ByteSize();
#endif
  }
  _st.counters["serialized_size"] = static_cast<double>(serializedSize);
  _st.counters["num_entities"] = static_cast<double>(entityCount);
  _st.counters["num_components"] = 1;
}

// NOLINTNEXTLINE
void BM_Serialize5Component(benchmark::State &_st)
{
  size_t serializedSize = 0;
  auto entityCount = _st.range(0);
  for (auto _: _st)
  {
    _st.PauseTiming();
    auto mgr = std::make_unique<EntityComponentManager>();
    for (int ii = 0; ii < entityCount; ++ii)
    {
      auto e = mgr->CreateEntity();
      mgr->CreateComponent(e, IntComponent(ii));
      mgr->CreateComponent(e, UIntComponent(ii));
      mgr->CreateComponent(e, DoubleComponent(ii));
      mgr->CreateComponent(e, StringComponent("foobar"));
      mgr->CreateComponent(e, BoolComponent(ii%2));
    }
    _st.ResumeTiming();

    auto stateMsg = mgr->State();
#if GOOGLE_PROTOBUF_VERSION >= 3004000
    serializedSize = stateMsg.ByteSizeLong();
#else
    serializedSize = stateMsg.ByteSize();
#endif
  }
  _st.counters["serialized_size"] = static_cast<double>(serializedSize);
  _st.counters["num_entities"] = static_cast<double>(entityCount);
  _st.counters["num_components"] = 5;
}

// NOLINTNEXTLINE
BENCHMARK(BM_Serialize1Component)
  ->Arg(10)
  ->Arg(50)
  ->Arg(100)
  ->Arg(500)
  ->Arg(1000)
  ->Unit(benchmark::kMillisecond);

// NOLINTNEXTLINE
BENCHMARK(BM_Serialize5Component)
  ->Arg(10)
  ->Arg(50)
  ->Arg(100)
  ->Arg(500)
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

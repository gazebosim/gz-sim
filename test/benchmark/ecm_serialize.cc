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

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"

#include "ignition/gazebo/components/Factory.hh"


namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
using IntComponent = components::Component<int, class IntComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.IntComponent",
    IntComponent)

using UIntComponent = components::Component<int, class IntComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.UIntComponent",
    UIntComponent)

using DoubleComponent = components::Component<double, class DoubleComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.DoubleComponent",
    DoubleComponent)

using StringComponent =
    components::Component<std::string, class StringComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.StringComponent",
    StringComponent)

using BoolComponent = components::Component<bool, class BoolComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.BoolComponent",
    BoolComponent)
}
}
}
}


using namespace ignition;
using namespace gazebo;
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
  _st.counters["serialized_size"] = serializedSize;
  _st.counters["num_entities"] = entityCount;
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
  _st.counters["serialized_size"] = serializedSize;
  _st.counters["num_entities"] = entityCount;
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
BENCHMARK_MAIN();
#pragma GCC diagnostic pop

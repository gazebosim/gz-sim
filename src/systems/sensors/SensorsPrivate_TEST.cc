/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <chrono>
#include <future>
#include <mutex>
#include <set>
#include <utility>

// SensorsPrivate is intentionally local to Sensors.cc. Compile the production
// implementation into this test so the lifecycle lock can be held at a
// deterministic interleaving point.
#include "Sensors.cc"

using namespace std::chrono_literals;

namespace {
template <typename Callable>
void ExpectReadWaitsForLifecycleMutation(Callable &&_callable) {
  gz::sim::systems::SensorsPrivate data;
  std::unique_lock<std::mutex> mutationLock(data.sensorsMutex);
  std::promise<void> started;
  auto startedFuture = started.get_future();
  auto read =
      std::async(std::launch::async,
                 [&data, &started,
                  callable = std::forward<Callable>(_callable)]() mutable {
                   started.set_value();
                   callable(data);
                 });

  ASSERT_EQ(std::future_status::ready, startedFuture.wait_for(1s));
  EXPECT_EQ(std::future_status::timeout, read.wait_for(100ms));

  mutationLock.unlock();
  EXPECT_EQ(std::future_status::ready, read.wait_for(1s));
}
} // namespace

/////////////////////////////////////////////////
TEST(SensorsPrivate, ConnectionReadWaitsForLifecycleMutation) {
  ExpectReadWaitsForLifecycleMutation(
      [](gz::sim::systems::SensorsPrivate &_data) {
        static_cast<void>(_data.SensorsHaveConnections());
      });
}

/////////////////////////////////////////////////
TEST(SensorsPrivate, NextUpdateReadWaitsForLifecycleMutation) {
  ExpectReadWaitsForLifecycleMutation(
      [](gz::sim::systems::SensorsPrivate &_data) {
        std::set<gz::sensors::SensorId> sensorsToUpdate;
        static_cast<void>(_data.NextUpdateTime(sensorsToUpdate, {}));
      });
}

/////////////////////////////////////////////////
TEST(SensorsPrivate, TriggerReadWaitsForLifecycleMutation) {
  ExpectReadWaitsForLifecycleMutation(
      [](gz::sim::systems::SensorsPrivate &_data) {
        static_cast<void>(_data.SensorsWithPendingTrigger());
      });
}

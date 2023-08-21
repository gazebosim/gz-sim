/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <atomic>

#include "gz/sim/Events.hh"
#include "gz/sim/EventManager.hh"

using namespace gz::sim;

/////////////////////////////////////////////////
TEST(EventManager, EmitConnectTest)
{
  EventManager eventManager;

  EXPECT_EQ(0u, eventManager.ConnectionCount<events::Pause>());

  bool paused1 = false;
  auto connection1 = eventManager.Connect<events::Pause>(
    [&paused1](bool _paused) {
      paused1 = _paused;
    });

  EXPECT_EQ(1u, eventManager.ConnectionCount<events::Pause>());

  // Emitting events causes connection callbacks to be fired.
  eventManager.Emit<events::Pause>(true);
  EXPECT_EQ(true, paused1);
  eventManager.Emit<events::Pause>(false);
  EXPECT_EQ(false, paused1);

  bool paused2 = false;
  auto connection2 = eventManager.Connect<events::Pause>(
    [&paused2](bool _paused) {
      paused2 = _paused;
    });

  EXPECT_EQ(2u, eventManager.ConnectionCount<events::Pause>());

  // Multiple connections should each be fired.
  eventManager.Emit<events::Pause>(true);
  EXPECT_EQ(true, paused1);
  EXPECT_EQ(true, paused2);

  eventManager.Emit<events::Pause>(false);
  EXPECT_EQ(false, paused1);
  EXPECT_EQ(false, paused2);

  // Clearing the ConnectionPtr will cause it to no longer fire.
  connection1.reset();

  eventManager.Emit<events::Pause>(true);
  EXPECT_EQ(false, paused1);
  EXPECT_EQ(true, paused2);

  EXPECT_EQ(1u, eventManager.ConnectionCount<events::Pause>());
}

/// Test that we are able to connect arbitrary events and signal them.
TEST(EventManager, NewEvent)
{
  EventManager eventManager;

  using TestEvent = gz::common::EventT<void(std::string, std::string)>;

  std::string val1, val2;
  auto connection = eventManager.Connect<TestEvent>(
      [&](std::string _val1, std::string _val2)
      {
        val1 = _val1;
        val2 = _val2;
      });

  eventManager.Emit<TestEvent>("foo", "bar");
  EXPECT_EQ("foo", val1);
  EXPECT_EQ("bar", val2);

  connection.reset();
  eventManager.Emit<TestEvent>("baz", "bing");
  EXPECT_EQ("foo", val1);
  EXPECT_EQ("bar", val2);
}

TEST(EventManager, Ambiguous)
{
  EventManager eventManager;
  using TestEvent1 = gz::common::EventT<void(void)>;
  using TestEvent2 = gz::common::EventT<void(void)>;

  std::atomic<int> calls = 0;
  auto connection = eventManager.Connect<TestEvent1>([&](){ calls++;});

  // The expectation would be that firing both events would cause only 1 call,
  // but it gets matched twice based on fcn signature.
  eventManager.Emit<TestEvent1>();
  eventManager.Emit<TestEvent2>();
  EXPECT_EQ(2, calls);
}

TEST(EventManager, Disambiguate)
{
  EventManager eventManager;
  using TestEvent1 = gz::common::EventT<void(void), struct TestEvent1Tag>;
  using TestEvent2 = gz::common::EventT<void(void), struct TestEvent2Tag>;

  std::atomic<int> calls = 0;
  auto connection1 = eventManager.Connect<TestEvent1>([&](){ calls++;});

  // Firing both events should only call the callback once.
  eventManager.Emit<TestEvent1>();
  eventManager.Emit<TestEvent2>();
  EXPECT_EQ(1, calls);
}

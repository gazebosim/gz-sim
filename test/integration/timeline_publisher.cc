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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/log_playback_control.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/transport/Node.hh>
#include <gz/transport/WaitHelpers.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test fixture for the TimelinePublisher system
class TimelinePublisherTest : public InternalFixture<::testing::Test>
{
  /// \brief Create a world with a TimelinePublisher.
  /// \param[in] _pluginContent Content of the plugin element.
  /// \return World SDF.
  protected: static std::string World(const std::string &_pluginContent)
  {
    return R"(<?xml version="1.0" ?>
      <sdf version="1.11">
        <world name="timeline">
          <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
          </physics>
          <plugin filename="gz-sim-timeline-publisher-system"
                  name="gz::sim::systems::TimelinePublisher">)" +
          _pluginContent + R"(
          </plugin>
        </world>
      </sdf>)";
  }

  /// \brief Step a server one iteration at a time and check the messages
  /// received on /test/timeline and the iteration that published each one.
  /// Delivery is asynchronous, so after each iteration this waits for the
  /// messages that should have been published by then. A message published
  /// late makes that wait time out.
  /// \param[in] _sdf World SDF.
  /// \param[in] _iterations Number of iterations to run.
  /// \param[in] _expectedData Data of the expected messages, in order.
  /// \param[in] _expectedIteration Iteration expected to publish each
  /// message, give or take one.
  protected: static void ExpectMessages(const std::string &_sdf,
      int _iterations, const std::vector<double> &_expectedData,
      const std::vector<int> &_expectedIteration)
  {
    ASSERT_EQ(_expectedData.size(), _expectedIteration.size());

    std::mutex mutex;
    std::vector<double> received;
    std::function<void(const msgs::Double &)> cb =
        [&](const msgs::Double &_msg)
        {
          std::lock_guard<std::mutex> lock(mutex);
          received.push_back(_msg.data());
        };
    transport::Node node;
    ASSERT_TRUE(node.Subscribe("/test/timeline", cb));

    ServerConfig serverConfig;
    serverConfig.SetSdfString(_sdf);
    Server server(serverConfig);

    // The timeline advertises its topic when the server loads it. Wait until
    // this node discovers it, or the first messages could be missed.
    ASSERT_TRUE(transport::waitForTopic(node, "/test/timeline"));

    // Iteration in which each message was first seen
    std::vector<int> seenIteration;
    for (int i = 0; i < _iterations; ++i)
    {
      server.Run(true, 1, false);

      const auto due = static_cast<std::size_t>(std::count_if(
          _expectedIteration.begin(), _expectedIteration.end(),
          [i](int _iteration) { return _iteration + 1 <= i; }));
      const bool arrived = transport::waitUntil([&]
          {
            std::lock_guard<std::mutex> lock(mutex);
            return received.size() >= due;
          }, 5s);
      ASSERT_TRUE(arrived) << "Missing messages after iteration " << i;

      std::lock_guard<std::mutex> lock(mutex);
      seenIteration.resize(received.size(), i);
    }

    // Messages published in the last iteration weren't waited for yet.
    ASSERT_TRUE(transport::waitUntil([&]
        {
          std::lock_guard<std::mutex> lock(mutex);
          return received.size() >= _expectedData.size();
        }, 5s));

    std::lock_guard<std::mutex> lock(mutex);
    seenIteration.resize(received.size(), _iterations);
    ASSERT_EQ(_expectedData, received);
    for (std::size_t m = 0; m < seenIteration.size(); ++m)
      EXPECT_NEAR(_expectedIteration[m], seenIteration[m], 1) << m;
  }
};

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, LoopingOutputs)
{
  // Keyframes are listed out of order on purpose; outputs that share a time
  // fire in the order they are listed.
  const auto sdf = World(R"(
      <loop>true</loop>
      <period>0.5</period>
      <keyframe time="0.2">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 2</output>
        <output topic="/test/timeline" type="gz.msgs.Double">data: 3</output>
      </keyframe>
      <keyframe time="0">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 1</output>
      </keyframe>)");

  // 1.1 s of simulation: 3 cycles start, at 0, 0.5 and 1.0 s.
  ExpectMessages(sdf, 1100, {1, 2, 3, 1, 2, 3, 1},
      {0, 200, 200, 500, 700, 700, 1000});
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, PlayOnce)
{
  const auto sdf = World(R"(
      <loop>false</loop>
      <keyframe time="0.1">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 1</output>
      </keyframe>
      <keyframe time="0.3">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 2</output>
      </keyframe>)");

  ExpectMessages(sdf, 1000, {1, 2}, {100, 300});
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, LateKeyframeExtendsPeriod)
{
  // The keyframe at 0.3 s is later than the period, so the period becomes
  // 0.3 s and the next cycle starts as soon as that keyframe fires.
  const auto sdf = World(R"(
      <loop>true</loop>
      <period>0.2</period>
      <keyframe time="0">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 1</output>
      </keyframe>
      <keyframe time="0.3">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 2</output>
      </keyframe>)");

  ExpectMessages(sdf, 700, {1, 2, 1, 2, 1}, {0, 300, 300, 600, 600});
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, PeriodShorterThanStep)
{
  // Each 1 ms step covers ten cycles, but only the latest one fires.
  const auto sdf = World(R"(
      <loop>true</loop>
      <period>0.0001</period>
      <keyframe time="0">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 1</output>
      </keyframe>)");

  ExpectMessages(sdf, 5, {1, 1, 1, 1, 1}, {0, 1, 2, 3, 4});
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, ResetRestartsTimeline)
{
  const auto sdf = World(R"(
      <loop>false</loop>
      <keyframe time="0">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 1</output>
      </keyframe>
      <keyframe time="0.1">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 2</output>
      </keyframe>)");

  std::mutex mutex;
  std::vector<double> received;
  std::function<void(const msgs::Double &)> cb =
      [&](const msgs::Double &_msg)
      {
        std::lock_guard<std::mutex> lock(mutex);
        received.push_back(_msg.data());
      };
  transport::Node node;
  ASSERT_TRUE(node.Subscribe("/test/timeline", cb));

  ServerConfig serverConfig;
  serverConfig.SetSdfString(sdf);
  Server server(serverConfig);
  ASSERT_TRUE(transport::waitForTopic(node, "/test/timeline"));

  // Reset halfway through: the keyframe at 0.1 s hasn't fired yet, and the
  // timeline starts over from the keyframe at 0 s.
  server.Run(true, 50, false);
  server.ResetAll();
  server.Run(true, 150, false);
  EXPECT_TRUE(transport::waitUntil([&]
      {
        std::lock_guard<std::mutex> lock(mutex);
        return received.size() >= 3u;
      }, 5s));

  std::lock_guard<std::mutex> lock(mutex);
  const std::vector<double> expected{1, 1, 2};
  EXPECT_EQ(expected, received);
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, PlaysOnceByDefault)
{
  const auto sdf = World(R"(
      <period>0.1</period>
      <keyframe time="0">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 1</output>
      </keyframe>)");

  ExpectMessages(sdf, 300, {1}, {0});
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, SeekKeepsTimelineGoing)
{
  const auto sdf = World(R"(
      <loop>true</loop>
      <period>0.1</period>
      <keyframe time="0">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 1</output>
      </keyframe>)");

  std::atomic<int> received{0};
  std::function<void(const msgs::Double &)> cb =
      [&](const msgs::Double &) { ++received; };
  transport::Node node;
  ASSERT_TRUE(node.Subscribe("/test/timeline", cb));

  ServerConfig serverConfig;
  serverConfig.SetSdfString(sdf);
  Server server(serverConfig);
  ASSERT_TRUE(transport::waitForTopic(node, "/test/timeline"));

  auto seek = [&](int _sec)
  {
    msgs::LogPlaybackControl req;
    req.mutable_seek()->set_sec(_sec);
    msgs::Boolean rep;
    bool result{false};
    ASSERT_TRUE(node.Request("/world/timeline/playback/control", req, 5000,
        rep, result));
    ASSERT_TRUE(result);
  };

  // 1 s of simulation plays 10 cycles.
  server.Run(true, 1000, false);
  ASSERT_TRUE(transport::waitUntil([&] { return received >= 10; }, 5s));

  // After seeking back, the timeline starts over instead of waiting for the
  // simulation time to catch up.
  seek(0);
  server.Run(true, 1000, false);
  ASSERT_TRUE(transport::waitUntil([&] { return received >= 20; }, 5s))
      << received;

  // After seeking an hour ahead, the step that jumps fires once instead of
  // once for every cycle it skipped.
  seek(3600);
  server.Run(true, 2, false);
  ASSERT_TRUE(transport::waitUntil([&] { return received >= 21; }, 5s))
      << received;
  EXPECT_LE(received, 22);
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, InvalidTimesAreIgnored)
{
  // Times that aren't valid durations never fire, and a period that isn't
  // valid falls back to the time of the last keyframe.
  const auto sdf = World(R"(
      <loop>true</loop>
      <period>nan</period>
      <keyframe time="-1">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 2</output>
      </keyframe>
      <keyframe time="nan">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 2</output>
      </keyframe>
      <keyframe time="inf">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 2</output>
      </keyframe>
      <keyframe time="1e300">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 2</output>
      </keyframe>
      <keyframe time="0.1">
        <output topic="/test/timeline" type="gz.msgs.Double">data: 1</output>
      </keyframe>)");

  ExpectMessages(sdf, 250, {1, 1}, {100, 200});
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, InvalidOutputsAreIgnored)
{
  // An unknown message type, a message that doesn't parse, and a topic used
  // with two different types are ignored. The valid output still plays.
  const auto sdf = World(R"(
      <loop>false</loop>
      <keyframe time="0">
        <output topic="/test/bad_type" type="gz.msgs.NotAType">data: 1</output>
        <output topic="/test/bad_msg" type="gz.msgs.Double">bogus: 1</output>
        <output topic="/test/timeline" type="gz.msgs.Double">data: 1</output>
        <output topic="/test/timeline" type="gz.msgs.StringMsg">
          data: 'x'
        </output>
      </keyframe>)");

  ExpectMessages(sdf, 10, {1}, {0});
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, CallsServices)
{
  // Iteration the test is running, recorded with each request
  std::atomic<int> iteration{0};
  std::mutex mutex;
  std::vector<std::pair<int, std::string>> requests;
  std::function<bool(const msgs::StringMsg &, msgs::Boolean &)> srvCb =
      [&](const msgs::StringMsg &_req, msgs::Boolean &_rep)
      {
        std::lock_guard<std::mutex> lock(mutex);
        requests.emplace_back(iteration.load(), _req.data());
        _rep.set_data(true);
        return true;
      };
  transport::Node node;
  ASSERT_TRUE(node.Advertise("/test/timeline_srv", srvCb));

  // The second request is given as the content of the element.
  const auto sdf = World(R"(
      <loop>false</loop>
      <keyframe time="0.05">
        <service name="/test/timeline_srv" reqType="gz.msgs.StringMsg"
                 repType="gz.msgs.Boolean" timeout="2000"
                 reqMsg="data: 'first'"/>
        <service name="/test/timeline_srv" reqType="gz.msgs.StringMsg"
                 repType="gz.msgs.Boolean" timeout="2000">
          data: 'second'
        </service>
      </keyframe>)");

  ServerConfig serverConfig;
  serverConfig.SetSdfString(sdf);
  Server server(serverConfig);

  for (; iteration < 60; ++iteration)
    server.Run(true, 1, false);
  EXPECT_TRUE(transport::waitUntil([&]
      {
        std::lock_guard<std::mutex> lock(mutex);
        return requests.size() >= 2u;
      }));

  // Both calls are made in order, and not before their keyframe at 0.05 s.
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_EQ(2u, requests.size());
  EXPECT_EQ("first", requests[0].second);
  EXPECT_EQ("second", requests[1].second);
  EXPECT_GE(requests[0].first, 49);
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, ResetDropsQueuedServices)
{
  // The service holds the first call until the test releases it, so the
  // second call is still waiting in the queue when the world is reset.
  std::mutex mutex;
  std::condition_variable cv;
  bool release{false};
  std::vector<std::string> requests;
  std::function<bool(const msgs::StringMsg &, msgs::Boolean &)> srvCb =
      [&](const msgs::StringMsg &_req, msgs::Boolean &_rep)
      {
        std::unique_lock<std::mutex> lock(mutex);
        requests.push_back(_req.data());
        cv.notify_all();
        cv.wait(lock, [&] { return release; });
        _rep.set_data(true);
        return true;
      };
  transport::Node node;
  ASSERT_TRUE(node.Advertise("/test/timeline_srv", srvCb));

  const auto sdf = World(R"(
      <loop>false</loop>
      <keyframe time="0">
        <service name="/test/timeline_srv" reqType="gz.msgs.StringMsg"
                 repType="gz.msgs.Boolean" timeout="5000" reqMsg="data: 'a'"/>
        <service name="/test/timeline_srv" reqType="gz.msgs.StringMsg"
                 repType="gz.msgs.Boolean" timeout="5000" reqMsg="data: 'b'"/>
      </keyframe>)");

  ServerConfig serverConfig;
  serverConfig.SetSdfString(sdf);
  Server server(serverConfig);
  server.Run(true, 1, false);
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(cv.wait_for(lock, 5s, [&] { return !requests.empty(); }));
  }

  // The reset drops 'b', and the timeline starts over and queues both calls
  // again. The reset request is read in the first step, the world is reset
  // in the second, and the timeline plays again in the third.
  server.ResetAll();
  server.Run(true, 3, false);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  cv.notify_all();

  // Calls are made in order, so a 'b' kept by mistake would be the second
  // request.
  std::unique_lock<std::mutex> lock(mutex);
  ASSERT_TRUE(cv.wait_for(lock, 5s, [&] { return requests.size() >= 3u; }))
      << ::testing::PrintToString(requests);
  const std::vector<std::string> expected{"a", "a", "b"};
  EXPECT_EQ(expected, std::vector<std::string>(requests.begin(),
      requests.begin() + 3));
}

/////////////////////////////////////////////////
TEST_F(TimelinePublisherTest, SlowServicesDropStaleCalls)
{
  // The service holds the first call until the test releases it, while the
  // timeline queues one call per cycle.
  std::mutex mutex;
  std::condition_variable cv;
  bool release{false};
  int requests{0};
  std::function<bool(const msgs::StringMsg &, msgs::Boolean &)> srvCb =
      [&](const msgs::StringMsg &, msgs::Boolean &_rep)
      {
        std::unique_lock<std::mutex> lock(mutex);
        ++requests;
        cv.notify_all();
        cv.wait(lock, [&] { return release; });
        _rep.set_data(true);
        return true;
      };
  transport::Node node;
  ASSERT_TRUE(node.Advertise("/test/timeline_srv", srvCb));

  const auto sdf = World(R"(
      <loop>true</loop>
      <period>0.01</period>
      <keyframe time="0">
        <service name="/test/timeline_srv" reqType="gz.msgs.StringMsg"
                 repType="gz.msgs.Boolean" timeout="5000" reqMsg="data: 'a'"/>
      </keyframe>)");

  {
    ServerConfig serverConfig;
    serverConfig.SetSdfString(sdf);
    Server server(serverConfig);
    server.Run(true, 1, false);
    {
      std::unique_lock<std::mutex> lock(mutex);
      ASSERT_TRUE(cv.wait_for(lock, 5s, [&] { return requests > 0; }));
    }

    // Cycles 1 to 9 queue a call each while the call from cycle 0 is held.
    // Only the calls from the last two cycles are still waiting.
    server.Run(true, 99, false);
    {
      std::lock_guard<std::mutex> lock(mutex);
      release = true;
    }
    cv.notify_all();

    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(cv.wait_for(lock, 5s, [&] { return requests >= 3; }));
  }

  // The server is gone, so no more calls can be made. A queue that kept
  // every cycle would have made more calls before the server was destroyed.
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(3, requests);
}

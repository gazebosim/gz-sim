/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/pose.pb.h>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

class TriggeredPublisherTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    ignition::common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str());
    // Start server
    ServerConfig serverConfig;
    const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/triggered_publisher.sdf";
    serverConfig.SetSdfFile(sdfFile);

    this->server = std::make_unique<Server>(serverConfig);
    EXPECT_FALSE(server->Running());
    EXPECT_FALSE(*server->Running(0));

    server->SetUpdatePeriod(100us);
    server->Run(true, 1, false);
  }
  public: std::unique_ptr<Server> server;
};

/// \brief Helper function to wait until a predicate is true or a timeout occurs
/// \tparam Pred Predicate function of type bool()
/// \param[in] _timeoutMs Timeout in milliseconds
template <typename Pred>
bool waitUntil(int _timeoutMs, Pred _pred)
{
  using namespace std::chrono;
  auto tStart = steady_clock::now();
  auto sleepDur = milliseconds(std::min(100, _timeoutMs));
  auto waitDuration = milliseconds(_timeoutMs);
  while (duration_cast<milliseconds>(steady_clock::now() - tStart) <
         waitDuration)
  {
    if (_pred())
    {
      return true;
    }
    std::this_thread::sleep_for(sleepDur);
  }
  return false;
}

/////////////////////////////////////////////////
/// Check that empty message types do not need any data to be specified in the
/// configuration
TEST_F(TriggeredPublisherTest, EmptyInputEmptyOutput)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Empty>("/in_0");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_0", msgCb);
  IGN_SLEEP_MS(100ms);

  const std::size_t pubCount{10};
  for (std::size_t i = 0; i < pubCount; ++i)
  {
    EXPECT_TRUE(inputPub.Publish(msgs::Empty()));
  }
  waitUntil(5000, [&]{return pubCount == recvCount;});

  EXPECT_EQ(pubCount, recvCount);
}

/////////////////////////////////////////////////
TEST_F(TriggeredPublisherTest, WrongInputMessageTypeDoesNotMatch)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Boolean>("/in_0");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_0", msgCb);

  const std::size_t pubCount{10};
  for (std::size_t i = 0; i < pubCount; ++i)
  {
    EXPECT_TRUE(inputPub.Publish(msgs::Boolean()));
  }

  waitUntil(5000, [&]{return 0u == recvCount;});
  EXPECT_EQ(0u, recvCount);
}

/////////////////////////////////////////////////
TEST_F(TriggeredPublisherTest, InputMessagesTriggerOutputs)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Empty>("/in_1");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Boolean &)>(
      [&recvCount](const auto &_msg)
      {
        EXPECT_TRUE(_msg.data());
        ++recvCount;
      });
  node.Subscribe("/out_1", msgCb);

  const std::size_t pubCount{10};
  for (std::size_t i = 0; i < pubCount; ++i)
  {
    EXPECT_TRUE(inputPub.Publish(msgs::Empty()));
    IGN_SLEEP_MS(10);
  }

  waitUntil(5000, [&]{return pubCount == recvCount;});
  EXPECT_EQ(pubCount, recvCount);
}

/////////////////////////////////////////////////
TEST_F(TriggeredPublisherTest, MultipleOutputsForOneInput)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Empty>("/in_2");
  std::mutex recvMsgMutex;
  std::vector<bool> recvMsgs0;
  std::vector<bool> recvMsgs1;
  auto cbCreator = [&recvMsgMutex](std::vector<bool> &_msgVector)
  {
    return std::function<void(const msgs::Boolean &)>(
        [&_msgVector, &recvMsgMutex](const msgs::Boolean &_msg)
        {
          std::lock_guard<std::mutex> lock(recvMsgMutex);
          _msgVector.push_back(_msg.data());
        });
  };

  auto msgCb0 = cbCreator(recvMsgs0);
  auto msgCb1 = cbCreator(recvMsgs1);
  node.Subscribe("/out_2_0", msgCb0);
  node.Subscribe("/out_2_1", msgCb1);

  const int pubCount{10};
  for (int i = 0; i < pubCount; ++i)
  {
    EXPECT_TRUE(inputPub.Publish(msgs::Empty()));
    IGN_SLEEP_MS(10);
  }

  waitUntil(5000, [&]
            {
              std::lock_guard<std::mutex> lock(recvMsgMutex);
              return static_cast<std::size_t>(pubCount) == recvMsgs0.size() &&
                     static_cast<std::size_t>(pubCount) == recvMsgs1.size();
            });

  EXPECT_EQ(static_cast<std::size_t>(pubCount), recvMsgs0.size());
  EXPECT_EQ(static_cast<std::size_t>(pubCount), recvMsgs1.size());

  // The plugin has two outputs. We expect 10 messages in each output topic
  EXPECT_EQ(pubCount, std::count(recvMsgs0.begin(), recvMsgs0.end(), false));
  EXPECT_EQ(pubCount, std::count(recvMsgs1.begin(), recvMsgs1.end(), true));
}

/////////////////////////////////////////////////
TEST_F(TriggeredPublisherTest, ExactMatchBooleanInputs)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Boolean>("/in_3");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_3", msgCb);

  const std::size_t pubCount{10};
  const std::size_t trueCount{5};
  for (std::size_t i = 0; i < pubCount; ++i)
  {
    if (i < trueCount)
    {
      EXPECT_TRUE(inputPub.Publish(msgs::Convert(true)));
    }
    else
    {
      EXPECT_TRUE(inputPub.Publish(msgs::Convert(false)));
    }
    IGN_SLEEP_MS(10);
  }

  // The matcher filters out false messages and the inputs consist of 5 true and
  // 5 false messages, so we expect 5 output messages
  EXPECT_EQ(trueCount, recvCount);
}

/////////////////////////////////////////////////
TEST_F(TriggeredPublisherTest, MatchersWithLogicTypeAttribute)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Int32>("/in_4");
  std::atomic<std::size_t> recvCount[2]{0, 0};

  auto cbCreator = [](std::atomic<std::size_t> &_counter)
  {
    return std::function<void(const msgs::Empty &)>(
        [&_counter](const msgs::Empty &)
        {
          ++_counter;
        });
  };

  auto msgCb0 = cbCreator(recvCount[0]);
  auto msgCb1 = cbCreator(recvCount[1]);
  node.Subscribe("/out_4_0", msgCb0);
  node.Subscribe("/out_4_1", msgCb1);

  const int pubCount{10};
  for (int i = 0; i < pubCount; ++i)
  {
    EXPECT_TRUE(inputPub.Publish(
        msgs::Convert(static_cast<int32_t>(i - pubCount / 2))));
    IGN_SLEEP_MS(10);
  }
  // The negative matcher filters out 0 so we expect 9 output messages from the
  // 10 inputs
  EXPECT_EQ(9u, recvCount[0]);

  // The positive matcher only accepts the input value 0
  EXPECT_EQ(1u, recvCount[1]);
}

/////////////////////////////////////////////////
TEST_F(TriggeredPublisherTest, MultipleMatchersAreAnded)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Int32>("/in_5");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_5", msgCb);

  const int pubCount{10};
  for (int i = 0; i < pubCount; ++i)
  {
    EXPECT_TRUE(inputPub.Publish(
        msgs::Convert(static_cast<int32_t>(i - pubCount / 2))));
    IGN_SLEEP_MS(10);
  }
  // The matcher filters out negative numbers and the input is [-5,4], so we
  // expect 5 output messages.
  EXPECT_EQ(5u, recvCount);
}

/////////////////////////////////////////////////
TEST_F(TriggeredPublisherTest, FieldMatchers)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Vector2d>("/in_6");
  std::atomic<std::size_t> recvCount[2]{0, 0};

  auto cbCreator = [](std::atomic<std::size_t> &_counter)
  {
    return std::function<void(const msgs::Empty &)>(
        [&_counter](const msgs::Empty &)
        {
          ++_counter;
        });
  };

  auto msgCb0 = cbCreator(recvCount[0]);
  auto msgCb1 = cbCreator(recvCount[1]);
  node.Subscribe("/out_6_0", msgCb0);
  node.Subscribe("/out_6_1", msgCb1);

  const int pubCount{10};
  msgs::Vector2d msg;
  msg.set_x(1.0);
  for (int i = 0; i < pubCount; ++i)
  {
    msg.set_y(static_cast<double>(i));
    EXPECT_TRUE(inputPub.Publish(msg));
    IGN_SLEEP_MS(10);
  }

  // The first plugin matches x==1 and y==2 which only once out of the 10 inputs
  EXPECT_EQ(1u, recvCount[0]);
  // The second plugin matches x==1 and y!=2 which occurs 9 out of the 10 inputs
  EXPECT_EQ(9u, recvCount[1]);
}

/////////////////////////////////////////////////
/// Tests that if the specified field is a repeated field, a partial match is
/// used when comparing against the input.
TEST_F(TriggeredPublisherTest, FieldMatchersWithRepeatedFieldsUsePartialMatches)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Pose>("/in_7");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_7", msgCb);

  const int pubCount{10};
  for (int i = 0; i < pubCount; ++i)
  {
    msgs::Pose poseMsg;
    auto *frame = poseMsg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(std::string("frame") + std::to_string(i));

    auto *time = poseMsg.mutable_header()->mutable_stamp();
    time->set_sec(10);

    auto *other = poseMsg.mutable_header()->add_data();
    other->set_key("other_key");
    other->add_value("other_value");
    EXPECT_TRUE(inputPub.Publish(poseMsg));
    IGN_SLEEP_MS(10);
  }

  // The matcher filters out frame ids that are not frame0, so we expect 1
  // output. Even though the data field contains other key-value pairs, since
  // repeated fields use partial matching, the matcher will match one of the
  // inputs.
  EXPECT_EQ(1u, recvCount);
}

TEST_F(TriggeredPublisherTest, WrongInputWhenRepeatedSubFieldExpected)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Empty>("/in_7");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_7", msgCb);
  IGN_SLEEP_MS(10);

  const int pubCount{10};
  msgs::Empty msg;
  for (int i = 0; i < pubCount; ++i)
  {
    EXPECT_TRUE(inputPub.Publish(msg));
    IGN_SLEEP_MS(10);
  }

  EXPECT_EQ(0u, recvCount);
}

/////////////////////////////////////////////////
/// Tests that field matchers can be used to do a full match with repeated
/// fields by specifying the containing field of the repeated field in the
/// "field" attribute and setting the desired values of the repeated field in
/// the value of the <match> tag.
TEST_F(TriggeredPublisherTest,
       FieldMatchersWithRepeatedFieldsInValueUseFullMatches)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Pose>("/in_8");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_8", msgCb);
  IGN_SLEEP_MS(10);

  const int pubCount{10};
  for (int i = 0; i < pubCount; ++i)
  {
    msgs::Pose poseMsg;
    auto *frame = poseMsg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value("frame0");

    if (i < 5)
    {
      auto *other = poseMsg.mutable_header()->add_data();
      other->set_key("other_key");
      other->add_value("other_value");
    }
    EXPECT_TRUE(inputPub.Publish(poseMsg));
    IGN_SLEEP_MS(10);
  }

  // Since the field specified in "field" is not a repeated field, a full match
  // is required to trigger an output. Only the first 5 input messages have the
  // second "data" entry, so the expected recvCount is 5.
  EXPECT_EQ(5u, recvCount);
}

/////////////////////////////////////////////////
/// Tests that full matchers can be used with repeated fields by specifying the
/// desired values of the repeated field in the value of the <match> tag. The
/// message created from the value of <match> must be a full match of the input.
TEST_F(TriggeredPublisherTest,
       FullMatchersWithRepeatedFieldsInValueUseFullMatches)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Int32_V>("/in_9");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_9", msgCb);
    IGN_SLEEP_MS(10);

  const int pubCount{10};
  msgs::Int32_V msg;
  for (int i = 0; i < pubCount; ++i)
  {
    msg.add_data(i);
    EXPECT_TRUE(inputPub.Publish(msg));
    IGN_SLEEP_MS(10);
  }

  // The input contains an increasing sets of sequences, {0}, {0,1}, {0,1,2}...
  // The matcher only matches {0,1}
  EXPECT_EQ(1u, recvCount);
}

TEST_F(TriggeredPublisherTest, FullMatchersAcceptToleranceParam)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Float>("/in_10");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_10", msgCb);
  IGN_SLEEP_MS(10);

  const int pubCount{10};
  msgs::Float msg;
  for (int i = 0; i < pubCount; ++i)
  {
    msg.set_data(static_cast<float>(i)* 0.1);
    EXPECT_TRUE(inputPub.Publish(msg));
    IGN_SLEEP_MS(10);
  }

  // The input contains the sequence {0, 0.1, 0.2, ...}, the matcher is set to
  // match 0.5 with a tolerance of 0.15, so it should match {0.4, 0.5, 0.6}
  EXPECT_EQ(3u, recvCount);
}

TEST_F(TriggeredPublisherTest, FieldMatchersAcceptToleranceParam)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Pose>("/in_11");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_11", msgCb);
  IGN_SLEEP_MS(10);

  const int pubCount{10};
  msgs::Pose msg;
  for (int i = 0; i < pubCount; ++i)
  {
    msg.mutable_position()->set_x(0.1);
    msg.mutable_position()->set_z(static_cast<float>(i)* 0.1);
    EXPECT_TRUE(inputPub.Publish(msg));
    IGN_SLEEP_MS(10);
  }

  // The input contains the sequence {0, 0.1, 0.2, ...} in position.z, the
  // matcher is set to match 0.5 with a tolerance of 0.15, so it should match
  // {0.4, 0.5, 0.6}
  EXPECT_EQ(3u, recvCount);
}

TEST_F(TriggeredPublisherTest, SubfieldsOfRepeatedFieldsNotSupported)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Header>("/in_12");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_12", msgCb);
  IGN_SLEEP_MS(10);

  const int pubCount{10};
  for (int i = 0; i < pubCount; ++i)
  {
    msgs::Header msg;
    auto *data = msg.add_data();
    data->set_key("key1");
    data->add_value("value1");

    EXPECT_TRUE(inputPub.Publish(msg));
    IGN_SLEEP_MS(10);
  }

  // Subfields of repeated fiealds are not supported, so no output should be
  // triggered.
  EXPECT_EQ(0u, recvCount);
}

TEST_F(TriggeredPublisherTest, TriggerDelay)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Empty>("/in_13");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_13", msgCb);
  IGN_SLEEP_MS(100ms);

  const std::size_t pubCount{10};
  for (std::size_t i = 0; i < pubCount; ++i)
  {
    EXPECT_TRUE(inputPub.Publish(msgs::Empty()));
  }
  waitUntil(1000, [&]{return pubCount == recvCount;});

  // Delay has been specified, but simulation is not running. No messages
  // should have been received.
  EXPECT_EQ(0u, recvCount);

  // The simulation delay is 1000ms, which is equal to 1000 steps. Run
  // for 999 steps, and the count should still be zero. Take one additional
  // step and all the messages should arrive.
  this->server->Run(true, 999, false);
  waitUntil(1000, [&]{return pubCount == recvCount;});
  EXPECT_EQ(0u, recvCount);
  this->server->Run(true, 1, false);
  waitUntil(1000, [&]{return pubCount == recvCount;});
  EXPECT_EQ(pubCount, recvCount);
}

TEST_F(TriggeredPublisherTest, WrongInputWhenRepeatedFieldExpected)
{
  transport::Node node;
  auto inputPub = node.Advertise<msgs::Int32>("/invalid_topic");
  std::atomic<std::size_t> recvCount{0};
  auto msgCb = std::function<void(const msgs::Empty &)>(
      [&recvCount](const auto &)
      {
        ++recvCount;
      });
  node.Subscribe("/out_9", msgCb);
  IGN_SLEEP_MS(10);

  const int pubCount{10};
  msgs::Int32 msg;
  for (int i = 0; i < pubCount; ++i)
  {
    msg.set_data(i);
    EXPECT_TRUE(inputPub.Publish(msg));
    IGN_SLEEP_MS(10);
  }

  EXPECT_EQ(0u, recvCount);
}

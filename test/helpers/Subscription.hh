/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */

#ifndef GZ_SIM_TEST_HELPERS_SUBSCRIPTION_HH
#define GZ_SIM_TEST_HELPERS_SUBSCRIPTION_HH

#include <deque>
#include <mutex>
#include <string>

#include <gz/transport/Node.hh>

using namespace gz;

/// A stateful wrapper for Gazebo Transport subscriptions.
template<typename MessageT>
class Subscription
{
  /// Default constructor.
  public: Subscription() = default;

  /// Subscribe to a topic.
  /// \param[in] _node Node to use to subscribe.
  /// \param[in] _topicName Name of the topic to subscribe.
  /// \param[in] _messageHistoryDepth Maximum size for
  /// message history buffer. Buffer grows indefinitely
  /// by default.
  public: void Subscribe(
      transport::Node &_node,
      const std::string &_topicName,
      size_t _messageHistoryDepth = 0u)
  {
    if (this->subscribed)
    {
      throw std::logic_error("Subscription already subscribed");
    }
    std::lock_guard<std::mutex> lock(this->mutex);
    const auto callback = &Subscription::OnMessage;
    if (!_node.Subscribe(_topicName, callback, this))
    {
      throw std::runtime_error("Cannot subscribe to " + _topicName);
    }
    this->messageHistoryDepth = _messageHistoryDepth;
    this->subscribed = true;
  }

  /// Wait for `_count` messages to arrive.
  /// \param[in] _count Number of messages to wait for.
  /// \param[in] _timeout Maximum time to wait.
  /// \return true once `_count` messages have been
  /// received since first subscription, false otherwise.
  public: bool WaitForMessages(size_t _count, std::chrono::nanoseconds _timeout)
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    auto predicate = [&]() { return this->messageCount >= _count; };
    return this->messageArrival.wait_for(lock, _timeout, predicate);
  }

  /// Read all messages received so far.
  /// \note This is a destructive operation.
  /// \return entire message history, up to its
  /// specified depth. May be empty.
  public: std::deque<MessageT> ReadMessages()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return std::move(this->messageHistory);
  }

  /// Current number of messages stored.
  /// \return number of messages stored in the
  /// messageHistory container.
  public: int MessageHistorySize()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->messageHistory.size();
  }

  /// Read the message according to the index.
  /// \return message in the messageHistory container
  /// based on its index.
  public: MessageT GetMessageByIndex(int _index)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->messageHistory[_index];
  }

  /// Reset the messageHistory container by clearing
  /// existing messages.
  public: void ResetMessageHistory()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->messageHistory.clear();
  }

  /// Read last message received.
  /// \note This is a destructive operation.
  /// \throws std::runtime_error if there is
  /// no message to be read.
  public: MessageT ReadLastMessage()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->messageHistory.empty())
    {
      throw std::runtime_error("No messages to read");
    }
    MessageT message = std::move(this->messageHistory.back());
    this->messageHistory.pop_back();
    return message;
  }

  private: void OnMessage(const MessageT &_message)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->messageHistory.push_back(_message);
      if (this->messageHistoryDepth > 0u)
      {
        if (this->messageHistory.size() > this->messageHistoryDepth)
        {
          this->messageHistory.pop_front();
        }
      }
      ++this->messageCount;
    }
    this->messageArrival.notify_all();
  }

  private: bool subscribed{false};

  private: size_t messageCount{0u};

  private: size_t messageHistoryDepth{0u};

  private: std::deque<MessageT> messageHistory;

  private: std::condition_variable messageArrival;

  private: std::mutex mutex;
};

#endif  // GZ_SIM_TEST_HELPERS_SUBSCRIPTION_HH

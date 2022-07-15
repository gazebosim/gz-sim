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
#ifndef GZ_SIM_TEST_HELPERS_MESSAGE_RECEIVER_HH_
#define GZ_SIM_TEST_HELPERS_MESSAGE_RECEIVER_HH_

#include <atomic>
#include <list>
#include <mutex>
#include <string>

#include <gz/transport/Node.hh>
#include <gz/common/Console.hh>

namespace gz::sim::test 
{
template <typename T>
struct MessageReceiver
{
  std::string topic;
  std::mutex msgMutex;
  std::list<T> msgs;
  gz::transport::Node node;

  bool Start(const std::string &_topic) {
    this->topic = _topic;
    return this->node.Subscribe(_topic, &MessageReceiver<T>::Callback, this);
  }

  void Stop() {
    this->node.Unsubscribe(this->_topic);
  }

  bool Spin(
      const std::chrono::milliseconds &_rate = std::chrono::milliseconds(100),  
      const std::chrono::milliseconds &_timeout = std::chrono::milliseconds(5000))
  {
    auto t1 = std::chrono::system_clock::now();
    while (!this->Received()) 
    {
      std::this_thread::sleep_for(_rate);

      auto t2 = std::chrono::system_clock::now();
      if ((t2 - t1) >= _timeout)
      {
        ignwarn << "MessageReceiver::Spin timed out!" << std::endl;
        return false;
      }
    }
    return true;
  }

  void Callback(const T &_msg) {
    std::lock_guard<std::mutex> lk(this->msgMutex);
    this->msgs.push_back(_msg);
  }

  void Clear() {
    std::lock_guard<std::mutex> lk(this->msgMutex);
    return this->msgs.clear();
  }

  size_t Count() {
    std::lock_guard<std::mutex> lk(this->msgMutex);
    return this->msgs.size();
  }

  bool Received() {
    return this->Count() != 0;
  }

   T Last() {
    std::lock_guard<std::mutex> lk(this->msgMutex);
    return this->msgs.back();
  }
};
}  // namespace gz::sim:test

#endif  // GZ_SIM_TEST_HELPERS_MESSAGE_RECEIVER_HH_


/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/Node.hh>

namespace gz::sim::test
{
/// \brief Wait until a service becomes available.
/// See https://github.com/gazebosim/gz-transport/issues/468 for why this might
/// be necessary before make a service request.
/// \param[in] _node Transport Node to use
/// \param[in] _service Name of service to wait for
/// \param[in] _timeoutS Time out in seconds
bool waitForService(const transport::Node &_node, const std::string &_service,
                    int _timeoutS = 5)
{
  int curSleep = 0;
  while (curSleep < _timeoutS)
  {
    std::vector<transport::ServicePublisher> publishers;
    if (_node.ServiceInfo(_service, publishers))
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ++curSleep;
  }
  return false;
}
}  // namespace gz::sim::test

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

#ifndef GZ_SIM_TEST_HELPERS_UTIL_HH_
#define GZ_SIM_TEST_HELPERS_UTIL_HH_

#include <chrono>
#include <cstdint>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/sim/Server.hh>
#include <gz/transport/Node.hh>

namespace gz::sim::test
{
namespace detail
{
/////////////////////////////////////////////////
/// \brief Run one unpaused blocking server step.
/// \param[in] _server Server to step.
inline void RunOneUnpausedStep(gz::sim::Server &_server)
{
  _server.Run(true, 1, false);
}
}  // namespace detail

/////////////////////////////////////////////////
/// \brief Step the server until a predicate becomes true.
/// \param[in] _server Server to step.
/// \param[in] _maxSteps Maximum number of single steps.
/// \param[in] _predicate Predicate to evaluate before and after each step.
/// \return True when the predicate becomes true.
template <typename Predicate>
bool StepUntil(
    gz::sim::Server &_server, uint64_t _maxSteps, Predicate _predicate)
{
  if (_predicate())
    return true;

  for (uint64_t i = 0; i < _maxSteps; ++i)
  {
    detail::RunOneUnpausedStep(_server);
    if (_predicate())
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
/// \brief Wait without advancing simulation until a predicate becomes true.
/// \param[in] _timeout Maximum wall-clock wait.
/// \param[in] _predicate Predicate to evaluate.
/// \return True when the predicate becomes true.
template <typename Predicate>
bool WaitUntil(
    const std::chrono::steady_clock::duration &_timeout,
    Predicate _predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + _timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (_predicate())
      return true;

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return _predicate();
}

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

/// \brief Read a given file path and return its contents
/// \param[in] _filePath The path to the file
/// \return An optional string with the contents of the file. nullopt if there
/// was an error reading the file
std::optional<std::string> readFileContents(const std::string &_filePath)
{
  std::ifstream infile(_filePath);
  if (!infile.good())
  {
    return std::nullopt;
  }
  return std::string(std::istreambuf_iterator<char>(infile),
                     std::istreambuf_iterator<char>());
}
}  // namespace gz::sim::test

#endif  // GZ_SIM_TEST_HELPERS_UTIL_HH_

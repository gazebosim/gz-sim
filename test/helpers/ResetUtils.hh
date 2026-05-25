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

#ifndef GZ_SIM_TEST_HELPERS_RESET_UTILS_HH_
#define GZ_SIM_TEST_HELPERS_RESET_UTILS_HH_

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <string>

#include <gz/sim/Server.hh>
#include <gz/transport/Node.hh>

namespace gz::sim::test::reset
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
}

/////////////////////////////////////////////////
/// \brief Request a world reset over transport.
/// \param[in] _worldName Name of the world to reset.
inline void RequestWorldReset(const std::string &_worldName)
{
  gz::msgs::WorldControl req;
  gz::msgs::Boolean rep;
  req.mutable_reset()->set_all(true);
  gz::transport::Node node;

  constexpr unsigned int timeout = 1000u;
  bool result = false;
  const bool executed = node.Request(
      "/world/" + _worldName + "/control", req, timeout, rep, result);

  ASSERT_TRUE(executed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());
}

/////////////////////////////////////////////////
/// \brief Run one step so the server consumes the reset request.
/// \param[in] _server Server to step.
inline void ConsumeResetRequest(gz::sim::Server &_server)
{
  detail::RunOneUnpausedStep(_server);
}

/////////////////////////////////////////////////
/// \brief Run one step so the simulation runner applies the reset.
/// \param[in] _server Server to step.
inline void ApplyWorldReset(gz::sim::Server &_server)
{
  detail::RunOneUnpausedStep(_server);
}

/////////////////////////////////////////////////
/// \brief Request and apply a world reset using the standard two-step flow.
/// \param[in] _server Server to step.
/// \param[in] _worldName Name of the world to reset.
inline void RequestAndApplyWorldReset(
    gz::sim::Server &_server, const std::string &_worldName)
{
  RequestWorldReset(_worldName);
  ConsumeResetRequest(_server);
  ApplyWorldReset(_server);
}

}

#endif  // GZ_SIM_TEST_HELPERS_RESET_UTILS_HH_

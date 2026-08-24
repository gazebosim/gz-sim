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
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <cmath>
#include <string>

#include <gz/sim/Server.hh>
#include <gz/transport/Node.hh>

#include "Subscription.hh"
#include "Util.hh"

namespace gz::sim::test::reset
{
/////////////////////////////////////////////////
/// \brief Thresholds used by odometry reset checks.
struct OdomResetExpectations
{
  /// \brief Minimum pre-reset pose X and linear X values used to prove that
  /// the system entered a dirty state.
  public: double dirtyStateMin{0.05};

  /// \brief Tolerance for the first post-reset odometry sample.
  public: double resetTolerance{1e-2};

  /// \brief Tolerance for the settled post-reset odometry window.
  public: double settledTolerance{0.05};
};

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

/////////////////////////////////////////////////
/// \brief Drive a system into a dirty odometry state, reset it, and verify that
/// the odometry state is cleared while fresh commands still work.
/// \param[in] _server Server under test.
/// \param[in] _publisher Command publisher.
/// \param[in] _preResetOdom Pre-reset odometry subscription.
/// \param[in] _odomTopic Odometry topic to subscribe to after reset.
/// \param[in] _command Command message to publish before and after reset.
inline void ExpectOdomResetClearsState(
    gz::sim::Server &_server,
    gz::transport::Node::Publisher &_publisher,
    Subscription<gz::msgs::Odometry> &_preResetOdom,
    const std::string &_odomTopic,
    const gz::transport::ProtoMsg &_command,
    const OdomResetExpectations &_expectations = OdomResetExpectations())
{
  _server.Run(true, 1000, false);
  _preResetOdom.Clear();

  _publisher.Publish(_command);
  ASSERT_TRUE(gz::sim::test::StepUntil(_server, 2000,
      [&]
      {
        return _preResetOdom.Count() > 0u &&
            _preResetOdom.Last().pose().position().x() >
                _expectations.dirtyStateMin &&
            _preResetOdom.Last().twist().linear().x() >
                _expectations.dirtyStateMin;
      }));

  _server.ResetAll();

  // Use a fresh subscription so delayed pre-reset messages cannot satisfy the
  // post-reset assertions.
  gz::transport::Node postResetNode;
  Subscription<gz::msgs::Odometry> postResetOdom;
  postResetOdom.Subscribe(postResetNode, _odomTopic, 10u);

  ASSERT_TRUE(gz::sim::test::StepUntil(_server, 1000,
      [&]
      {
        if (postResetOdom.Count() == 0u)
          return false;

        const auto &odom = postResetOdom.Last();
        return std::abs(odom.pose().position().x()) <
                _expectations.resetTolerance &&
            std::abs(odom.pose().position().y()) <
                _expectations.resetTolerance &&
            std::abs(odom.twist().linear().x()) <
                _expectations.resetTolerance &&
            std::abs(odom.twist().angular().z()) <
                _expectations.resetTolerance;
      }));

  const auto postReset = postResetOdom.Last();
  EXPECT_NEAR(0.0, postReset.pose().position().x(),
      _expectations.resetTolerance);
  EXPECT_NEAR(0.0, postReset.pose().position().y(),
      _expectations.resetTolerance);
  EXPECT_NEAR(0.0, postReset.twist().linear().x(),
      _expectations.resetTolerance);
  EXPECT_NEAR(0.0, postReset.twist().angular().z(),
      _expectations.resetTolerance);

  _server.Run(true, 500, false);
  const auto settled = postResetOdom.Last();
  EXPECT_NEAR(0.0, settled.pose().position().x(),
      _expectations.settledTolerance);
  EXPECT_NEAR(0.0, settled.pose().position().y(),
      _expectations.settledTolerance);
  EXPECT_NEAR(0.0, settled.twist().linear().x(),
      _expectations.settledTolerance);
  EXPECT_NEAR(0.0, settled.twist().angular().z(),
      _expectations.settledTolerance);

  postResetOdom.Clear();
  _publisher.Publish(_command);
  ASSERT_TRUE(gz::sim::test::StepUntil(_server, 2000,
      [&]
      {
        return postResetOdom.Count() > 0u &&
            postResetOdom.Last().twist().linear().x() >
                _expectations.dirtyStateMin;
      }));
}

}

#endif  // GZ_SIM_TEST_HELPERS_RESET_UTILS_HH_

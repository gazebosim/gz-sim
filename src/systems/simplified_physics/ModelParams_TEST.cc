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

#include <string>
#include <vector>

#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Plugin.hh>

#include "ModelParams.hh"

using namespace gz;
using namespace sim::systems::simplified_physics;

namespace
{
/// \brief Parse the parameters from the text inside a plugin element.
ModelParams Parse(const std::string &_contents, bool _warn = false)
{
  const std::string text = R"(<sdf version="1.9"><model name="m">
    <link name="l"/>
    <plugin filename="gz-sim-simplified-physics-system"
            name="gz::sim::systems::SimplifiedPhysicsModel">)" +
      _contents + R"(</plugin></model></sdf>)";

  sdf::Root root;
  const auto errors = root.LoadSdfString(text);
  EXPECT_TRUE(errors.empty()) << errors;
  const auto *model = root.Model();
  EXPECT_NE(nullptr, model);
  if (!model || model->Plugins().empty())
    return {};
  return ParseModelParams(model->Plugins()[0].Contents(), "m", _warn);
}
}

/////////////////////////////////////////////////
TEST(ModelParams, Defaults)
{
  const auto params = Parse("<mode>planar_kinematic</mode>");
  EXPECT_TRUE(params.wheels.empty());
  EXPECT_EQ(0.0, params.odometryNoise);
  EXPECT_FALSE(params.animateWheels);
  EXPECT_EQ(PlanarResponse::Slide, params.collisionResponse);
}

/////////////////////////////////////////////////
TEST(ModelParams, CollisionResponse)
{
  EXPECT_EQ(PlanarResponse::Stop,
      Parse("<collision_response>stop</collision_response>")
          .collisionResponse);
  EXPECT_EQ(PlanarResponse::Slide,
      Parse("<collision_response>slide</collision_response>")
          .collisionResponse);

  // A value that is not known leaves the default.
  for (const std::string bad : {"bounce", "Stop", "", "1"})
  {
    EXPECT_EQ(PlanarResponse::Slide,
        Parse("<collision_response>" + bad + "</collision_response>")
            .collisionResponse) << bad;
  }
}

/////////////////////////////////////////////////
TEST(ModelParams, ReadsAllParameters)
{
  const auto params = Parse(R"(
    <mode>planar_kinematic</mode>
    <odometry_noise>0.015</odometry_noise>
    <animate_wheels>true</animate_wheels>
    <wheel radius="0.3">left_wheel_joint</wheel>
    <wheel>right_wheel_joint</wheel>)");

  EXPECT_DOUBLE_EQ(0.015, params.odometryNoise);
  EXPECT_TRUE(params.animateWheels);
  ASSERT_EQ(2u, params.wheels.size());
  EXPECT_EQ("left_wheel_joint", params.wheels[0].joint);
  ASSERT_TRUE(params.wheels[0].radius.has_value());
  EXPECT_DOUBLE_EQ(0.3, *params.wheels[0].radius);
  EXPECT_EQ("right_wheel_joint", params.wheels[1].joint);
  EXPECT_FALSE(params.wheels[1].radius.has_value());
}

/////////////////////////////////////////////////
TEST(ModelParams, SkipsValuesThatCannotBeUsed)
{
  for (const std::string bad : {"-0.1", "abc", "nan", "1e400", ""})
  {
    const auto params = Parse(
        "<odometry_noise>" + bad + "</odometry_noise>");
    EXPECT_EQ(0.0, params.odometryNoise) << bad;
  }

  const auto params = Parse(R"(
    <wheel radius="-1">left</wheel>
    <wheel radius="x">right</wheel>
    <wheel></wheel>)");
  ASSERT_EQ(2u, params.wheels.size());
  EXPECT_FALSE(params.wheels[0].radius.has_value());
  EXPECT_FALSE(params.wheels[1].radius.has_value());
}

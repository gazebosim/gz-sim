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

#include <gtest/gtest.h>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>

#include <chrono>
#include <string>
#include <vector>

#include <sdf/Element.hh>
#include <sdf/Root.hh>
#include <gz/common/Image.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/SystemPluginPtr.hh"

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include "plugins/MockSystem.hh"
#include "helpers/EnvTestFixture.hh"
#include "helpers/Util.hh"
#include "helpers/ResetUtils.hh"
#include "helpers/Subscription.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;
namespace reset = gz::sim::test::reset;

constexpr double kStartingAltitude = 100.0f;
constexpr double kStartingPressure = 100129.46;

constexpr double kEndingAltitude =  80.390198f;
constexpr double kEndingPressure = 100362.99;

//////////////////////////////////////////////////
class ResetFixture: public InternalFixture<InternalFixture<::testing::Test>>
{
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    sdf::Plugin sdfPlugin;
    sdfPlugin.SetName("gz::sim::MockSystem");
    sdfPlugin.SetFilename("libMockSystem.so");
    auto plugin = sm.LoadPlugin(sdfPlugin);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();
    this->mockSystem = static_cast<sim::MockSystem *>(
        systemPtr->QueryInterface<sim::System>());
  }

  public: gz::sim::SystemPluginPtr systemPtr;
  public: sim::MockSystem *mockSystem;

  private: sim::SystemLoader sm;
};

common::Image toImage(const msgs::Image &_msg)
{
  common::Image image;
  common::Image::PixelFormatType pixelFormat =
      common::Image::ConvertPixelFormat(
        msgs::ConvertPixelFormatType(
        _msg.pixel_format_type()));
  image.SetFromData(
          reinterpret_cast<const unsigned char *>(_msg.data().c_str()),
          _msg.width(), _msg.height(), pixelFormat);
  return image;
}

/////////////////////////////////////////////////
/// This test checks that that air-pressure and camera sensor systems
/// handle Reset events
TEST_F(ResetFixture, GZ_UTILS_TEST_DISABLED_ON_MAC(HandleReset))
{
  // This test fails on Github Actions. Skip it for now.
  // Note: The GITHUB_ACTIONS environment variable is automatically set when
  // running on Github Actions.
  std::string githubAction;
  if (common::env("GITHUB_ACTIONS", githubAction))
  {
    GTEST_SKIP();
  }

  gz::sim::ServerConfig serverConfig;

  const std::string sdfFile = common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "reset_sensors.sdf");

  serverConfig.SetSdfFile(sdfFile);

  sdf::Root root;
  root.Load(sdfFile);
  sim::Server server(serverConfig);

  const std::string sensorName = "air_pressure_sensor";
  auto topic = "world/default/model/box/link/link/"
      "sensor/air_pressure_sensor/air_pressure";

  transport::Node node;
  Subscription<msgs::FluidPressure> pressureReceiver;
  Subscription<msgs::Image> imageReceiver;

  // A pointer to the ecm. This will be valid once we run the mock system
  sim::EntityComponentManager *ecm = nullptr;

  this->mockSystem->configureCallback =
    [&ecm](const Entity &,
           const std::shared_ptr<const sdf::Element> &,
           EntityComponentManager &_ecm,
           EventManager &)
    {
      ecm = &_ecm;
    };

  // Validate update info in the reset
  this->mockSystem->resetCallback =
    [](const sim::UpdateInfo &_info,
       sim::EntityComponentManager &)
    {
      EXPECT_EQ(0u, _info.iterations);
      EXPECT_EQ(std::chrono::steady_clock::duration{0}, _info.simTime);
    };

  server.AddSystem(this->systemPtr);

  ASSERT_NE(nullptr, ecm);
  auto entity = ecm->EntityByComponents(components::Name("box"));
  ASSERT_NE(kNullEntity, entity);
  auto poseComp = ecm->Component<components::Pose>(entity);
  ASSERT_NE(nullptr, poseComp);

  // Verify initial conditions of the world
  {
    EXPECT_FLOAT_EQ(kStartingAltitude, poseComp->Data().Z());
    EXPECT_EQ(1u, this->mockSystem->configureCallCount);
    EXPECT_EQ(0u, this->mockSystem->resetCallCount);
    EXPECT_EQ(0u, this->mockSystem->preUpdateCallCount);
    EXPECT_EQ(0u, this->mockSystem->updateCallCount);
    EXPECT_EQ(0u, this->mockSystem->postUpdateCallCount);
  }

  auto current = 1u;
  auto target = 2000u;
  const auto remainingSteps = [&server](uint64_t _target)
  {
    const auto iteration = server.IterationCount().value_or(0u);
    return iteration < _target ? _target - iteration : 0u;
  };

  // Run until a sensor measurement
  pressureReceiver.Subscribe(node, topic, 1u);
  imageReceiver.Subscribe(node, "camera", 1u);
  ASSERT_TRUE(gz::sim::test::StepUntil(server, remainingSteps(target),
      [&pressureReceiver]()
      {
        return pressureReceiver.Count() > 0u;
      }));

  EXPECT_GE(server.IterationCount().value(), current);
  EXPECT_FLOAT_EQ(kStartingPressure, pressureReceiver.Last().pressure());

  ASSERT_TRUE(gz::sim::test::StepUntil(server, remainingSteps(target),
      [&imageReceiver]()
      {
        return imageReceiver.Count() > 0u;
      }));

  // Mostly green box
  {
    auto image = toImage(imageReceiver.Last());
    auto centerPix = image.Pixel(image.Width()/2, image.Height()/2);
    EXPECT_GE(centerPix.G(), 0.3);
    EXPECT_FLOAT_EQ(0.0, centerPix.R());
    EXPECT_FLOAT_EQ(0.0, centerPix.B());
  }
  // Run until 2000 steps
  const auto pressureCountBeforeRun = pressureReceiver.Count();
  const auto imageCountBeforeRun = imageReceiver.Count();
  server.Run(true, remainingSteps(target), false);
  ASSERT_TRUE(gz::sim::test::WaitUntil(5s,
      [&pressureReceiver, &imageReceiver, pressureCountBeforeRun,
       imageCountBeforeRun]()
      {
        return pressureReceiver.Count() > pressureCountBeforeRun &&
            imageReceiver.Count() > imageCountBeforeRun;
      }));

  // Check iterator state
  EXPECT_EQ(target, server.IterationCount().value());
  EXPECT_EQ(1u, this->mockSystem->configureCallCount);
  EXPECT_EQ(0u, this->mockSystem->resetCallCount);
  EXPECT_EQ(target, this->mockSystem->preUpdateCallCount);
  EXPECT_EQ(target, this->mockSystem->updateCallCount);
  EXPECT_EQ(target, this->mockSystem->postUpdateCallCount);

  // Check world state
  EXPECT_GT(pressureReceiver.Count(), pressureCountBeforeRun);
  EXPECT_GT(imageReceiver.Count(), imageCountBeforeRun);
  EXPECT_FLOAT_EQ(kEndingAltitude, poseComp->Data().Z());
  EXPECT_FLOAT_EQ(kEndingPressure, pressureReceiver.Last().pressure());

  {
    auto image = toImage(imageReceiver.Last());
    auto centerPix = image.Pixel(image.Width()/2, image.Height()/2);

    // Gray background
    EXPECT_FLOAT_EQ(centerPix.G(), centerPix.R());
    EXPECT_FLOAT_EQ(centerPix.G(), centerPix.B());
  }

  // wait until expected no. of messages are received.
  // sim runs for 2000 iterations with camera at 10 Hz + 1 msg at t=0
  ASSERT_TRUE(gz::sim::test::WaitUntil(5s,
      [&imageReceiver]()
      {
        return imageReceiver.Count() >= 21u;
      }));

  // Send command to reset to initial state
  reset::RequestWorldReset("default");

  // It takes two iterations for this to propagate,
  // the first is for the message to be received and internal state setup
  reset::ConsumeResetRequest(server);
  EXPECT_EQ(1u, this->mockSystem->configureCallCount);
  EXPECT_EQ(0u, this->mockSystem->resetCallCount);
  EXPECT_EQ(target + 1, this->mockSystem->preUpdateCallCount);
  EXPECT_EQ(target + 1, this->mockSystem->updateCallCount);
  EXPECT_EQ(target + 1, this->mockSystem->postUpdateCallCount);

  const auto imageCountBeforeResetApply = imageReceiver.Count();
  const auto pressureCountBeforeResetApply = pressureReceiver.Count();

  // The second iteration is where the reset actually occurs.
  reset::ApplyWorldReset(server);
  {
    EXPECT_EQ(1u, this->mockSystem->configureCallCount);
    EXPECT_EQ(1u, this->mockSystem->resetCallCount);

    // These should not increment, because only reset is called
    EXPECT_EQ(target + 1, this->mockSystem->preUpdateCallCount);
    EXPECT_EQ(target + 1, this->mockSystem->updateCallCount);
    EXPECT_EQ(target + 1, this->mockSystem->postUpdateCallCount);

    EXPECT_FLOAT_EQ(kStartingAltitude, poseComp->Data().Z());

    // Reset does not cause messages to be sent
    EXPECT_EQ(imageCountBeforeResetApply, imageReceiver.Count());
    EXPECT_EQ(pressureCountBeforeResetApply, pressureReceiver.Count());
  }

  target = 4001;
  const auto postResetTarget = 2000u;

  const auto pressureCountBeforePostReset = pressureReceiver.Count();
  ASSERT_TRUE(gz::sim::test::StepUntil(server, remainingSteps(postResetTarget),
      [&pressureReceiver, pressureCountBeforePostReset]()
      {
        return pressureReceiver.Count() > pressureCountBeforePostReset;
      }));
  EXPECT_GE(server.IterationCount().value(), 1u);
  EXPECT_FLOAT_EQ(kStartingPressure, pressureReceiver.Last().pressure());

  const auto imageCountBeforePostReset = imageReceiver.Count();
  ASSERT_TRUE(gz::sim::test::StepUntil(server, remainingSteps(postResetTarget),
      [&imageReceiver, imageCountBeforePostReset]()
      {
        return imageReceiver.Count() > imageCountBeforePostReset;
      }));

  // Mostly green box
  {
    auto image = toImage(imageReceiver.Last());
    auto centerPix = image.Pixel(image.Width()/2, image.Height()/2);
    EXPECT_GE(centerPix.G(), 0.3);
    EXPECT_FLOAT_EQ(0.0, centerPix.R());
    EXPECT_FLOAT_EQ(0.0, centerPix.B());
  }

  // Run until target steps
  const auto pressureCountBeforeSecondRun = pressureReceiver.Count();
  const auto imageCountBeforeSecondRun = imageReceiver.Count();

  server.Run(true, remainingSteps(postResetTarget), false);
  ASSERT_TRUE(gz::sim::test::WaitUntil(5s,
      [&pressureReceiver, &imageReceiver, pressureCountBeforeSecondRun,
       imageCountBeforeSecondRun]()
      {
        return pressureReceiver.Count() > pressureCountBeforeSecondRun &&
            imageReceiver.Count() > imageCountBeforeSecondRun;
      }));

  // Check iterator state
  EXPECT_EQ(postResetTarget, server.IterationCount().value());
  EXPECT_EQ(1u, this->mockSystem->configureCallCount);
  EXPECT_EQ(1u, this->mockSystem->resetCallCount);
  EXPECT_EQ(target, this->mockSystem->preUpdateCallCount);
  EXPECT_EQ(target, this->mockSystem->updateCallCount);
  EXPECT_EQ(target, this->mockSystem->postUpdateCallCount);

  // Check world state
  EXPECT_GT(pressureReceiver.Count(), pressureCountBeforeSecondRun);
  EXPECT_GT(imageReceiver.Count(), imageCountBeforeSecondRun);
  EXPECT_FLOAT_EQ(kEndingAltitude, poseComp->Data().Z());
  EXPECT_FLOAT_EQ(kEndingPressure, pressureReceiver.Last().pressure());

  {
    auto image = toImage(imageReceiver.Last());
    auto centerPix = image.Pixel(image.Width()/2, image.Height()/2);

    // Gray background
    EXPECT_FLOAT_EQ(centerPix.G(), centerPix.R());
    EXPECT_FLOAT_EQ(centerPix.G(), centerPix.B());
  }

}

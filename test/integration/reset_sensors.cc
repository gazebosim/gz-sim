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

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <sdf/Element.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <gz/common/Image.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/SystemPluginPtr.hh"

#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/rendering/Events.hh"

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

  /// \brief Check repeated resets with a spawned or deleted model.
  /// \param[in] _spawn True to spawn a new model; false to delete an original.
  public: void CheckResetRenderingEntities(bool _spawn);

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
void ResetFixture::CheckResetRenderingEntities(bool _spawn)
{
  sdf::Root root;
  ASSERT_TRUE(root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "reset_sensors.sdf")).empty());
  ASSERT_NE(nullptr, root.WorldByIndex(0));
  root.WorldByIndex(0)->SetGravity(math::Vector3d::Zero);

  // A spawned red box obscures the original green box in the camera image.
  sdf::Root spawnedRoot;
  ASSERT_TRUE(spawnedRoot.LoadSdfString(R"(
    <sdf version='1.10'>
      <model name='spawned_box'>
        <static>true</static>
        <pose>5 0 100 0 0 0</pose>
        <link name='link'>
          <visual name='visual'>
            <geometry><box><size>2 2 2</size></box></geometry>
            <material>
              <ambient>1 0 0 1</ambient>
              <diffuse>1 0 0 1</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>)").empty());
  ASSERT_NE(nullptr, spawnedRoot.Model());

  EntityComponentManager *ecm = nullptr;
  Entity changedEntity = kNullEntity;
  Entity cameraEntity = kNullEntity;
  bool changeModel = false;
  bool removeCamera = false;
  std::atomic<unsigned int> cameraCount{0};
  std::atomic<bool> duplicateCameras{false};
  common::ConnectionPtr postRender;
  transport::Node node;
  Subscription<msgs::Image> images;

  ServerConfig serverConfig;
  serverConfig.SetSdfRoot(root);
  Server server(serverConfig);
  server.SetUpdatePeriod(1ms);

  this->mockSystem->configureCallback =
      [&](const Entity &, const std::shared_ptr<const sdf::Element> &,
          EntityComponentManager &_ecm, EventManager &_events)
      {
        ecm = &_ecm;
        // Inspect rendering objects only from the rendering thread. This
        // connection also lets removals finish when no camera remains.
        postRender = _events.Connect<events::PostRender>([&]()
        {
          auto scene = rendering::sceneFromFirstRenderEngine();
          if (scene)
          {
            cameraCount = scene->SensorCount();
            if (cameraCount > 1u)
              duplicateCameras = true;
          }
        });
      };
  this->mockSystem->preUpdateCallback =
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        EventManager events;
        SdfEntityCreator creator(_ecm, events);
        if (changeModel)
        {
          if (_spawn)
          {
            changedEntity = creator.CreateEntities(spawnedRoot.Model());
            creator.SetParent(changedEntity,
                _ecm.EntityByComponents(components::World()));
          }
          else
          {
            changedEntity = _ecm.EntityByComponents(components::Name("box"));
            creator.RequestRemoveEntity(changedEntity, true);
          }
          changeModel = false;
        }
        if (removeCamera)
        {
          creator.RequestRemoveEntity(cameraEntity, true);
          removeCamera = false;
        }
      };
  server.AddSystem(this->systemPtr);
  ASSERT_NE(nullptr, ecm);
  cameraEntity = ecm->EntityByComponents(components::Name("camera"));
  ASSERT_NE(kNullEntity, cameraEntity);

  images.Subscribe(node, "camera", 1u);
  enum class View { Green, Red, Background };
  const auto sees = [&](View _view)
  {
    if (images.Count() == 0u)
      return false;
    auto image = toImage(images.Last());
    auto pixel = image.Pixel(image.Width() / 2, image.Height() / 2);
    if (_view == View::Green)
      return pixel.G() > 0.3 && pixel.R() < 0.05;
    if (_view == View::Red)
      return pixel.R() > 0.3 && pixel.G() < 0.05;
    return std::abs(pixel.R() - pixel.G()) < 0.01 &&
        std::abs(pixel.R() - pixel.B()) < 0.01;
  };
  ASSERT_TRUE(test::StepUntil(server, 3000,
      [&] { return cameraCount == 1u && sees(View::Green); }));

  for (unsigned int i = 0; i < 3; ++i)
  {
    SCOPED_TRACE(i);
    images.Clear();
    changeModel = true;
    ASSERT_TRUE(test::StepUntil(server, 3000, [&]()
    {
      return sees(_spawn ? View::Red : View::Background);
    }));
    ASSERT_NE(kNullEntity, changedEntity);
    EXPECT_EQ(_spawn, ecm->HasEntity(changedEntity));

    reset::RequestAndApplyWorldReset(server, "default");
    EXPECT_EQ(!_spawn, ecm->HasEntity(changedEntity));
    images.Clear();
    ASSERT_TRUE(test::StepUntil(server, 3000,
        [&] { return cameraCount == 1u && sees(View::Green); }));
    EXPECT_EQ(1u, cameraCount.load());
    EXPECT_FALSE(duplicateCameras.load());
  }

  // A deleted original camera must also be recreated, while a surviving
  // camera must have been reused throughout the preceding reset cycles.
  removeCamera = true;
  ASSERT_TRUE(test::StepUntil(server, 3000,
      [&] { return cameraCount == 0u; }));
  EXPECT_FALSE(ecm->HasEntity(cameraEntity));
  images.Clear();
  reset::RequestAndApplyWorldReset(server, "default");
  EXPECT_TRUE(ecm->HasEntity(cameraEntity));
  ASSERT_TRUE(test::StepUntil(server, 3000,
      [&] { return cameraCount == 1u && sees(View::Green); }));
  EXPECT_EQ(1u, cameraCount.load());
  EXPECT_FALSE(duplicateCameras.load());
}

/////////////////////////////////////////////////
TEST_F(ResetFixture, GZ_UTILS_TEST_DISABLED_ON_MAC(RemoveSpawnedVisuals))
{
  this->CheckResetRenderingEntities(true);
}

/////////////////////////////////////////////////
TEST_F(ResetFixture, GZ_UTILS_TEST_DISABLED_ON_MAC(RestoreRemovedVisuals))
{
  this->CheckResetRenderingEntities(false);
}

/////////////////////////////////////////////////
/// This test checks that that air-pressure and camera sensor systems
/// handle Reset events
TEST_F(ResetFixture, HandleReset)
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

  // The reset rebuilds the ECM component storage (ResetTo -> CopyFrom clones
  // every component into freshly allocated objects), which invalidates any
  // component pointer cached before the reset. Re-fetch the pose component
  // see issue #3635).
  poseComp = ecm->Component<components::Pose>(entity);
  ASSERT_NE(nullptr, poseComp);

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

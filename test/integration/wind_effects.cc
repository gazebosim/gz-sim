/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/wind.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/WindMode.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test WindEffects system
class WindEffectsTest : public InternalFixture<::testing::Test>
{
  public: void StartServer(const std::string &_sdfFile)
  {
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) + _sdfFile);
    this->server = std::make_unique<Server>(serverConfig);

    EXPECT_FALSE(this->server->Running());
    EXPECT_FALSE(*this->server->Running(0));
  }

  public: std::unique_ptr<Server> server;
};

/// \brief A System that records component values.
template <typename ComponentType>
class LinkComponentRecorder
{
  /// \brief Constructor.
  /// \param[in] _linkName Name of link.
  /// \param[in] _createComp Whether to create the component on the link. This
  /// is useful if other systems are expected to populate the component but they
  /// require the component to be created first.
  // cppcheck-suppress unmatchedSuppression
  // cppcheck-suppress passedByValue
  public: LinkComponentRecorder(std::string _linkName, bool _createComp = false)
      : linkName(std::move(_linkName))
  {
    sdf::Plugin sdfPlugin;
    sdfPlugin.SetName("gz::sim::MockSystem");
    sdfPlugin.SetFilename("libMockSystem.so");
    auto plugin = loader.LoadPlugin(sdfPlugin);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem =
        dynamic_cast<MockSystem *>(systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);

    if (_createComp)
    {
      this->mockSystem->preUpdateCallback =
        [this](const UpdateInfo &, EntityComponentManager &_ecm)
        {
          auto linkEntity = _ecm.EntityByComponents(
              components::Link(), components::Name(this->linkName));
          if (linkEntity != kNullEntity)
          {
            // Create component on the link if it doesn't already exist
            if (!_ecm.Component<ComponentType>(linkEntity))
            {
              _ecm.CreateComponent(linkEntity, ComponentType());
            }
          }
        };
    }

    this->mockSystem->postUpdateCallback =
        [this](const UpdateInfo &,
              const EntityComponentManager &_ecm)
        {
          auto linkEntity = _ecm.EntityByComponents(
              components::Link(), components::Name(this->linkName));

          if (linkEntity != kNullEntity)
          {
            auto value = _ecm.Component<ComponentType>(linkEntity);
            if (value)
            {
              this->values.push_back(*value);
            }
          }
        };
  }

  public: SystemPluginPtr systemPtr;

  /// \brief The recorded component values
  public: std::vector<ComponentType> values;
  public: std::string linkName;

  protected: SystemLoader loader;
  protected: MockSystem *mockSystem;
};

/// \brief Publisher that blocks until two messages have been received
///
/// This is a workaround that avoids using sleeps to wait for a
/// message to be received by the system under test. The idea is that the
/// publisher publishes the message twice and if the subscriber in this class
/// receives the message twice then we an be sure that the system under test
/// has received it at least once.
template <typename T>
class BlockingPublisher
{
  // cppcheck-suppress unmatchedSuppression
  // cppcheck-suppress passedByValue
  public: BlockingPublisher(std::string _topic,
                    std::chrono::milliseconds _timeOut)
        : topic(std::move(_topic)), timeOut(_timeOut)
  {
    this->pub = this->node.template Advertise<T>(this->topic);
  }

  public: bool Publish(const T &_msg)
  {
    {
      std::lock_guard<std::mutex> lock(this->onMsgMutex);
      this->onMsgCount = 0;
    }
    this->node.Subscribe(this->topic, &BlockingPublisher<T>::OnMsg, this);

    this->pub.Publish(_msg);
    // Publish a second time
    this->pub.Publish(_msg);

    std::unique_lock<std::mutex> lk(this->onMsgMutex);
    if (this->onMsgCount < 2)
    {
      // wait for onMsgCount to reach 2
      this->cv.wait_for(lk, this->timeOut,
                        [this]() { return this->onMsgCount == 2; });
    }
    return (this->onMsgCount == 2);
  }

  public: void OnMsg(const T &)
  {
    std::lock_guard<std::mutex> lock(this->onMsgMutex);
    ++this->onMsgCount;
    this->cv.notify_one();
  }

  private: std::string topic;
  private: std::chrono::milliseconds timeOut;
  private: transport::Node node;
  private: transport::Node::Publisher pub;
  private: std::mutex onMsgMutex;
  private: std::condition_variable cv;
  private: std::size_t onMsgCount{0};
};

/////////////////////////////////////////////////
/// Check if 'enable_wind' set only in <model> works
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(WindEffectsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(WindEnabledInModel))
{
  this->StartServer("/test/worlds/wind_effects.sdf");

  LinkComponentRecorder<components::WindMode> linkWindMode("box_test1");

  this->server->AddSystem(linkWindMode.systemPtr);
  EXPECT_TRUE(linkWindMode.values.empty());

  this->server->Run(true, 10, false);

  ASSERT_FALSE(linkWindMode.values.empty());
  EXPECT_TRUE(linkWindMode.values.back().Data());
}

/////////////////////////////////////////////////
/// Check if 'enable_wind' set only in <link> works
TEST_F(WindEffectsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(WindEnabledInLink))
{
  this->StartServer("/test/worlds/wind_effects.sdf");

  LinkComponentRecorder<components::WindMode> linkWindMode("box_test2");

  this->server->AddSystem(linkWindMode.systemPtr);
  EXPECT_TRUE(linkWindMode.values.empty());

  this->server->Run(true, 10, false);

  ASSERT_FALSE(linkWindMode.values.empty());
  EXPECT_TRUE(linkWindMode.values.back().Data());
}

////////////////////////////////////////////////
TEST_F(WindEffectsTest , GZ_UTILS_TEST_DISABLED_ON_WIN32(WindForce))
{
  this->StartServer("/test/worlds/wind_effects.sdf");
  LinkComponentRecorder<components::WorldLinearAcceleration> linkAccelerations(
      "box_test1", true);

  using namespace std::chrono_literals;
  this->server->SetUpdatePeriod(0ns);

  this->server->AddSystem(linkAccelerations.systemPtr);

  // Computing the exact motion of the link, without reimplementing the wind
  // effects system here, is difficult. So, here, we simply check that the
  // acceleration monotonically decreases as the velocity of the link
  // approaches the wind velocity. The complimentary filter should stabilize
  // after it has run for 3 seconds.
  const std::size_t nIters{3000};
  this->server->Run(true, nIters, false);

  EXPECT_EQ(nIters, linkAccelerations.values.size());
  linkAccelerations.values.clear();

  this->server->Run(true, nIters, false);

  ASSERT_EQ(nIters, linkAccelerations.values.size());

  double lastAccelMagnitude = linkAccelerations.values[0].Data().Length();
  for (std::size_t i = 1; i < nIters; ++i)
  {
    double accelMagnitude = linkAccelerations.values[i].Data().Length();
    // std::cout << linkAccelerations.values[i].Data() << std::endl;
    EXPECT_LT(1e-6, lastAccelMagnitude - accelMagnitude);
    lastAccelMagnitude = accelMagnitude;
  }
}

////////////////////////////////////////////////
TEST_F(WindEffectsTest , GZ_UTILS_TEST_DISABLED_ON_WIN32(ComplexWindForce))
{
  this->StartServer("/test/worlds/sea_storm_effects.sdf");
  LinkComponentRecorder<components::WorldLinearAcceleration>
      belowSurfaceAccelerations("box_below_surface", true);
  LinkComponentRecorder<components::WorldLinearAcceleration>
      aboveSurfaceAccelerations("box_above_surface", true);
  LinkComponentRecorder<components::WorldLinearAcceleration>
      upHighAccelerations("box_up_high", true);

  using namespace std::chrono_literals;
  this->server->SetUpdatePeriod(0ns);

  this->server->AddSystem(belowSurfaceAccelerations.systemPtr);
  this->server->AddSystem(aboveSurfaceAccelerations.systemPtr);
  this->server->AddSystem(upHighAccelerations.systemPtr);

  const std::size_t nIters{3000};
  this->server->Run(true, nIters, false);

  ASSERT_EQ(nIters, belowSurfaceAccelerations.values.size());
  ASSERT_EQ(nIters, aboveSurfaceAccelerations.values.size());
  ASSERT_EQ(nIters, upHighAccelerations.values.size());

  double maxAboveSurfaceAccelMagnitude = 0.;
  for (std::size_t i = 0; i < nIters; ++i)
  {
    const double belowSurfaceAccelMagnitude =
        belowSurfaceAccelerations.values[i].Data().Length();
    const double aboveSurfaceAccelMagnitude =
        aboveSurfaceAccelerations.values[i].Data().Length();
    const double upHighAccelMagnitude =
        upHighAccelerations.values[i].Data().Length();
    maxAboveSurfaceAccelMagnitude = std::max(
        maxAboveSurfaceAccelMagnitude, aboveSurfaceAccelMagnitude);
    EXPECT_LE(aboveSurfaceAccelMagnitude, upHighAccelMagnitude);
    EXPECT_LT(belowSurfaceAccelMagnitude, 1e-6);
  }
  EXPECT_GT(maxAboveSurfaceAccelMagnitude, 1e-6);
}

////////////////////////////////////////////////
TEST_F(WindEffectsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(TopicsAndServices))
{
  using namespace std::chrono_literals;

  this->StartServer("/test/worlds/wind_effects.sdf");
  this->server->Run(true, 10, false);

  // As specified in SDF
  math::Vector3d windVelSeed{10.0, 0.0, 10.0};

  transport::Node node;
  std::size_t timeout{5000};
  const std::string windService{"/world/wind_demo/wind_info"};

  {
    bool executed{false};
    msgs::Wind res;
    node.Request(windService, timeout, res, executed);

    EXPECT_TRUE(executed);
    EXPECT_TRUE(windVelSeed == msgs::Convert(res.linear_velocity()));
    EXPECT_TRUE(res.enable_wind());
  }

  // Set a new wind velocity seed
  math::Vector3d newWindVelSeed{0.0, 1.0, 0.0};
  msgs::Wind windCmd;
  msgs::Set(windCmd.mutable_linear_velocity(), newWindVelSeed);
  windCmd.set_enable_wind(false);

  BlockingPublisher<msgs::Wind> pub("/world/wind_demo/wind", timeout * 1ms);
  EXPECT_TRUE(pub.Publish(windCmd));

  // Run once to process the command
  this->server->Run(true, 1, false);

  // We can now make the request
  {
    bool executed{false};
    msgs::Wind res;
    node.Request(windService, timeout, res, executed);
    EXPECT_TRUE(executed);
    EXPECT_TRUE(newWindVelSeed == msgs::Convert(res.linear_velocity()));
    EXPECT_FALSE(res.enable_wind());
  }

  // Since wind is now disabled, the velocity should remain constant
  LinkComponentRecorder<components::WorldLinearVelocity> linkVelocities(
      "box_test1", true);
  this->server->AddSystem(linkVelocities.systemPtr);
  this->server->SetUpdatePeriod(0ns);

  const std::size_t nIters{1000};
  this->server->Run(true, nIters, false);

  ASSERT_EQ(nIters, linkVelocities.values.size());

  double lastVelMagnitude = linkVelocities.values[0].Data().Length();
  for (std::size_t i = 1; i < nIters; ++i) {
    double velMagnitude = linkVelocities.values[i].Data().Length();
    EXPECT_LT(std::fabs(lastVelMagnitude - velMagnitude), 1e-6);
    lastVelMagnitude = velMagnitude;
  }
}

/// Test if adding a link with wind after first iteration adds
/// WorldLinearVelocity component properly
TEST_F(WindEffectsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WindEntityAddedAfterStart))
{
  const std::string windBox = R"EOF(
  <?xml version="1.0" ?>
  <sdf version="1.6">
      <model name="box_wind">
        <pose>5 5 5 0 0 0</pose>
        <enable_wind>true</enable_wind>
        <link name="test_link_wind">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
        </link>
      </model>
  </sdf>)EOF";

  this->StartServer("/test/worlds/wind_effects.sdf");

  LinkComponentRecorder<components::WorldLinearVelocity>
    linkVelocityComponent("test_link_wind");
  this->server->AddSystem(linkVelocityComponent.systemPtr);
  EXPECT_TRUE(linkVelocityComponent.values.empty());

  // Run the logger for a time, check it is still empty
  this->server->Run(true, 10, false);
  EXPECT_TRUE(linkVelocityComponent.values.empty());

  // Add the box to be logged via the command system
  // and check that is not empty
  transport::Node node;
  msgs::EntityFactory req;
  unsigned int timeout = 5000;
  std::string service{"/world/wind_demo/create"};
  msgs::Boolean res;
  bool result;

  req.set_sdf(windBox);
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Now box_wind WorldLinearVelocity component should be added
  this->server->Run(true, 10, false);
  ASSERT_FALSE(linkVelocityComponent.values.empty());
}

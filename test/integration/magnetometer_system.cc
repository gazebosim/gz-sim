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

#include <gz/msgs/magnetometer.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/MagneticField.hh"
#include "gz/sim/components/Magnetometer.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define TOL 1e-4

using namespace gz;
using namespace sim;

/// \brief Test MagnetometerTest system
class MagnetometerTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
std::vector<msgs::Magnetometer> magnetometerMsgs;

/////////////////////////////////////////////////
void magnetometerCb(const msgs::Magnetometer &_msg)
{
  mutex.lock();
  magnetometerMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the detected field from a rotated magnetometer
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(MagnetometerTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(RotatedMagnetometer))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/magnetometer.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "magnetometer_sensor";

  auto topic = "world/magnetometer_sensor/model/magnetometer_model/link/link/"
      "sensor/magnetometer_sensor/magnetometer";

  // Create a system that records magnetometer data
  test::Relay testSystem;

  std::vector<math::Pose3d> poses;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Magnetometer,
                  components::Name,
                  components::WorldPose>(
            [&](const Entity &_entity,
                const components::Magnetometer *,
                const components::Name *_name,
                const components::WorldPose *_worldPose) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);
              poses.push_back(_worldPose->Data());

              auto sensorComp = _ecm.Component<components::Sensor>(_entity);
              EXPECT_NE(nullptr, sensorComp);

              if (_info.iterations == 1)
                return true;

              // This component is created on the 2nd PreUpdate
              auto topicComp = _ecm.Component<components::SensorTopic>(_entity);
              EXPECT_NE(nullptr, topicComp);
              if (topicComp)
              {
                EXPECT_EQ(topic, topicComp->Data());
              }

              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // subscribe to magnetometer topic
  transport::Node node;
  node.Subscribe(topic, &magnetometerCb);

  // step world and verify magnetometer's detected field
  // Run server
  size_t iters = 200u;
  server.Run(true, iters, false);
  EXPECT_EQ(iters, poses.size());

  // Hardcoded SDF values
  math::Vector3d worldMagneticField(0.94, 0.76, -0.12);

  math::Vector3d field = poses.back().Rot().Inverse().RotateVector(
        worldMagneticField);
  mutex.lock();
  EXPECT_NEAR(magnetometerMsgs.back().mutable_field_tesla()->x(),
      field.X(), TOL);
  EXPECT_NEAR(magnetometerMsgs.back().mutable_field_tesla()->y(),
      field.Y(), TOL);
  EXPECT_NEAR(magnetometerMsgs.back().mutable_field_tesla()->z(),
      field.Z(), TOL);
  mutex.unlock();
}


/////////////////////////////////////////////////
// Test to check to ensure that the fix for using tesla units works correctly
// Note(gilbert): It seems the above test (RotatedMagnetometer)
// does not actually run because the spherical coordinates are not set.
// In order to not break the above tests, we create a new world and run
// two simulations, one with gauss and the other with tesla units.
// See https://github.com/gazebosim/gz-sim/issues/2312

class MagnetometerMessageBuffer {
public:
  void add(const msgs::Magnetometer &_msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    magnetometerMsgs.push_back(_msg);
  }

  msgs::Magnetometer at(const size_t index) const
  {
    std::lock_guard<std::mutex> lock(mutex);
    return magnetometerMsgs.at(index);
  }

  msgs::Magnetometer latest() const
  {
    std::lock_guard<std::mutex> lock(mutex);
    return magnetometerMsgs.back();
  }

  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex);
    return magnetometerMsgs.size();
  }

private:
  mutable std::mutex mutex;
  std::vector<msgs::Magnetometer> magnetometerMsgs;
};

MagnetometerMessageBuffer gaussMessages;
MagnetometerMessageBuffer teslaMessages;

/////////////////////////////////////////////////
void gaussMagnetometerCb(const msgs::Magnetometer &_msg)
{
  gaussMessages.add(_msg);
}

/////////////////////////////////////////////////
void teslaMagnetometerCb(const msgs::Magnetometer &_msg)
{
  teslaMessages.add(_msg);
}

TEST_F(MagnetometerTest,
  GZ_UTILS_TEST_DISABLED_ON_WIN32(GaussToTeslaCorrection))
{
  // First, run the world without the fix enabled and record the magnetic field.
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/magnetometer_with_tesla.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server gaussServer(serverConfig);
  EXPECT_FALSE(gaussServer.Running());
  EXPECT_FALSE(*gaussServer.Running(0));

  const auto topic = "world/magnetometer_sensor/model/magnetometer_model/"
      "link/link/sensor/magnetometer_sensor/magnetometer";

  // subscribe to magnetometer topic
  std::unique_ptr<transport::Node> gaussNode =
    std::make_unique<transport::Node>();
  gaussNode->Subscribe(topic, &gaussMagnetometerCb);

  // step world and verify magnetometer's detected field
  // Run the gauss server
  const size_t iters = 100u;
  gaussServer.Run(true, iters, false);

  // Now, we need to create a new server where the magnetometer
  // is correctly publishing using tesla.
  // Flip the use_tesla_for_magnetic_field field to true
  sdf::SDFPtr teslaWorldSdf(new sdf::SDF());
  sdf::init(teslaWorldSdf);
  ASSERT_TRUE(sdf::readFile(sdfFile, teslaWorldSdf));
  sdf::ElementPtr root = teslaWorldSdf->Root();
  EXPECT_TRUE(root->HasElement("world"));

  sdf::ElementPtr world = root->GetElement("world");
  const std::string pluginName = "gz::sim::systems::Magnetometer";
  const std::string useTeslaElementName = "use_tesla_for_magnetic_field";
  bool elementSet = false;
  for (sdf::ElementPtr plugin = world->GetElement("plugin");
       plugin;
       plugin = plugin->GetNextElement("plugin")) {
        if (plugin->Get<std::string>("name") == pluginName) {
            // Found the magnetometer plugin, now force the field to true
            if (plugin->HasElement(useTeslaElementName)) {
                sdf::ElementPtr teslaElement =
                  plugin->GetElement(useTeslaElementName);
                teslaElement->Set(true);
                elementSet = true;
                break;
            }
        }
    }
  EXPECT_TRUE(elementSet);

  // Now, re-run the world but with the tesla units being published
  EXPECT_TRUE(serverConfig.SetSdfString(teslaWorldSdf->ToString()));

  Server teslaServer(serverConfig);
  EXPECT_FALSE(teslaServer.Running());
  EXPECT_FALSE(*teslaServer.Running(0));

  // subscribe to tesla magnetometer topic and unsubscribe from the gauss topic
  gaussNode = nullptr;
  std::unique_ptr<transport::Node> teslaNode =
    std::make_unique<transport::Node>();
  teslaNode->Subscribe(topic, &teslaMagnetometerCb);

  // step world and verify magnetometer's detected field
  // Run the tesla server
  teslaServer.Run(true, iters, false);

  // Now compare the two sets of magnetic data
  ASSERT_EQ(gaussMessages.size(), teslaMessages.size());

  for (size_t index = 0; index < teslaMessages.size(); ++index)
  {
    const auto& gaussMessage = gaussMessages.at(index);
    const auto& teslaMessage = teslaMessages.at(index);

    // 1 gauss is 10^-4 teslas
    auto gauss_to_tesla = [&](const float gauss) -> float {
        return gauss / 10'000;
    };

    EXPECT_FLOAT_EQ(gauss_to_tesla(gaussMessage.field_tesla().x()),
                    teslaMessage.field_tesla().x());
    EXPECT_FLOAT_EQ(gauss_to_tesla(gaussMessage.field_tesla().y()),
                    teslaMessage.field_tesla().y());
    EXPECT_FLOAT_EQ(gauss_to_tesla(gaussMessage.field_tesla().z()),
                    teslaMessage.field_tesla().z());
  }
}

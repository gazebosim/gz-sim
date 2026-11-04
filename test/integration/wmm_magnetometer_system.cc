/*
 * Copyright (C) 2026 Rudis Laboratories
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

#include <mutex>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/pose.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

// Include WMM directly for unit testing (source added via CMakeLists.txt)
#include "WorldMagneticModel.hh"

using namespace gz;
using namespace sim;

/// \brief Test fixture for WMM magnetometer tests
class WMMMagnetometerTest : public InternalFixture<::testing::Test>
{
};

// ============================================================
// Reference values from WMMHR2025 official tool
// All at 0 km altitude, date 2025.5
// Fields: X_north(nT), Y_east(nT), Z_down(nT), F_total(nT)
// ============================================================
struct WMMRefPoint
{
  const char *name;
  double lat;
  double lon;
  double X_nT;   // North component
  double Y_nT;   // East component
  double Z_nT;   // Down component
  double F_nT;   // Total intensity
};

static const WMMRefPoint kRefPoints[] = {
  {"Sydney",    -33.8688, 151.2093,  23980.0,   5424.1, -51488.6, 57057.3},
  {"Reykjavik",  64.1466, -21.9426,  12994.5,  -2587.7,  50914.2, 52610.0},
  {"SaoPaulo",  -23.5505, -46.6333,  16138.6,  -6479.1, -14759.5, 22809.6},
  {"Tokyo",      35.6762, 139.6503,  30086.0,  -4179.3,  35641.7, 46829.1},
  {"NorthPole",  89.9,      0.0,      1757.2,    477.1,  56772.1, 56801.3},
  {"SouthPole", -89.9,      0.0,     14445.5,  -8784.3, -51581.8, 54281.8},
  {"Equator",     0.0,      0.0,     27438.7,  -1894.2, -16001.0, 31819.8},
};

static const size_t kNumRefPoints =
    sizeof(kRefPoints) / sizeof(kRefPoints[0]);

// ============================================================
// Unit test: WorldMagneticModel::ComputeField against reference
// ============================================================
TEST_F(WMMMagnetometerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WMMComputeFieldAccuracy))
{
  systems::WorldMagneticModel wmm;
  std::string cofPath = std::string(PROJECT_SOURCE_PATH) +
      "/src/systems/magnetometer/wmm/WMMHR.COF";

  ASSERT_TRUE(wmm.Load(cofPath, 2025.5))
      << "Failed to load COF file: " << cofPath;
  EXPECT_TRUE(wmm.IsLoaded());
  EXPECT_EQ(133, wmm.MaxDegree());
  EXPECT_DOUBLE_EQ(2025.5, wmm.DecimalYear());

  // Tolerance: 5 nT for components, 5 nT for total intensity.
  // The reference tool rounds to 0.1 nT, so this is very generous.
  constexpr double kTolNT = 5.0;

  for (size_t i = 0; i < kNumRefPoints; ++i)
  {
    const auto &ref = kRefPoints[i];
    auto result = wmm.ComputeField(ref.lat, ref.lon, 0.0);

    EXPECT_NEAR(result.fieldNED.X(), ref.X_nT, kTolNT)
        << "X (North) mismatch at " << ref.name;
    EXPECT_NEAR(result.fieldNED.Y(), ref.Y_nT, kTolNT)
        << "Y (East) mismatch at " << ref.name;
    EXPECT_NEAR(result.fieldNED.Z(), ref.Z_nT, kTolNT)
        << "Z (Down) mismatch at " << ref.name;
    EXPECT_NEAR(result.totalIntensity, ref.F_nT, kTolNT)
        << "Total intensity mismatch at " << ref.name;
  }
}

// ============================================================
// Unit test: WMM at geographic poles (singularity handling)
// ============================================================
TEST_F(WMMMagnetometerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WMMPolarSingularity))
{
  systems::WorldMagneticModel wmm;
  std::string cofPath = std::string(PROJECT_SOURCE_PATH) +
      "/src/systems/magnetometer/wmm/WMMHR.COF";
  ASSERT_TRUE(wmm.Load(cofPath, 2025.5));

  // Exact poles: should not crash or produce NaN
  auto north = wmm.ComputeField(90.0, 0.0, 0.0);
  EXPECT_FALSE(std::isnan(north.fieldNED.X()));
  EXPECT_FALSE(std::isnan(north.fieldNED.Y()));
  EXPECT_FALSE(std::isnan(north.fieldNED.Z()));
  EXPECT_GT(north.totalIntensity, 50000.0);  // ~56000 nT at north pole

  auto south = wmm.ComputeField(-90.0, 0.0, 0.0);
  EXPECT_FALSE(std::isnan(south.fieldNED.X()));
  EXPECT_FALSE(std::isnan(south.fieldNED.Y()));
  EXPECT_FALSE(std::isnan(south.fieldNED.Z()));
  EXPECT_GT(south.totalIntensity, 50000.0);  // ~54000 nT at south pole
}

// ============================================================
// Unit test: WMM at various altitudes
// ============================================================
TEST_F(WMMMagnetometerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WMMAltitudeVariation))
{
  systems::WorldMagneticModel wmm;
  std::string cofPath = std::string(PROJECT_SOURCE_PATH) +
      "/src/systems/magnetometer/wmm/WMMHR.COF";
  ASSERT_TRUE(wmm.Load(cofPath, 2025.5));

  // Field strength should decrease with altitude
  auto ground = wmm.ComputeField(0.0, 0.0, 0.0);
  auto high = wmm.ComputeField(0.0, 0.0, 100.0);  // 100 km altitude

  EXPECT_GT(ground.totalIntensity, high.totalIntensity)
      << "Field should be weaker at higher altitude";
}

// ============================================================
// Integration test helpers
// ============================================================
static std::mutex g_mutex;
static std::vector<msgs::Magnetometer> g_magnetometerMsgs;

void wmmMagnetometerCb(const msgs::Magnetometer &_msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_magnetometerMsgs.push_back(_msg);
}

// ============================================================
// Integration test: Default ENU/Tesla at Sydney
// ============================================================
TEST_F(WMMMagnetometerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WMMIntegrationENUTesla))
{
  g_magnetometerMsgs.clear();

  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_BINARY_PATH) +
      "/test/worlds/magnetometer_wmm.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  auto topic =
      "world/magnetometer_wmm_sensor/model/magnetometer_model/"
      "link/link/sensor/magnetometer_sensor/magnetometer";

  transport::Node node;
  node.Subscribe(topic, &wmmMagnetometerCb);

  // Run enough iterations for the sensor to publish
  server.Run(true, 200, false);

  std::lock_guard<std::mutex> lock(g_mutex);
  ASSERT_FALSE(g_magnetometerMsgs.empty())
      << "No magnetometer messages received";

  const auto &msg = g_magnetometerMsgs.back();

  // Reference: Sydney at 0 km, 2025.5
  // NED: X=23980.0, Y=5424.1, Z=-51488.6 (nT)
  // ENU conversion: X_enu = Y_ned, Y_enu = X_ned, Z_enu = -Z_ned
  // In Tesla: multiply by 1e-9
  double refENU_X = 5424.1e-9;   // East = Y_ned
  double refENU_Y = 23980.0e-9;  // North = X_ned
  double refENU_Z = 51488.6e-9;  // Up = -Z_ned

  // Tolerance: 50 nT = 50e-9 T (generous to account for sensor pose effects)
  constexpr double kTolT = 50.0e-9;

  EXPECT_NEAR(msg.field_tesla().x(), refENU_X, kTolT)
      << "ENU X (East) mismatch";
  EXPECT_NEAR(msg.field_tesla().y(), refENU_Y, kTolT)
      << "ENU Y (North) mismatch";
  EXPECT_NEAR(msg.field_tesla().z(), refENU_Z, kTolT)
      << "ENU Z (Up) mismatch";

  // Verify unit and frame metadata
  EXPECT_EQ(msg.unit(), msgs::Magnetometer::TESLA);
  EXPECT_EQ(msg.frame(), msgs::Magnetometer::ENU);
}

// ============================================================
// Integration test: NED/Gauss at Tokyo
// ============================================================
TEST_F(WMMMagnetometerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WMMIntegrationNEDGauss))
{
  g_magnetometerMsgs.clear();

  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_BINARY_PATH) +
      "/test/worlds/magnetometer_wmm_ned_gauss.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  auto topic =
      "world/magnetometer_wmm_ned_gauss_sensor/model/magnetometer_model/"
      "link/link/sensor/magnetometer_sensor/magnetometer";

  transport::Node node;
  node.Subscribe(topic, &wmmMagnetometerCb);

  server.Run(true, 200, false);

  std::lock_guard<std::mutex> lock(g_mutex);
  ASSERT_FALSE(g_magnetometerMsgs.empty())
      << "No magnetometer messages received";

  const auto &msg = g_magnetometerMsgs.back();

  // Reference: Tokyo at 0 km, 2025.5
  // NED: X=30086.0, Y=-4179.3, Z=35641.7 (nT)
  // In Gauss: multiply by 1e-5
  double refNED_X = 30086.0e-5;   // North
  double refNED_Y = -4179.3e-5;   // East
  double refNED_Z = 35641.7e-5;   // Down

  // Tolerance: 50 nT = 50e-5 Gauss
  constexpr double kTolG = 50.0e-5;

  EXPECT_NEAR(msg.field_tesla().x(), refNED_X, kTolG)
      << "NED X (North) mismatch";
  EXPECT_NEAR(msg.field_tesla().y(), refNED_Y, kTolG)
      << "NED Y (East) mismatch";
  EXPECT_NEAR(msg.field_tesla().z(), refNED_Z, kTolG)
      << "NED Z (Down) mismatch";

  // Verify unit and frame metadata
  EXPECT_EQ(msg.unit(), msgs::Magnetometer::GAUSS);
  EXPECT_EQ(msg.frame(), msgs::Magnetometer::NED);
}

// ============================================================
// Integration test: Transport magnetometer to new location
// triggers WMM recomputation
// ============================================================
TEST_F(WMMMagnetometerTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WMMTransportRecalculation))
{
  g_magnetometerMsgs.clear();

  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_BINARY_PATH) +
      "/test/worlds/magnetometer_wmm_transport.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  auto topic =
      "world/magnetometer_wmm_transport/model/magnetometer_model/"
      "link/link/sensor/magnetometer_sensor/magnetometer";

  transport::Node node;
  node.Subscribe(topic, &wmmMagnetometerCb);

  // --- Phase 1: Get field at origin (equator, 0,0) ---
  server.Run(true, 200, false);

  math::Vector3d fieldAtOrigin;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    ASSERT_FALSE(g_magnetometerMsgs.empty())
        << "No magnetometer messages at origin";
    const auto &msg = g_magnetometerMsgs.back();
    fieldAtOrigin.Set(
        msg.field_tesla().x(),
        msg.field_tesla().y(),
        msg.field_tesla().z());
  }

  // Verify field at origin roughly matches Equator/0,0 reference
  // ENU: X_enu=Y_ned=-1894.2e-9, Y_enu=X_ned=27438.7e-9, Z_enu=-Z_ned=16001.0e-9
  EXPECT_NEAR(fieldAtOrigin.X(), -1894.2e-9, 50e-9)
      << "Origin East component mismatch";
  EXPECT_NEAR(fieldAtOrigin.Y(), 27438.7e-9, 50e-9)
      << "Origin North component mismatch";
  EXPECT_NEAR(fieldAtOrigin.Z(), 16001.0e-9, 50e-9)
      << "Origin Up component mismatch";

  // --- Phase 2: Move model far away (roughly 500km north = ~4.5 degrees) ---
  // At ENU with heading 0, Y axis is North. 500000m north ≈ 4.5 degrees lat.
  // This exceeds the 100m recalculation distance.
  g_magnetometerMsgs.clear();

  msgs::Pose req;
  req.set_name("magnetometer_model");
  req.mutable_position()->set_y(500000.0);  // 500 km north

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/magnetometer_wmm_transport/set_pose"};

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run more iterations to get updated field
  server.Run(true, 200, false);

  math::Vector3d fieldAtNewPos;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    ASSERT_FALSE(g_magnetometerMsgs.empty())
        << "No magnetometer messages after transport";
    const auto &msg = g_magnetometerMsgs.back();
    fieldAtNewPos.Set(
        msg.field_tesla().x(),
        msg.field_tesla().y(),
        msg.field_tesla().z());
  }

  // The field should have changed significantly after moving 500km
  double fieldDiff = (fieldAtNewPos - fieldAtOrigin).Length();
  EXPECT_GT(fieldDiff, 100e-9)
      << "Field should change significantly after 500km transport. "
      << "Origin: " << fieldAtOrigin << " New: " << fieldAtNewPos;

  // The new field should still be reasonable (not zero or garbage)
  double totalIntensity = fieldAtNewPos.Length();
  EXPECT_GT(totalIntensity, 20000e-9)  // > 20000 nT
      << "Field intensity should be physically reasonable";
  EXPECT_LT(totalIntensity, 70000e-9)  // < 70000 nT
      << "Field intensity should be physically reasonable";
}

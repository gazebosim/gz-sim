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

#include <gz/msgs/fluid_pressure.pb.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Thread-safe latch capturing the most recent FluidPressure
/// sample received on a single topic.
class PressureLatch
{
  /// \brief Constructor. Subscribes to the given topic.
  /// \param[in] _topic Topic to subscribe to.
  public: explicit PressureLatch(const std::string &_topic)
  {
    this->node.Subscribe(_topic, &PressureLatch::OnMsg, this);
  }

  /// \brief Block until at least one sample has been received or the
  /// timeout expires.
  /// \param[in] _timeout Maximum time to wait.
  /// \return True if a sample was received before the timeout.
  public: bool WaitForSample(std::chrono::milliseconds _timeout) const
  {
    const auto deadline = std::chrono::steady_clock::now() + _timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (this->haveSample.load(std::memory_order_acquire))
        return true;
      std::this_thread::sleep_for(5ms);
    }
    return this->haveSample.load(std::memory_order_acquire);
  }

  /// \brief Get the latest pressure reading received.
  /// \return Pressure in Pa.
  public: double PressurePa() const
  {
    std::lock_guard<std::mutex> lk(this->mtx);
    return this->pressurePa;
  }

  /// \brief Subscriber callback.
  private: void OnMsg(const msgs::FluidPressure &_msg)
  {
    std::lock_guard<std::mutex> lk(this->mtx);
    this->pressurePa = _msg.pressure();
    this->haveSample.store(true, std::memory_order_release);
  }

  /// \brief Transport node.
  private: transport::Node node;

  /// \brief Mutex guarding the latest pressure sample.
  private: mutable std::mutex mtx;

  /// \brief Latest pressure sample in Pa.
  private: double pressurePa{0.0};

  /// \brief Whether at least one sample has been received.
  private: std::atomic<bool> haveSample{false};
};

/// \brief Test the HydrostaticPressure system.
class HydrostaticPressureTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(HydrostaticPressureTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(AllZones))
{
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "hydrostatic_pressure.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  PressureLatch surface("/test/pressure/surface");
  PressureLatch shallow("/test/pressure/shallow");
  PressureLatch deep   ("/test/pressure/deep");
  PressureLatch above  ("/test/pressure/above");

  // Discovery window so subscribers match the plugin publishers before
  // the first publish lands.
  std::this_thread::sleep_for(100ms);

  // Plugins publish at 200 Hz; running 1000 iterations of 1 ms sim time
  // yields several samples per topic.
  server.Run(true, 1000, false);

  ASSERT_TRUE(surface.WaitForSample(2000ms));
  ASSERT_TRUE(shallow.WaitForSample(2000ms));
  ASSERT_TRUE(deep.WaitForSample(2000ms));
  ASSERT_TRUE(above.WaitForSample(2000ms));

  constexpr double kSurfacePa = 101325.0;
  constexpr double kFluidDensity = 1025.0;
  constexpr double kGravity = 9.80665;

  // Surface (z = 0): depth = 0 -> polynomial returns 0 -> pressure
  // matches surface pressure exactly.
  EXPECT_NEAR(surface.PressurePa(), kSurfacePa, 1.0);

  // Shallow (z = -10): F&M (1983) and the simple rho*g*h form agree to
  // ~0.3 % at shallow depths. F&M's effective column density is
  // ~1028 kg/m^3 vs. the surface density 1025 kg/m^3 used by rho*g*h,
  // so F&M reads slightly higher. A 0.5 % tolerance covers that gap
  // with margin and is still well below sensor noise.
  const double simple = kSurfacePa + kFluidDensity * kGravity * 10.0;
  EXPECT_NEAR(shallow.PressurePa(), simple, 0.005 * simple);

  // 1000 m (z = -1000): UNESCO Technical Papers in Marine Science
  // No. 44 (Saunders 1981) at standard ocean conditions: 1000 m of
  // seawater ~= 1010.6 dbar ~= 10.106 MPa above surface. 0.1 %
  // tolerance is F&M's documented accuracy bound.
  constexpr double kDeepDeltaPa = 1.0106e7;
  const double expectedDeep = kSurfacePa + kDeepDeltaPa;
  EXPECT_NEAR(deep.PressurePa(), expectedDeep, 0.001 * expectedDeep);

  // Above water (z = +5): a real depth sensor floors at atmospheric -
  // the plugin clamps the depth to zero rather than reporting a
  // negative pressure delta.
  EXPECT_NEAR(above.PressurePa(), kSurfacePa, 1.0);
}

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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/diagnostics.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/System.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

namespace
{
const char kReportTopic[] = "/world/default/performance/report";
const char kEnableService[] = "/world/default/performance/enable";

/// \brief A system that makes simulation steps slow.
class SlowSystem : public System, public ISystemPreUpdate
{
  /// \brief Constructor
  /// \param[in] _spin True to use the CPU, false to be blocked
  /// \param[in] _duration Time that it takes in a step
  /// \param[in] _every It takes time once in this many steps
  /// \param[in] _phase In which of those steps, from 0 to _every - 1
  public: explicit SlowSystem(bool _spin,
              std::chrono::steady_clock::duration _duration = 5ms,
              uint64_t _every = 1u, uint64_t _phase = 0u)
      : spin(_spin), duration(_duration), every(_every), phase(_phase) {}

  // Documentation inherited
  public: void PreUpdate(const UpdateInfo &_info,
              EntityComponentManager &) override
  {
    if (_info.iterations % this->every != this->phase)
      return;

    const auto end = std::chrono::steady_clock::now() + this->duration;
    if (this->spin)
    {
      while (std::chrono::steady_clock::now() < end)
      {
      }
    }
    else
    {
      std::this_thread::sleep_until(end);
    }
  }

  /// \brief True to use the CPU, false to be blocked
  private: bool spin;

  /// \brief Time that it takes in a step
  private: std::chrono::steady_clock::duration duration;

  /// \brief It takes time once in this many steps
  private: uint64_t every;

  /// \brief In which of those steps it takes time
  private: uint64_t phase;
};

/// \brief The verdict of a report
struct Verdict
{
  std::string resource;
  std::string location;
  std::string culprit;
  bool limited{false};
  double rtf{0.0};

  /// \brief Why each sensor without data had none, by sensor name
  std::map<std::string, std::string> dataless;
};

/// \brief Collects reports from the performance topic.
class ReportListener
{
  /// \brief Constructor
  public: ReportListener()
  {
    this->node.Subscribe(kReportTopic, &ReportListener::OnReport, this);
  }

  /// \brief Wait for a report
  /// \param[in] _timeout Time to wait
  /// \param[out] _verdict The last report received
  /// \return True if a report arrived
  public: bool Wait(std::chrono::seconds _timeout, Verdict &_verdict)
  {
    const auto end = std::chrono::steady_clock::now() + _timeout;
    while (std::chrono::steady_clock::now() < end)
    {
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (this->count > 0)
        {
          _verdict = this->last;
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  /// \brief Forget the reports received so far
  public: void Reset()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->count = 0;
  }

  /// \brief Number of reports received
  /// \return The count
  public: int Count()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->count;
  }

  /// \brief Lowest real time factor received so far
  /// \return The real time factor
  public: double MinRtf()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->minRtf;
  }

  /// \brief Callback of the report topic
  /// \param[in] _msg The report
  private: void OnReport(const msgs::Diagnostics &_msg)
  {
    std::map<std::string, std::string> data;
    std::map<std::string, std::string> dataless;
    const std::string datalessPrefix{"dataless_sensor/"};
    for (const auto &entry : _msg.header().data())
    {
      if (entry.value_size() < 1)
        continue;
      data[entry.key()] = entry.value(0);
      if (entry.key().rfind(datalessPrefix, 0) == 0)
        dataless[entry.key().substr(datalessPrefix.size())] = entry.value(0);
    }

    std::lock_guard<std::mutex> lock(this->mutex);
    this->last.resource = data["resource"];
    this->last.location = data["location"];
    this->last.culprit = data["culprit"];
    this->last.limited = data["limited"] == "true";
    this->last.rtf = _msg.real_time_factor();
    this->last.dataless = std::move(dataless);
    this->minRtf = std::min(this->minRtf, this->last.rtf);
    ++this->count;
  }

  private: transport::Node node;
  private: std::mutex mutex;
  private: Verdict last;
  private: int count{0};
  private: double minRtf{std::numeric_limits<double>::max()};
};

/// \brief Get a configuration that loads a world with a few models
ServerConfig shapesConfig()
{
  ServerConfig config;
  config.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "shapes.sdf"));
  return config;
}

/// \brief Get a configuration that loads a world with a camera and an IMU,
/// which only work while something subscribes to them
ServerConfig sensorsConfig()
{
  ServerConfig config;
  config.SetSdfString(R"(<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <model name="bot">
      <static>true</static>
      <link name="link">
        <sensor name="camera" type="camera">
          <update_rate>10</update_rate>
          <topic>/performance_test/image</topic>
          <camera>
            <camera_info_topic>/performance_test/camera_info</camera_info_topic>
            <horizontal_fov>1.0</horizontal_fov>
            <image><width>64</width><height>48</height></image>
            <clip><near>0.1</near><far>10</far></clip>
          </camera>
        </sensor>
        <sensor name="imu" type="imu">
          <update_rate>100</update_rate>
          <topic>/performance_test/imu</topic>
        </sensor>
      </link>
    </model>
  </world>
</sdf>)");
  return config;
}

/// \brief Get a configuration that loads a world with two models, which runs
/// at real time
ServerConfig realTimeConfig()
{
  ServerConfig config;
  config.SetSdfString(R"(<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
      </link>
    </model>
    <model name="sphere">
      <pose>0 3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><sphere><radius>0.5</radius></sphere></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>)");
  return config;
}
}  // namespace

/// \brief Test the performance monitor with a running simulation.
class PerformanceMonitorTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void TearDown() override
  {
    common::unsetenv("GZ_SIM_PERFORMANCE_MONITOR");
    InternalFixture::TearDown();
  }
};

/////////////////////////////////////////////////
TEST_F(PerformanceMonitorTest, NoReportsUnlessRequested)
{
  ReportListener listener;

  Server server(shapesConfig());
  ASSERT_TRUE(server.Run(false, 0, false));

  std::this_thread::sleep_for(4s);
  EXPECT_EQ(0, listener.Count());
  server.Stop();
}

/////////////////////////////////////////////////
TEST_F(PerformanceMonitorTest, BusySystemOnModelIsCpu)
{
  common::setenv("GZ_SIM_PERFORMANCE_MONITOR", "1");
  ReportListener listener;

  Server server(shapesConfig());
  const auto box = server.EntityByName("box");
  ASSERT_TRUE(box.has_value());
  ASSERT_TRUE(server.AddSystem(std::make_shared<SlowSystem>(true), *box,
        std::nullopt).value_or(false));
  ASSERT_TRUE(server.Run(false, 0, false));

  Verdict verdict;
  ASSERT_TRUE(listener.Wait(30s, verdict));
  EXPECT_TRUE(verdict.limited);
  EXPECT_EQ("cpu", verdict.resource);
  EXPECT_EQ("system", verdict.location);
  EXPECT_NE(std::string::npos, verdict.culprit.find("on model box"))
      << verdict.culprit;
  EXPECT_LT(verdict.rtf, 0.5);
  server.Stop();
}

/////////////////////////////////////////////////
TEST_F(PerformanceMonitorTest, BlockedSystemIsExternal)
{
  common::setenv("GZ_SIM_PERFORMANCE_MONITOR", "1");
  ReportListener listener;

  Server server(shapesConfig());
  ASSERT_TRUE(server.AddSystem(
        std::make_shared<SlowSystem>(false)).value_or(false));
  ASSERT_TRUE(server.Run(false, 0, false));

  Verdict verdict;
  ASSERT_TRUE(listener.Wait(30s, verdict));
  EXPECT_TRUE(verdict.limited);
  EXPECT_EQ("external", verdict.resource);
  EXPECT_EQ("system", verdict.location);
  server.Stop();
}

/////////////////////////////////////////////////
// At a real time target only the steps that run past their slot lower the
// RTF. A system that fits in the slot of every step isn't the bottleneck,
// even if it takes more time overall than one that makes a few steps late.
TEST_F(PerformanceMonitorTest, LateStepsBeatWorkThatFitsTheSlot)
{
  common::setenv("GZ_SIM_PERFORMANCE_MONITOR", "1");
  ReportListener listener;

  Server server(realTimeConfig());
  const auto box = server.EntityByName("box");
  ASSERT_TRUE(box.has_value());
  const auto sphere = server.EntityByName("sphere");
  ASSERT_TRUE(sphere.has_value());
  // 0.6 ms of every 1 ms step: 30 ms per 50 steps, which costs no real time
  ASSERT_TRUE(server.AddSystem(std::make_shared<SlowSystem>(true, 600us),
        *box, std::nullopt).value_or(false));
  // 20 ms once every 50 steps, which costs about 20 ms per 50 steps
  ASSERT_TRUE(server.AddSystem(std::make_shared<SlowSystem>(true, 20ms, 50u),
        *sphere, std::nullopt).value_or(false));
  ASSERT_TRUE(server.Run(false, 0, false));

  // Skip the first report, the warm up may still be in it
  Verdict verdict;
  ASSERT_TRUE(listener.Wait(30s, verdict));
  listener.Reset();
  ASSERT_TRUE(listener.Wait(30s, verdict));
  EXPECT_TRUE(verdict.limited);
  EXPECT_LT(verdict.rtf, 0.9);
  EXPECT_EQ("system", verdict.location);
  EXPECT_NE(std::string::npos, verdict.culprit.find("on model sphere"))
      << verdict.culprit;
  server.Stop();
}

/////////////////////////////////////////////////
// A step that runs past its slot by less than a slot doesn't cost real time
// if the next step is fast: the loop keeps its schedule and the next step
// makes up for it. Only the steps that put the loop a whole slot behind do.
TEST_F(PerformanceMonitorTest, OverrunsThatTheNextStepMakesUpForAreNotLate)
{
  common::setenv("GZ_SIM_PERFORMANCE_MONITOR", "1");
  ReportListener listener;

  Server server(realTimeConfig());
  const auto box = server.EntityByName("box");
  ASSERT_TRUE(box.has_value());
  const auto sphere = server.EntityByName("sphere");
  ASSERT_TRUE(sphere.has_value());
  // 1.45 ms every other step, about 0.5 ms past the slot each time, which
  // the fast step after it makes up for: 50 ms past the slot per 200 steps
  // that cost no real time
  ASSERT_TRUE(server.AddSystem(std::make_shared<SlowSystem>(true, 1450us, 2u),
        *box, std::nullopt).value_or(false));
  // 20 ms once every 200 steps, which costs about 20 ms per 200 steps
  ASSERT_TRUE(server.AddSystem(
        std::make_shared<SlowSystem>(true, 20ms, 200u, 1u),
        *sphere, std::nullopt).value_or(false));
  ASSERT_TRUE(server.Run(false, 0, false));

  // Skip the first report, the warm up may still be in it
  Verdict verdict;
  ASSERT_TRUE(listener.Wait(30s, verdict));
  listener.Reset();
  ASSERT_TRUE(listener.Wait(30s, verdict));
  EXPECT_TRUE(verdict.limited);
  EXPECT_LT(verdict.rtf, 0.95);
  EXPECT_EQ("system", verdict.location);
  EXPECT_NE(std::string::npos, verdict.culprit.find("on model sphere"))
      << verdict.culprit;
  server.Stop();
}

/////////////////////////////////////////////////
TEST_F(PerformanceMonitorTest, EnabledAtRuntimeThroughService)
{
  ReportListener listener;

  Server server(shapesConfig());
  ASSERT_TRUE(server.AddSystem(
        std::make_shared<SlowSystem>(true)).value_or(false));
  ASSERT_TRUE(server.Run(false, 0, false));

  // Nothing is reported yet
  std::this_thread::sleep_for(1s);
  EXPECT_EQ(0, listener.Count());

  transport::Node node;
  msgs::Boolean req;
  msgs::Boolean rep;
  bool result{false};
  req.set_data(true);
  ASSERT_TRUE(node.Request(kEnableService, req, 5000u, rep, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(rep.data());

  Verdict verdict;
  ASSERT_TRUE(listener.Wait(30s, verdict));
  EXPECT_EQ("cpu", verdict.resource);

  // Disable it again, no more reports should arrive
  req.set_data(false);
  ASSERT_TRUE(node.Request(kEnableService, req, 5000u, rep, result));
  EXPECT_TRUE(result);
  EXPECT_FALSE(rep.data());

  // Let a report that was on its way arrive before counting
  std::this_thread::sleep_for(1s);
  listener.Reset();
  std::this_thread::sleep_for(8s);
  EXPECT_EQ(0, listener.Count());
  server.Stop();
}

/////////////////////////////////////////////////
TEST_F(PerformanceMonitorTest, WorldResetDoesNotProduceBogusReports)
{
  common::setenv("GZ_SIM_PERFORMANCE_MONITOR", "1");
  ReportListener listener;

  Server server(shapesConfig());
  ASSERT_TRUE(server.AddSystem(
        std::make_shared<SlowSystem>(true)).value_or(false));
  ASSERT_TRUE(server.Run(false, 0, false));

  Verdict verdict;
  ASSERT_TRUE(listener.Wait(30s, verdict));

  transport::Node node;
  msgs::WorldControl req;
  msgs::Boolean rep;
  bool result{false};
  req.mutable_reset()->set_all(true);
  ASSERT_TRUE(node.Request("/world/default/control", req, 5000u, rep, result));
  EXPECT_TRUE(result);

  // Reports continue after the reset, none with a negative real time factor.
  listener.Reset();
  ASSERT_TRUE(listener.Wait(30s, verdict));
  EXPECT_EQ("cpu", verdict.resource);
  EXPECT_GE(listener.MinRtf(), 0.0);
  server.Stop();
}

/////////////////////////////////////////////////
// Sensors only work while something subscribes to their data. The reports say
// which sensors produced no data, and why.
TEST_F(PerformanceMonitorTest,
    GZ_UTILS_TEST_DISABLED_ON_MAC(SensorsWithoutDataAreReported))
{
  common::setenv("GZ_SIM_PERFORMANCE_MONITOR", "1");
  ReportListener listener;

  Server server(sensorsConfig());
  ASSERT_TRUE(server.Run(false, 0, false));

  // The second report from now. The first one can start before a change.
  const auto nextVerdict = [&listener]()
  {
    listener.Reset();
    const auto end = std::chrono::steady_clock::now() + 60s;
    while (listener.Count() < 2 && std::chrono::steady_clock::now() < end)
      std::this_thread::sleep_for(100ms);
    EXPECT_LE(2, listener.Count());
    Verdict verdict;
    EXPECT_TRUE(listener.Wait(1s, verdict));
    return verdict;
  };

  auto verdict = nextVerdict();
  EXPECT_EQ("no_subscribers", verdict.dataless["bot::link::camera"]);
  EXPECT_EQ("no_subscribers", verdict.dataless["bot::link::imu"]);

  transport::Node node;
  std::atomic<int> infos{0};
  std::function<void(const msgs::CameraInfo &)> onInfo =
      [&infos](const msgs::CameraInfo &) { ++infos; };
  ASSERT_TRUE(node.Subscribe("/performance_test/camera_info", onInfo));
  std::function<void(const msgs::IMU &)> onImu = [](const msgs::IMU &) {};
  ASSERT_TRUE(node.Subscribe("/performance_test/imu", onImu));
  verdict = nextVerdict();
  EXPECT_LT(0, infos.load());
  EXPECT_EQ("info_only", verdict.dataless["bot::link::camera"]);
  EXPECT_EQ(0u, verdict.dataless.count("bot::link::imu"));

  std::atomic<int> images{0};
  std::function<void(const msgs::Image &)> onImage =
      [&images](const msgs::Image &) { ++images; };
  ASSERT_TRUE(node.Subscribe("/performance_test/image", onImage));
  verdict = nextVerdict();
  EXPECT_LT(0, images.load());
  EXPECT_TRUE(verdict.dataless.empty());
  server.Stop();
}

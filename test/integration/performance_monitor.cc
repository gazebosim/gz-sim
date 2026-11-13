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
#include <gz/msgs/diagnostics.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
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

/// \brief A system that makes every simulation step slow.
class SlowSystem : public System, public ISystemPreUpdate
{
  /// \brief Constructor
  /// \param[in] _spin True to use the CPU, false to be blocked
  public: explicit SlowSystem(bool _spin) : spin(_spin) {}

  // Documentation inherited
  public: void PreUpdate(const UpdateInfo &,
              EntityComponentManager &) override
  {
    const auto end = std::chrono::steady_clock::now() + 5ms;
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
};

/// \brief A system that makes every simulation step slow in PostUpdate.
/// Post updates run in their own threads when they are parallel, which is the
/// default in this version.
class SlowPostUpdateSystem : public System, public ISystemPostUpdate
{
  // Documentation inherited
  public: void PostUpdate(const UpdateInfo &,
              const EntityComponentManager &) override
  {
    const auto end = std::chrono::steady_clock::now() + 5ms;
    while (std::chrono::steady_clock::now() < end)
    {
    }
  }
};

/// \brief The verdict of a report
struct Verdict
{
  std::string resource;
  std::string location;
  std::string culprit;
  bool limited{false};
  double rtf{0.0};
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
    for (const auto &entry : _msg.header().data())
    {
      if (entry.value_size() > 0)
        data[entry.key()] = entry.value(0);
    }

    std::lock_guard<std::mutex> lock(this->mutex);
    this->last.resource = data["resource"];
    this->last.location = data["location"];
    this->last.culprit = data["culprit"];
    this->last.limited = data["limited"] == "true";
    this->last.rtf = _msg.real_time_factor();
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
TEST_F(PerformanceMonitorTest, BusyPostUpdateSystemOnModelIsCpu)
{
  common::setenv("GZ_SIM_PERFORMANCE_MONITOR", "1");
  ReportListener listener;

  Server server(shapesConfig());
  const auto box = server.EntityByName("box");
  ASSERT_TRUE(box.has_value());
  ASSERT_TRUE(server.AddSystem(std::make_shared<SlowPostUpdateSystem>(),
        *box, std::nullopt).value_or(false));
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

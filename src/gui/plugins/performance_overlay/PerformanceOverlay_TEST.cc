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

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/gui/GuiEvents.hh"
#include "test_config.hh"
#include "../../../../test/helpers/EnvTestFixture.hh"

#include "../../GuiRunner.hh"
#include "PerformanceOverlay.hh"
#include "PerformanceOverlayData.hh"

int g_argc = 1;
char **g_argv;

using namespace gz;
using sim::Entity;
using sim::EntityComponentManager;
using sim::ParsePerformanceReport;
using sim::PerformanceOverlayReport;
using sim::PerformanceOverlaySensor;
using sim::UpdateInfo;
using sim::kPerformanceOverlayRows;
namespace components = sim::components;

namespace
{
/// \brief Add a key and value to the header of a report
void addData(msgs::Diagnostics &_msg, const std::string &_key,
    const std::string &_value)
{
  auto *data = _msg.mutable_header()->add_data();
  data->set_key(_key);
  data->add_value(_value);
}

/// \brief Add a time to a report, as a share of one second
void addShare(msgs::Diagnostics &_msg, const std::string &_name, double _share)
{
  auto *entry = _msg.add_time();
  entry->set_name(_name);
  entry->mutable_elapsed()->set_nsec(static_cast<int32_t>(_share * 1e9));
  entry->mutable_wall()->set_sec(1);
}

/// \brief A report like the ones of the monitor
msgs::Diagnostics physicsReport()
{
  msgs::Diagnostics msg;
  msg.set_real_time_factor(0.31);
  msg.mutable_real_time()->set_sec(5);
  addData(msg, "limited", "true");
  addData(msg, "resource", "cpu");
  addData(msg, "location", "physics");
  addData(msg, "culprit", "fidget_spinner");
  addData(msg, "desired_rtf", "1.000000");
  addData(msg, "iterations", "1550");
  addData(msg, "render_frames", "60");
  addData(msg, "software_renderer", "false");
  addShare(msg, "step/physics", 0.8);
  addShare(msg, "step/render_wait", 0.1);
  addShare(msg, "step/other_systems", 0.04);
  addShare(msg, "step/loop", 0.02);
  addShare(msg, "render/total", 0.5);
  addShare(msg, "render/gpu_wait", 0.1);
  addShare(msg, "sensor/robot::base::camera", 0.4);
  addShare(msg, "sensor/robot::base::camera/gpu_wait", 0.1);
  addShare(msg, "system/gz::sim::v11::systems::Physics", 0.8);
  addShare(msg, "system/gz::sim::v11::systems::Sensors", 0.1);
  addShare(msg, "system/gz::sim::v11::systems::Slow@robot", 0.04);
  addShare(msg, "model/fidget_spinner/physics_estimate", 0.7);
  addShare(msg, "model/fidget_spinner/scene_estimate", 0.01);
  addShare(msg, "model/fidget_spinner/sensors", 0.0);
  addShare(msg, "model/fidget_spinner/plugins", 0.0);
  addShare(msg, "model/robot/physics_estimate", 0.1);
  addShare(msg, "model/robot/sensors", 0.4);
  addShare(msg, "model/robot/plugins", 0.04);
  addShare(msg, "model/idle_model/physics_estimate", 0.001);
  return msg;
}

/// \brief Records the entities selected through the main window
class SelectionListener : public QObject
{
  public: bool eventFilter(QObject *_obj, QEvent *_event) override
  {
    if (_event->type() == sim::gui::events::EntitiesSelected::kType)
    {
      auto *selected =
          static_cast<sim::gui::events::EntitiesSelected *>(_event);
      this->entities = selected->Data();
    }
    return QObject::eventFilter(_obj, _event);
  }

  public: std::vector<Entity> entities;
};
}  // namespace

/////////////////////////////////////////////////
TEST(PerformanceOverlayData, ReadsTheVerdictAndTheStep)
{
  const auto report = ParsePerformanceReport(physicsReport());

  EXPECT_DOUBLE_EQ(0.31, report.rtf);
  EXPECT_DOUBLE_EQ(1.0, report.desiredRtf);
  EXPECT_DOUBLE_EQ(5.0, report.windowSeconds);
  EXPECT_TRUE(report.limited);
  EXPECT_FALSE(report.softwareRenderer);
  EXPECT_EQ(1550u, report.iterations);
  EXPECT_EQ(60u, report.renderFrames);
  EXPECT_NEAR(0.8, report.physics, 1e-9);
  EXPECT_NEAR(0.1, report.renderWait, 1e-9);
  EXPECT_NEAR(0.04, report.otherSystems, 1e-9);
  EXPECT_NEAR(0.02, report.loop, 1e-9);
  EXPECT_NEAR(0.2, report.GpuWaitRatio(), 1e-9);

  EXPECT_EQ("Limited by the CPU", report.Headline());
  EXPECT_EQ("Physics: fidget_spinner (estimated)", report.Detail());
}

/////////////////////////////////////////////////
TEST(PerformanceOverlayData, WordsEveryResource)
{
  PerformanceOverlayReport report;
  report.limited = true;
  report.resource = "gpu";
  EXPECT_EQ("Limited by the GPU", report.Headline());
  report.resource = "cpu_and_gpu";
  EXPECT_EQ("Limited by CPU and GPU", report.Headline());
  report.resource = "external";
  EXPECT_EQ("Waiting on something external", report.Headline());
  report.resource = "unknown";
  EXPECT_EQ("Limited, resource unknown", report.Headline());
  report.limited = false;
  EXPECT_EQ("Keeping up", report.Headline());
}

/////////////////////////////////////////////////
TEST(PerformanceOverlayData, OnlyPhysicsIsCalledAnEstimate)
{
  PerformanceOverlayReport report;
  report.limited = true;
  report.location = "rendering";
  report.culprit = "camera";
  EXPECT_EQ("Rendering: camera", report.Detail());

  report.location = "system";
  report.culprit = "Slow";
  EXPECT_EQ("System: Slow", report.Detail());

  report.location = "engine";
  report.culprit = "";
  EXPECT_EQ("Simulation loop", report.Detail());

  report.limited = false;
  report.location = "physics";
  report.culprit = "spinner";
  EXPECT_EQ("Largest use is Physics: spinner (estimated)", report.Detail());

  report.location = "none";
  report.culprit = "";
  EXPECT_EQ("", report.Detail());
}

/////////////////////////////////////////////////
TEST(PerformanceOverlayData, GroupsTheEntriesOfEachModel)
{
  const auto report = ParsePerformanceReport(physicsReport());

  ASSERT_EQ(2u, report.models.size());
  EXPECT_EQ("fidget_spinner", report.models[0].name);
  EXPECT_NEAR(0.71, report.models[0].Total(), 1e-9);
  EXPECT_EQ("robot", report.models[1].name);
  EXPECT_NEAR(0.1, report.models[1].physics, 1e-9);
  EXPECT_NEAR(0.4, report.models[1].sensors, 1e-9);
  EXPECT_NEAR(0.04, report.models[1].plugins, 1e-9);
}

/////////////////////////////////////////////////
TEST(PerformanceOverlayData, ModelNamesCanHaveSlashes)
{
  msgs::Diagnostics msg;
  addShare(msg, "model/lab/robot/sensors", 0.2);
  addShare(msg, "model/lab/robot/physics_estimate", 0.1);

  const auto report = ParsePerformanceReport(msg);
  ASSERT_EQ(1u, report.models.size());
  EXPECT_EQ("lab/robot", report.models[0].name);
  EXPECT_NEAR(0.3, report.models[0].Total(), 1e-9);
}

/////////////////////////////////////////////////
TEST(PerformanceOverlayData, ReadsTheSensors)
{
  const auto report = ParsePerformanceReport(physicsReport());

  ASSERT_EQ(1u, report.sensors.size());
  EXPECT_EQ("robot::base::camera", report.sensors[0].name);
  EXPECT_EQ("robot::camera", report.sensors[0].Label());
  EXPECT_EQ("robot", report.sensors[0].Model());
  EXPECT_NEAR(0.4, report.sensors[0].share, 1e-9);
  EXPECT_NEAR(0.1, report.sensors[0].gpuShare, 1e-9);

  PerformanceOverlaySensor unscoped;
  unscoped.name = "camera";
  EXPECT_EQ("camera", unscoped.Label());
  EXPECT_EQ("", unscoped.Model());
}

/////////////////////////////////////////////////
TEST(PerformanceOverlayData, SystemsLeaveOutWhatHasItsOwnBar)
{
  const auto report = ParsePerformanceReport(physicsReport());

  ASSERT_EQ(1u, report.systems.size());
  EXPECT_EQ("Slow", report.systems[0].name);
}

/////////////////////////////////////////////////
TEST(PerformanceOverlayData, KeepsTheLargestRows)
{
  msgs::Diagnostics msg;
  addShare(msg, "sensor/a", 0.1);
  addShare(msg, "sensor/b", 0.4);
  addShare(msg, "sensor/c", 0.3);
  addShare(msg, "sensor/d", 0.2);
  addShare(msg, "sensor/e", 0.001);

  const auto report = ParsePerformanceReport(msg);
  ASSERT_EQ(kPerformanceOverlayRows, report.sensors.size());
  EXPECT_EQ("b", report.sensors[0].name);
  EXPECT_EQ("c", report.sensors[1].name);
  EXPECT_EQ("d", report.sensors[2].name);
}

/////////////////////////////////////////////////
TEST(PerformanceOverlayData, SurvivesAnEmptyMessage)
{
  msgs::Diagnostics msg;
  auto *entry = msg.add_time();
  entry->set_name("step/physics");

  const auto report = ParsePerformanceReport(msg);
  EXPECT_DOUBLE_EQ(0.0, report.physics);
  EXPECT_DOUBLE_EQ(0.0, report.GpuWaitRatio());
  EXPECT_FALSE(report.limited);
  EXPECT_TRUE(report.models.empty());
}

/////////////////////////////////////////////////
class PerformanceOverlayGui : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(PerformanceOverlayGui, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(Workflow))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  auto *runner = new sim::GuiRunner("perf_overlay_test");
  runner->setParent(gui::App());

  ASSERT_TRUE(app->LoadPlugin("PerformanceOverlay"));
  auto *win = app->findChild<gui::MainWindow *>();
  ASSERT_NE(nullptr, win);
  auto plugins = win->findChildren<sim::PerformanceOverlay *>();
  ASSERT_EQ(1, plugins.size());
  auto *plugin = plugins[0];

  EXPECT_EQ("Performance", plugin->Title());
  EXPECT_TRUE(plugin->Report().isEmpty());
  EXPECT_FALSE(plugin->Monitoring());

  // Stand in for the server
  transport::Node node;
  bool enableRequested{false};
  const std::function<bool(const msgs::Boolean &, msgs::Boolean &)> enable =
      [&enableRequested](const msgs::Boolean &_req, msgs::Boolean &_res)
      {
        enableRequested = _req.data();
        _res.set_data(_req.data());
        return true;
      };
  ASSERT_TRUE(node.Advertise("/world/perf_overlay_test/performance/enable",
      enable));
  auto pub = node.Advertise<msgs::Diagnostics>(
      "/world/perf_overlay_test/performance/report");

  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::Name("perf_overlay_test"));
  const Entity model = ecm.CreateEntity();
  ecm.CreateComponent(model, components::Model());
  ecm.CreateComponent(model, components::Name("fidget_spinner"));
  ecm.CreateComponent(model, components::ParentEntity(world));

  UpdateInfo info;
  info.paused = true;
  plugin->Update(info, ecm);
  EXPECT_TRUE(plugin->Paused());

  const auto waitFor = [](const std::function<bool()> &_condition)
  {
    for (int i = 0; i < 100 && !_condition(); ++i)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      QCoreApplication::processEvents();
    }
    return _condition();
  };

  // The panel starts the monitor
  EXPECT_TRUE(waitFor([&] { return plugin->Monitoring(); }));
  EXPECT_TRUE(enableRequested);
  EXPECT_FALSE(plugin->Notice().isEmpty());

  // Reports reach the panel
  ASSERT_TRUE(waitFor([&] { return pub.HasConnections(); }));
  EXPECT_TRUE(pub.Publish(physicsReport()));
  ASSERT_TRUE(waitFor([&] { return !plugin->Report().isEmpty(); }));
  const auto report = plugin->Report();
  EXPECT_EQ("Limited by the CPU", report["headline"].toString());
  EXPECT_EQ(2, report["models"].toList().size());
  EXPECT_EQ(1, plugin->History().size());
  EXPECT_TRUE(plugin->Notice().isEmpty());

  // The history doesn't grow forever
  for (int i = 0; i < 50; ++i)
    EXPECT_TRUE(pub.Publish(physicsReport()));
  EXPECT_TRUE(waitFor([&] { return plugin->History().size() == 40; }));

  // Clicking on a row selects the model
  SelectionListener listener;
  win->installEventFilter(&listener);
  plugin->SelectModel("not_a_model");
  plugin->Update(info, ecm);
  EXPECT_TRUE(listener.entities.empty());

  plugin->SelectModel("fidget_spinner");
  plugin->Update(info, ecm);
  EXPECT_EQ(std::vector<Entity>{model}, listener.entities);
  win->removeEventFilter(&listener);

  // Stopping it is up to the user
  plugin->SetMonitoring(false);
  EXPECT_TRUE(waitFor([&] { return !plugin->Monitoring(); }));
  EXPECT_FALSE(enableRequested);
  EXPECT_FALSE(plugin->Report().isEmpty());
}

/////////////////////////////////////////////////
TEST_F(PerformanceOverlayGui,
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(RefusedByTheServer))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  auto *runner = new sim::GuiRunner("perf_overlay_refused");
  runner->setParent(gui::App());

  ASSERT_TRUE(app->LoadPlugin("PerformanceOverlay"));
  auto *win = app->findChild<gui::MainWindow *>();
  ASSERT_NE(nullptr, win);
  auto *plugin = win->findChildren<sim::PerformanceOverlay *>()[0];

  transport::Node node;
  const std::function<bool(const msgs::Boolean &, msgs::Boolean &)> enable =
      [](const msgs::Boolean &, msgs::Boolean &_res)
      {
        _res.set_data(false);
        return true;
      };
  ASSERT_TRUE(node.Advertise("/world/perf_overlay_refused/performance/enable",
      enable));

  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::Name("perf_overlay_refused"));
  plugin->Update(UpdateInfo(), ecm);

  for (int i = 0; i < 100 && !plugin->Notice().contains("another world"); ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    QCoreApplication::processEvents();
  }
  EXPECT_TRUE(plugin->Notice().contains("another world"))
      << plugin->Notice().toStdString();
  EXPECT_FALSE(plugin->Monitoring());
}

/////////////////////////////////////////////////
TEST_F(PerformanceOverlayGui,
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(ServerWithoutMonitor))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);
  app->AddPluginPath(std::string(PROJECT_BINARY_PATH) + "/lib");

  auto *runner = new sim::GuiRunner("perf_overlay_absent");
  runner->setParent(gui::App());

  ASSERT_TRUE(app->LoadPlugin("PerformanceOverlay"));
  auto *win = app->findChild<gui::MainWindow *>();
  ASSERT_NE(nullptr, win);
  auto *plugin = win->findChildren<sim::PerformanceOverlay *>()[0];

  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::Name("perf_overlay_absent"));

  for (int i = 0; i < 100 && !plugin->Notice().contains("does not report"); ++i)
  {
    plugin->Update(UpdateInfo(), ecm);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    QCoreApplication::processEvents();
  }
  EXPECT_TRUE(plugin->Notice().contains("does not report"))
      << plugin->Notice().toStdString();
  EXPECT_FALSE(plugin->Monitoring());
}

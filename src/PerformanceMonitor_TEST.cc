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

#include <chrono>
#include <string>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/World.hh"

#include "PerformanceMonitor.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

namespace
{
const char kPhysics[] = "gz::sim::systems::Physics";
const char kSensors[] = "gz::sim::systems::Sensors";

/// \brief An interval that ran at half of its real time target.
PerformanceWindow slowWindow()
{
  PerformanceWindow window;
  window.wall = 10s;
  window.simTime = 5s;
  window.iterations = 5000;
  window.desiredRtf = 1.0;
  return window;
}

/// \brief Create a system sample
PerformanceSystemSample system(const std::string &_name,
    std::chrono::steady_clock::duration _critical,
    const std::string &_model = "")
{
  PerformanceSystemSample sample;
  sample.name = _name;
  sample.model = _model;
  sample.wall = _critical;
  sample.critical = _critical;
  return sample;
}

/// \brief Create a sensor sample
PerformanceSensorSample sensor(const std::string &_name,
    const std::string &_model, std::chrono::steady_clock::duration _total,
    std::chrono::steady_clock::duration _gpuWait)
{
  PerformanceSensorSample sample;
  sample.name = _name;
  sample.model = _model;
  sample.updates = 100;
  sample.total = _total;
  sample.gpuWait = _gpuWait;
  return sample;
}

/// \brief Create a render pass total
events::RenderFrameTimes renderTimes(
    std::chrono::steady_clock::duration _sensors,
    std::chrono::steady_clock::duration _gpuWait)
{
  events::RenderFrameTimes times;
  times.sensors = _sensors;
  times.gpuWait = _gpuWait;
  // By default the simulation thread was blocked for all of the render time
  times.simThreadWait = _sensors;
  return times;
}
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, NotLimited)
{
  PerformanceWindow window = slowWindow();
  window.simTime = 10s;
  window.sleep = 8s;
  window.systems.push_back(system(kPhysics, 1500ms));

  const auto report = AnalyzePerformance(window);
  EXPECT_DOUBLE_EQ(1.0, report.rtf);
  EXPECT_FALSE(report.limited);
  EXPECT_EQ(PerformanceResource::kNone, report.resource);
  // The biggest consumer is still named.
  EXPECT_EQ(PerformanceLocation::kPhysics, report.location);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, SmallPacingJitterIsNotALimit)
{
  PerformanceWindow window = slowWindow();
  window.simTime = 9800ms;
  window.systems.push_back(system(kPhysics, 9000ms));

  const auto report = AnalyzePerformance(window);
  EXPECT_FALSE(report.limited);
  EXPECT_EQ(PerformanceResource::kNone, report.resource);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, GpuLimitedSensor)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kPhysics, 1s));
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 300;
  window.render = renderTimes(9s, 8s);
  window.sensors.push_back(sensor("a::l::small", "a", 1s, 500ms));
  window.sensors.push_back(sensor("b::l::big", "b", 8s, 7500ms));

  const auto report = AnalyzePerformance(window);
  EXPECT_TRUE(report.limited);
  EXPECT_EQ(PerformanceResource::kGpu, report.resource);
  EXPECT_EQ(PerformanceLocation::kRendering, report.location);
  EXPECT_EQ("b::l::big", report.culprit);
  ASSERT_FALSE(report.models.empty());
  EXPECT_EQ("b", report.models.front().name);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, CpuLimitedRendering)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 300;
  window.render = renderTimes(9s, 1s);
  window.sensors.push_back(sensor("a::l::cam", "a", 9s, 1s));

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kCpu, report.resource);
  EXPECT_EQ(PerformanceLocation::kRendering, report.location);
  EXPECT_EQ("a::l::cam", report.culprit);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, SlowSceneUpdateIsNamed)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 300;
  window.render = renderTimes(1s, 100ms);
  window.render.sceneUpdate = 8s;
  window.render.simThreadWait = 8s;
  window.sensors.push_back(sensor("a::l::cam", "a", 1s, 100ms));

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kCpu, report.resource);
  EXPECT_EQ("scene update", report.culprit);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, ComparableGpuWaitAndCpuIsBoth)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 300;
  window.render = renderTimes(8s, 4s);
  window.sensors.push_back(sensor("a::l::cam", "a", 8s, 4s));

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kCpuAndGpu, report.resource);
  EXPECT_EQ("cpu_and_gpu", report.ResourceName());
  EXPECT_EQ("a::l::cam", report.culprit);
  EXPECT_NE(std::string::npos, report.ToString().find("CPU and GPU"));
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, HeavySceneNamesTheModelWithMostVisuals)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 300;
  window.render = renderTimes(1s, 100ms);
  window.render.preRender = 7s;
  window.render.simThreadWait = 8s;
  window.sensors.push_back(sensor("cam_model::l::cam", "cam_model", 1s, 100ms));

  PerformanceModelSample soup;
  soup.name = "soup";
  soup.visuals = 9000u;
  PerformanceModelSample camera;
  camera.name = "cam_model";
  camera.visuals = 1u;
  window.models = {camera, soup};

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kCpu, report.resource);
  EXPECT_EQ("scene update, mostly model soup", report.culprit);
  ASSERT_FALSE(report.models.empty());
  EXPECT_EQ("soup", report.models.front().name);
  EXPECT_GT(report.models.front().scene, 6s);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, SensorsSystemComputingIsCpu)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 10;
  // The little time that the render thread uses is mostly GPU wait, and the
  // simulation thread barely waits for it.
  window.render = renderTimes(100ms, 90ms);
  window.sensors.push_back(sensor("a::l::cam", "a", 100ms, 90ms));

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kCpu, report.resource);
  EXPECT_EQ(PerformanceLocation::kRendering, report.location);
  EXPECT_EQ("scene update", report.culprit);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, SoftwareRasterizerIsCpu)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 300;
  window.render = renderTimes(9s, 8s);
  window.render.softwareRenderer = true;
  window.sensors.push_back(sensor("a::l::cam", "a", 9s, 8s));

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kCpu, report.resource);
  EXPECT_EQ(PerformanceLocation::kRendering, report.location);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, EngineWithoutGpuWaitsIsUnknown)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 300;
  window.render = renderTimes(9s, 0s);

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kUnknown, report.resource);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, PhysicsNamesTheCostliestModel)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kPhysics, 9s));

  PerformanceModelSample ground;
  ground.name = "ground";
  ground.collisions = 1;
  window.models.push_back(ground);

  PerformanceModelSample pile;
  pile.name = "pile";
  pile.dof = 600;
  pile.collisions = 100;
  pile.contacts = 400.0;
  window.models.push_back(pile);

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kCpu, report.resource);
  EXPECT_EQ(PerformanceLocation::kPhysics, report.location);
  EXPECT_EQ("pile", report.culprit);
  ASSERT_EQ(2u, report.models.size());
  EXPECT_EQ("pile", report.models.front().name);
  EXPECT_GT(report.models.front().physics, report.models.back().physics);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, PhysicsSpreadOverModelsNamesNoModel)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kPhysics, 9s));

  for (const char *name : {"a", "b", "c"})
  {
    PerformanceModelSample model;
    model.name = name;
    model.dof = 6;
    model.collisions = 1;
    window.models.push_back(model);
  }
  window.models[0].dof = 12;

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceLocation::kPhysics, report.location);
  EXPECT_EQ("no model dominates, largest is a at 47%", report.culprit);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, NestedStaticModelDoesNotMove)
{
  EntityComponentManager ecm;
  const auto addChild = [&ecm](Entity _parent)
  {
    const Entity entity = ecm.CreateEntity();
    ecm.SetParentEntity(entity, _parent);
    ecm.CreateComponent(entity, components::ParentEntity(_parent));
    return entity;
  };

  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());

  // A model that can move with a static model nested inside of it, like a
  // conveyor assembled from static blocks.
  const Entity outer = addChild(world);
  ecm.CreateComponent(outer, components::Model());
  ecm.CreateComponent(outer, components::Name("conveyor"));
  const Entity freeLink = addChild(outer);
  ecm.CreateComponent(freeLink, components::Link());
  const Entity freeCollision = addChild(freeLink);
  ecm.CreateComponent(freeCollision, components::Collision());

  const Entity nested = addChild(outer);
  ecm.CreateComponent(nested, components::Model());
  ecm.CreateComponent(nested, components::Name("block"));
  ecm.CreateComponent(nested, components::Static(true));
  const Entity fixedLink = addChild(nested);
  ecm.CreateComponent(fixedLink, components::Link());
  const Entity fixedCollision = addChild(fixedLink);
  ecm.CreateComponent(fixedCollision, components::Collision());

  const auto models = CollectPerformanceModels(ecm,
      {{freeCollision, 3.0}, {fixedCollision, 40.0}});
  ASSERT_EQ(1u, models.size());
  EXPECT_EQ("conveyor", models[0].name);
  EXPECT_EQ(6u, models[0].dof);
  EXPECT_EQ(2u, models[0].collisions);
  EXPECT_DOUBLE_EQ(3.0, models[0].contacts);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, LinkWeldedToTheWorldDoesNotMove)
{
  EntityComponentManager ecm;
  const auto addChild = [&ecm](Entity _parent)
  {
    const Entity entity = ecm.CreateEntity();
    ecm.SetParentEntity(entity, _parent);
    ecm.CreateComponent(entity, components::ParentEntity(_parent));
    return entity;
  };

  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());

  const Entity model = addChild(world);
  ecm.CreateComponent(model, components::Model());
  ecm.CreateComponent(model, components::Name("belt"));
  const Entity link = addChild(model);
  ecm.CreateComponent(link, components::Link());
  ecm.CreateComponent(link, components::Name("base"));
  const Entity collision = addChild(link);
  ecm.CreateComponent(collision, components::Collision());
  const Entity joint = addChild(model);
  ecm.CreateComponent(joint, components::Joint());
  ecm.CreateComponent(joint, components::JointType(sdf::JointType::FIXED));
  ecm.CreateComponent(joint, components::ParentLinkName("world"));
  ecm.CreateComponent(joint, components::ChildLinkName("base"));

  const auto models = CollectPerformanceModels(ecm, {{collision, 48.0}});
  ASSERT_EQ(1u, models.size());
  EXPECT_EQ(0u, models[0].dof);
  EXPECT_EQ(1u, models[0].collisions);
  EXPECT_DOUBLE_EQ(0.0, models[0].contacts);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, MovingMeshInContactOutweighsStaticGeometry)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kPhysics, 9s));

  PerformanceModelSample spinner;
  spinner.name = "spinner";
  spinner.dof = 1;
  spinner.collisions = 5;
  spinner.contacts = 4.0;
  spinner.contactingMeshTriangles = 7724;
  window.models.push_back(spinner);

  PerformanceModelSample house;
  house.name = "house";
  house.collisions = 175;
  window.models.push_back(house);

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ("spinner", report.culprit);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, SlowPluginOnModel)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kPhysics, 1s));
  auto slow = system("my::SlowPlugin", 8s, "robot");
  slow.sampledWall = 500ms;
  slow.sampledCpu = 495ms;
  window.systems.push_back(slow);

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kCpu, report.resource);
  EXPECT_EQ(PerformanceLocation::kSystem, report.location);
  EXPECT_EQ("my::SlowPlugin on model robot", report.culprit);
  ASSERT_FALSE(report.models.empty());
  EXPECT_EQ("robot", report.models.front().name);
  EXPECT_EQ(8s, report.models.front().plugins);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, BlockedPluginIsExternal)
{
  PerformanceWindow window = slowWindow();
  auto blocked = system("my::LockStep", 9s);
  blocked.sampledWall = 500ms;
  blocked.sampledCpu = 10ms;
  window.systems.push_back(blocked);

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kExternal, report.resource);
  EXPECT_EQ(PerformanceLocation::kSystem, report.location);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, TimeOutsideOfSystemsIsTheLoop)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kPhysics, 1s));

  const auto report = AnalyzePerformance(window);
  EXPECT_EQ(PerformanceResource::kCpu, report.resource);
  EXPECT_EQ(PerformanceLocation::kEngine, report.location);
  EXPECT_EQ(9s, report.engine);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, UnthrottledAlwaysHasABottleneck)
{
  PerformanceWindow window = slowWindow();
  window.desiredRtf = 0.0;
  window.simTime = 300s;
  window.systems.push_back(system(kPhysics, 9s));

  const auto report = AnalyzePerformance(window);
  EXPECT_TRUE(report.limited);
  EXPECT_EQ(PerformanceResource::kCpu, report.resource);
  EXPECT_EQ(PerformanceLocation::kPhysics, report.location);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, MessageCarriesTheVerdict)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 300;
  window.render = renderTimes(9s, 8s);
  window.sensors.push_back(sensor("a::l::cam", "a", 9s, 8s));

  const auto msg = AnalyzePerformance(window).ToMsg();
  EXPECT_DOUBLE_EQ(0.5, msg.real_time_factor());

  std::string resource;
  std::string location;
  std::string culprit;
  for (const auto &data : msg.header().data())
  {
    ASSERT_EQ(1, data.value_size());
    if (data.key() == "resource")
      resource = data.value(0);
    else if (data.key() == "location")
      location = data.value(0);
    else if (data.key() == "culprit")
      culprit = data.value(0);
  }
  EXPECT_EQ("gpu", resource);
  EXPECT_EQ("rendering", location);
  EXPECT_EQ("a::l::cam", culprit);

  bool sensorFound = false;
  for (const auto &time : msg.time())
    sensorFound = sensorFound || time.name() == "sensor/a::l::cam";
  EXPECT_TRUE(sensorFound);
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, TextNamesTheBottleneck)
{
  PerformanceWindow window = slowWindow();
  window.systems.push_back(system(kSensors, 8s));
  window.renderFrames = 300;
  window.render = renderTimes(9s, 8s);
  window.sensors.push_back(sensor("a::l::cam", "a", 9s, 8s));

  const auto text = AnalyzePerformance(window).ToString();
  EXPECT_NE(std::string::npos, text.find("Bottleneck: GPU")) << text;
  EXPECT_NE(std::string::npos, text.find("a::l::cam")) << text;
}

/////////////////////////////////////////////////
namespace
{
/// \brief Run an iteration of the monitor
/// \param[in, out] _monitor The monitor
/// \param[in] _wall Wall time at the end of the iteration
/// \param[in] _simTime Simulation time
void iterate(PerformanceMonitor &_monitor,
    const std::chrono::steady_clock::time_point &_wall,
    std::chrono::steady_clock::duration _simTime)
{
  UpdateInfo info;
  info.simTime = _simTime;
  info.dt = 1ms;
  info.paused = false;
  _monitor.EndIteration(_wall, _wall, info, 1.0);
}
}  // namespace

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, OnlyOneMonitorAtATime)
{
  EXPECT_FALSE(PerformanceMonitorEnabled());

  auto first = PerformanceMonitor::Create(1s, false);
  ASSERT_NE(nullptr, first);
  EXPECT_TRUE(PerformanceMonitorEnabled());
  EXPECT_EQ(nullptr, PerformanceMonitor::Create(1s, false));
  EXPECT_TRUE(PerformanceMonitorEnabled());

  first.reset();
  EXPECT_FALSE(PerformanceMonitorEnabled());

  EXPECT_NE(nullptr, PerformanceMonitor::Create(1s, false));
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, WorldResetStartsOver)
{
  const auto start = std::chrono::steady_clock::now();
  auto monitor = PerformanceMonitor::Create(1s, false);
  ASSERT_NE(nullptr, monitor);

  // The warm up, which starts with the first iteration, and then an interval
  iterate(*monitor, start + 1s, 1s);
  iterate(*monitor, start + 3s, 3s);
  iterate(*monitor, start + 5s, 5s);
  EXPECT_TRUE(monitor->ReportDue());

  // The simulation time goes back. That can't be used for an interval.
  iterate(*monitor, start + 6s, 1ms);
  EXPECT_FALSE(monitor->ReportDue());

  iterate(*monitor, start + 9s, 3s);
  EXPECT_FALSE(monitor->ReportDue());
  iterate(*monitor, start + 11s, 5s);
  EXPECT_TRUE(monitor->ReportDue());
}

/////////////////////////////////////////////////
TEST(PerformanceMonitorTest, TimeBetweenRunsIsNotCounted)
{
  const auto start = std::chrono::steady_clock::now();
  auto monitor = PerformanceMonitor::Create(1s, false);
  ASSERT_NE(nullptr, monitor);

  iterate(*monitor, start + 1s, 1s);
  iterate(*monitor, start + 3s, 3s);
  iterate(*monitor, start + 3100ms, 3100ms);
  EXPECT_FALSE(monitor->ReportDue());

  // A long time without running doesn't complete the interval.
  monitor->StartRun(start + 100s);
  iterate(*monitor, start + 100s + 100ms, 3200ms);
  EXPECT_FALSE(monitor->ReportDue());

  // Running does.
  iterate(*monitor, start + 101s, 4100ms);
  EXPECT_TRUE(monitor->ReportDue());
}

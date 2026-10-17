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

// Times the DiffDrive system alone. The system is called directly, with no
// server, physics or other system, so the numbers are the cost of
// PreUpdate and PostUpdate for N robots. A stand-in for the physics system
// consumes the wheel velocity commands and writes the wheel positions between
// the two phases, as a physics engine does, outside the timed regions.

#include <benchmark/benchmark.h>

#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/transport/Node.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"

#include "DiffDrive.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief SDF world with _count differential drive robots.
std::string WorldSdf(int _count)
{
  std::ostringstream out;
  out << "<sdf version='1.10'><world name='diff_drive_bench'>";
  for (int i = 0; i < _count; ++i)
  {
    out << "<model name='robot_" << i << "'>"
        << "<link name='base'/>"
        << "<link name='left_wheel'/>"
        << "<link name='right_wheel'/>"
        << "<joint name='left_joint' type='revolute'>"
        << "<parent>base</parent><child>left_wheel</child>"
        << "<axis><xyz>0 1 0</xyz></axis></joint>"
        << "<joint name='right_joint' type='revolute'>"
        << "<parent>base</parent><child>right_wheel</child>"
        << "<axis><xyz>0 1 0</xyz></axis></joint>"
        << "<plugin name='gz::sim::systems::DiffDrive'"
        << " filename='gz-sim-diff-drive-system'>"
        << "<left_joint>left_joint</left_joint>"
        << "<right_joint>right_joint</right_joint>"
        << "<wheel_separation>0.4</wheel_separation>"
        << "<wheel_radius>0.1</wheel_radius>"
        << "<odom_publish_frequency>50</odom_publish_frequency>"
        << "</plugin></model>";
  }
  out << "</world></sdf>";
  return out.str();
}
}  // namespace

/// \brief Step DiffDrive for N robots. Range 0 is the robot count, range 1
/// is 1 when every odometry and tf topic has a subscriber. Range 2 is how the
/// stand-in physics system clears the joint velocity commands after reading
/// them: 0 sets them to zero (the simplified physics system), 1 removes the
/// components (the Physics system).
static void BM_DiffDriveStep(benchmark::State &_st)
{
  const int count = static_cast<int>(_st.range(0));
  const bool subscribed = _st.range(1) != 0;
  const bool removeCommands = _st.range(2) != 0;

  sdf::Root root;
  if (!root.LoadSdfString(WorldSdf(count)).empty())
  {
    _st.SkipWithError("Failed to load SDF");
    return;
  }
  const sdf::World *world = root.WorldByIndex(0);

  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);
  creator.CreateEntities(world);

  std::vector<std::unique_ptr<systems::DiffDrive>> systems;
  std::vector<Entity> leftJoints;
  std::vector<Entity> rightJoints;
  std::vector<std::string> names;
  for (unsigned int i = 0; i < world->ModelCount(); ++i)
  {
    const sdf::Model *modelSdf = world->ModelByIndex(i);
    const Entity model = ecm.EntityByComponents(
        components::Name(modelSdf->Name()), components::Model());
    if (model == kNullEntity)
    {
      _st.SkipWithError("Model entity not found");
      return;
    }
    names.push_back(modelSdf->Name());
    leftJoints.push_back(ecm.EntityByComponents(
        components::Name("left_joint"), components::ParentEntity(model)));
    rightJoints.push_back(ecm.EntityByComponents(
        components::Name("right_joint"), components::ParentEntity(model)));
    systems.push_back(std::make_unique<systems::DiffDrive>());
    systems.back()->Configure(model, modelSdf->Plugins()[0].Element(), ecm,
        eventMgr);
  }

  transport::Node node;
  std::atomic<int> received{0};
  std::function<void(const msgs::Odometry &)> odomCb =
      [&received](const msgs::Odometry &) { ++received; };
  std::function<void(const msgs::Pose_V &)> tfCb =
      [&received](const msgs::Pose_V &) { ++received; };
  if (subscribed)
  {
    for (const auto &name : names)
    {
      node.Subscribe("/model/" + name + "/odometry", odomCb);
      node.Subscribe("/model/" + name + "/tf", tfCb);
    }
  }

  // Constant forward command with a turn, as a navigation stack sends.
  msgs::Twist cmd;
  cmd.mutable_linear()->set_x(0.5);
  cmd.mutable_angular()->set_z(0.1);
  for (const auto &name : names)
  {
    auto pub = node.Advertise<msgs::Twist>("/model/" + name + "/cmd_vel");
    // Let the subscription handshake finish.
    for (int attempt = 0; attempt < 200 && !pub.HasConnections(); ++attempt)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    pub.Publish(cmd);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  UpdateInfo info;
  info.dt = std::chrono::milliseconds(1);
  info.paused = false;

  using Clock = std::chrono::steady_clock;
  double position = 0.0;
  double preSeconds = 0.0;
  double postSeconds = 0.0;
  auto step = [&](bool _timed) -> double
  {
    info.iterations++;
    info.simTime += info.dt;
    info.realTime += info.dt;

    const auto t0 = Clock::now();
    for (auto &system : systems)
      system->PreUpdate(info, ecm);
    const auto t1 = Clock::now();

    // Stand-in for the physics system: consume the velocity commands and
    // turn the wheels, so the odometry has input.
    position += 0.001;
    for (unsigned int i = 0; i < leftJoints.size(); ++i)
    {
      for (Entity joint : {leftJoints[i], rightJoints[i]})
      {
        if (removeCommands)
        {
          ecm.RemoveComponent<components::JointVelocityCmd>(joint);
        }
        else if (auto velCmd =
                 ecm.Component<components::JointVelocityCmd>(joint))
        {
          std::fill(velCmd->Data().begin(), velCmd->Data().end(), 0.0);
        }
      }
      if (ecm.Component<components::JointPosition>(leftJoints[i]))
      {
        ecm.SetComponentData<components::JointPosition>(
            leftJoints[i], {position});
        ecm.SetComponentData<components::JointPosition>(
            rightJoints[i], {position});
      }
    }

    const auto t2 = Clock::now();
    for (auto &system : systems)
      system->PostUpdate(info, ecm);
    const auto t3 = Clock::now();

    // End of step bookkeeping, as in the simulation runner.
    ecm.ClearNewlyCreatedEntities();
    ecm.SetAllComponentsUnchanged();

    if (!_timed)
      return 0.0;
    preSeconds += std::chrono::duration<double>(t1 - t0).count();
    postSeconds += std::chrono::duration<double>(t3 - t2).count();
    return std::chrono::duration<double>((t1 - t0) + (t3 - t2)).count();
  };

  // Warm up: the first steps create components and initialize odometry.
  for (int i = 0; i < 100; ++i)
    step(false);

  // The stand-in physics clears the commands every step, so run PreUpdate
  // once more and look at what DiffDrive wrote.
  systems[0]->PreUpdate(info, ecm);
  auto leftVel = ecm.Component<components::JointVelocityCmd>(leftJoints[0]);
  if (!leftVel || leftVel->Data().empty() ||
      std::abs(leftVel->Data()[0]) < 1e-9)
  {
    _st.SkipWithError("DiffDrive did not receive the command");
    return;
  }

  for (auto _ : _st)
    _st.SetIterationTime(step(true));

  _st.counters["robots"] = count;
  _st.counters["ns_per_robot"] = benchmark::Counter(
      static_cast<double>(count),
      benchmark::Counter::kIsIterationInvariantRate |
      benchmark::Counter::kInvert, benchmark::Counter::kIs1000);
  const double steps = static_cast<double>(_st.iterations());
  _st.counters["pre_ns_per_robot"] = preSeconds / steps / count * 1e9;
  _st.counters["post_ns_per_robot"] = postSeconds / steps / count * 1e9;
  _st.counters["msgs"] = received.load();
}

BENCHMARK(BM_DiffDriveStep)
    ->Args({100, 0, 0})
    ->Args({500, 0, 0})
    ->Args({500, 0, 1})
    ->Args({500, 1, 0})
    ->UseManualTime()
    ->Iterations(3000)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();

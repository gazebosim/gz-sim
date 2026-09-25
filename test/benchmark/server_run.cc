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

#include <benchmark/benchmark.h>
#include <cmath>
#include <cstring>
#include <vector>

#include <gz/common/Util.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/joint_trajectory.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"

#include "test_config.hh"
#include "../helpers/Relay.hh"

using namespace gz;
using namespace sim;
using namespace components;

ServerConfig getServerConfig(const std::string &_physics_engine,
                             const std::string &_world_sdf)
{
  std::string path = common::joinPaths(std::string(PROJECT_SOURCE_PATH), "/test/worlds/models");
  common::setenv("GZ_SIM_RESOURCE_PATH", path.c_str());
  ServerConfig serverConfig;
  serverConfig.SetWaitForAssets(true);
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
                                            "test/worlds/", _world_sdf));
  serverConfig.SetPhysicsEngine(_physics_engine);

  return serverConfig;
}

void BM_RuntimeWorld(benchmark::State &_st, const std::string &_physics_engine,
                     const std::string &_world_sdf)
{
  auto stabilizingSteps = _st.range(0);
  ServerConfig serverConfig { getServerConfig(_physics_engine, _world_sdf) };
  sim::Server server(serverConfig); // Add system from plugin
  // Wait for simulation to stabilize before timing
  server.Run(true, stabilizingSteps, false);

  for (auto _ : _st)
  {
    server.Run(true, 1, false);
  }
}

void BM_RuntimeWorldContacts(benchmark::State &_st, const std::string &_physics_engine,
                             const std::string &_world_sdf)
{
  auto stabilizingSteps = _st.range(0);
  ServerConfig serverConfig { getServerConfig(_physics_engine, _world_sdf) };
  bool contactsEnabled = false;

  // Instantiate the relay helper
  test::Relay relaySystem;
  // Register a callback to access the ECM in PreUpdate
  relaySystem.OnPreUpdate([&](const sim::UpdateInfo &/*_info*/,
                              sim::EntityComponentManager &_ecm)
  {
    // If contacts have been enabled, don't try enabling again
    if (contactsEnabled)
    {
      return;
    }
    // Iterate over all Collision entities
    _ecm.Each<components::Collision>(
      [&](const Entity &_entity, const components::Collision *) -> bool
      {
        // Check if ContactSensorData has already been created
        if (_ecm.EntityHasComponentType(_entity,
              components::ContactSensorData::typeId))
        {
          return true;
        }
        // Enable collision by creating the ContactSensorData component.
        _ecm.CreateComponent(_entity, components::ContactSensorData());
        return true;
      });
    contactsEnabled = true;
  });
  sim::Server server(serverConfig);
  // Wait for simulation to stabilize before adding contacts
  server.Run(true, stabilizingSteps, false);
  // Add the relay system to the server
  server.AddSystem(relaySystem.systemPtr);

  for (auto _ : _st)
  {
    server.Run(true, 1, false);
  }
}

void BM_LoadWorld(benchmark::State &_st, const std::string &_physics_engine,
                     const std::string &_world_sdf)
{
  ServerConfig serverConfig { getServerConfig(_physics_engine, _world_sdf) };
  for (auto _ : _st)
  {
    // Add system from plugin
    sim::Server server(serverConfig);
    // Run one step
    server.Run(true, 1, false);
  }
}

void BM_MobileRobot(benchmark::State &_st, const std::string &_physics_engine,
                    const std::string &_world_sdf)
{
  ServerConfig serverConfig { getServerConfig(_physics_engine, _world_sdf) };
  sim::Server server(serverConfig);

  transport::Node node;
  const std::string trajectoryTopic = "/model/RR_position_control/joint_trajectory";
  auto cmdVelPublisher1 = node.Advertise<msgs::Twist>("/model/vehicle_1/cmd_vel");
  auto cmdVelPublisher2 = node.Advertise<msgs::Twist>("/model/vehicle_2/cmd_vel");
  auto trajectoryPublisher = node.Advertise<msgs::JointTrajectory>(trajectoryTopic);

  // Set up command velocity messages to mobile robots
  double desiredLinVel = 1.0;
  double desiredAngVel = 0.2;
  msgs::Twist cmdVelMsg;
  msgs::Set(
    cmdVelMsg.mutable_linear(), math::Vector3d(desiredLinVel, 0, 0));
  msgs::Set(
    cmdVelMsg.mutable_angular(), math::Vector3d(0.0, 0, desiredAngVel));

  // Set up joint trajectory messages
  const size_t kNumberOfJoints = 2;
  const std::string jointNames[kNumberOfJoints] = {"RR_position_control_joint1",
                                                  "RR_position_control_joint2"};

  std::vector<std::array<int, 2>> trajectoryTimes;
  std::vector<std::array<double, kNumberOfJoints>> trajectoryPositions;

  // Generate 1000 trajectory points programmatically
  int numPoints = 1000;
  for (int i = 1; i <= numPoints; ++i)
  {
    double t = i * 0.5;
    int sec = static_cast<int>(t);
    int nsec = static_cast<int>((t - sec) * 1e9);
    trajectoryTimes.push_back({sec, nsec});

    // Oscillating positions over time
    double pos1 = std::sin(t);
    double pos2 = std::cos(t);
    trajectoryPositions.push_back({pos1, pos2});
  }

  // Create new JointTrajectory message based on the defined trajectory
  msgs::JointTrajectory joint_traj_msg;
  for (const auto &jointName : jointNames)
  {
    joint_traj_msg.add_joint_names(jointName);
  }
  for (size_t i = 0; i < trajectoryPositions.size(); ++i)
  {
    msgs::JointTrajectoryPoint point;

    // Set the temporal information for the point
    auto time = point.mutable_time_from_start();
    time->set_sec(trajectoryTimes[i][0]);
    time->set_nsec(trajectoryTimes[i][1]);

    // Add target positions to the point
    for (size_t j = 0; j < kNumberOfJoints; ++j)
    {
      point.add_positions(trajectoryPositions[i][j]);
    }

    // Add point to the trajectory
    joint_traj_msg.add_points();
    joint_traj_msg.mutable_points(i)->CopyFrom(point);
  }

  cmdVelPublisher1.Publish(cmdVelMsg);
  cmdVelPublisher2.Publish(cmdVelMsg);
  trajectoryPublisher.Publish(joint_traj_msg);

  for (auto _ : _st)
  {
    server.Run(true, 1, false);
  }
}

/*
The benchmark for 3k_shapes.sdf can take a while to run. Use the
'--benchmark_filter="(/sdf)"' argument when you run
build/gz-sim/bin/BENCHMARK_server_run if you want to exclude this
benchmark.
*/

/* Benchmark runtime performance */
BENCHMARK_CAPTURE(BM_RuntimeWorld, sdf_shapes_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "shapes.sdf")
    ->Arg(10)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_RuntimeWorld, sdf_shapes_dart,
                  "gz-physics-dartsim-plugin",
                  "shapes.sdf")
    ->Arg(1) // dartsim does not need warmup (no island sleep function for now)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_RuntimeWorld, sdf_gpu_lidar_sensor_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "gpu_lidar_sensor.sdf")
    ->Arg(10)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_RuntimeWorld, sdf_gpu_lidar_sensor_dart,
                  "gz-physics-dartsim-plugin",
                  "gpu_lidar_sensor.sdf")
    ->Arg(1) // dartsim does not need warmup (no island sleep function for now)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_RuntimeWorld, lengthy_sdf_3k_shapes_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "3k_shapes.sdf")
    ->Arg(3000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_RuntimeWorld, lengthy_sdf_3k_shapes_dart,
                  "gz-physics-dartsim-plugin",
                  "3k_shapes.sdf")
    ->Arg(1) // dartsim does not need warmup (no island sleep function for now)
    ->Unit(benchmark::kMillisecond);

/* Benchmark runtime with contacts performance */
BENCHMARK_CAPTURE(BM_RuntimeWorldContacts, sdf_shapes_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "shapes.sdf")
    ->Arg(10)
    ->Unit(benchmark::kNanosecond);

BENCHMARK_CAPTURE(BM_RuntimeWorldContacts, sdf_shapes_dart,
                  "gz-physics-dartsim-plugin",
                  "shapes.sdf")
    ->Arg(1)
    ->Unit(benchmark::kNanosecond);

BENCHMARK_CAPTURE(BM_RuntimeWorldContacts, sdf_gpu_lidar_sensor_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "gpu_lidar_sensor.sdf")
    ->Arg(10)
    ->Unit(benchmark::kNanosecond);

BENCHMARK_CAPTURE(BM_RuntimeWorldContacts, sdf_gpu_lidar_sensor_dart,
                  "gz-physics-dartsim-plugin",
                  "gpu_lidar_sensor.sdf")
    ->Arg(1)
    ->Unit(benchmark::kNanosecond);


BENCHMARK_CAPTURE(BM_RuntimeWorldContacts, lengthy_sdf_3k_shapes_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "3k_shapes.sdf")
    ->Arg(3000)
    ->Iterations(1000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_RuntimeWorldContacts, lengthy_sdf_3k_shapes_dart,
                  "gz-physics-dartsim-plugin",
                  "3k_shapes.sdf")
    ->Arg(1)
    ->Iterations(1000)
    ->Unit(benchmark::kMillisecond);

/* Benchmark load time */
BENCHMARK_CAPTURE(BM_LoadWorld, sdf_shapes_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "shapes.sdf")
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_LoadWorld, sdf_shapes_dart,
                  "gz-physics-dartsim-plugin",
                  "shapes.sdf")
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_LoadWorld, sdf_gpu_lidar_sensor_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "gpu_lidar_sensor.sdf")
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_LoadWorld, sdf_gpu_lidar_sensor_dart,
                  "gz-physics-dartsim-plugin",
                  "gpu_lidar_sensor.sdf")
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_LoadWorld, lengthy_sdf_3k_shapes_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "3k_shapes.sdf")
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_LoadWorld, lengthy_sdf_3k_shapes_dart,
                  "gz-physics-dartsim-plugin",
                  "3k_shapes.sdf")
    ->Unit(benchmark::kMillisecond);

/* Benchmark runtime for world with sensors and moving robots */
BENCHMARK_CAPTURE(BM_MobileRobot, sdf_moving_robots_and_sensors_dart,
                  "gz-physics-dartsim-plugin",
                  "moving_robots_and_sensors.sdf")
    ->Iterations(10000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_CAPTURE(BM_MobileRobot, sdf_moving_robots_and_sensors_bullet,
                  "gz-physics-bullet-featherstone-plugin",
                  "moving_robots_and_sensors.sdf")
    ->Iterations(10000)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();

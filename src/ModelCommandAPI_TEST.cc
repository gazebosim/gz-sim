/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <cstdio>
#include <cstdlib>
#include <string>

#include <gtest/gtest.h>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"  // NOLINT(build/include)

static const std::string kGzModelCommand(
    std::string(BREW_RUBY) + std::string(GZ_PATH) + " model ");

/////////////////////////////////////////////////
/// \brief Used to avoid the cases where the zero is
/// represented as a negative number.
/// \param _text Output string that may have negative zero values.
void ReplaceNegativeZeroValues(std::string &_text)
{
  std::string neg_zero{"-0.000000"};
  std::string zero{"0.000000"};
  size_t pos = 0;
  while ((pos = _text.find(neg_zero, pos)) != std::string::npos)
  {
    _text.replace(pos, neg_zero.length(), zero);
    pos += zero.length();
  }
}

/////////////////////////////////////////////////
std::string customExecStr(std::string _cmd)
{
  std::cout << "Running command [" << _cmd << "]" << std::endl;

  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != nullptr)
    {
      result += buffer;
    }
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
// Test `gz model` command when no Gazebo server is running.
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST(ModelCommandAPI, GZ_UTILS_TEST_DISABLED_ON_WIN32(NoServerRunning))
{
  const std::string cmd = kGzModelCommand + "--list ";
  const std::string output = customExecStr(cmd);
  const std::string expectedOutput =
        "\nService call to [/gazebo/worlds] timed out\n"
        "Command failed when trying to get the world name "
        "of the running simulation.\n";
  EXPECT_EQ(expectedOutput, output);
}

/////////////////////////////////////////////////
// Tests `gz model` command.
TEST(ModelCommandAPI, GZ_UTILS_TEST_DISABLED_ON_WIN32(Commands))
{
  gz::sim::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "static_diff_drive_vehicle.sdf"));

  gz::sim::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: gz model --list
  {
    const std::string cmd = kGzModelCommand + "--list";
    const std::string output = customExecStr(cmd);
    const std::string expectedOutput =
      "\nRequesting state for world [diff_drive]..."
      "\n\nAvailable models:\n"
      "    - ground_plane\n"
      "    - vehicle_blue\n";
    EXPECT_EQ(expectedOutput, output);
  }

  // Tested command: gz model -m vehicle_blue
  {
    const std::string cmd = kGzModelCommand + "-m vehicle_blue";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [diff_drive]...\n\n"
      "Model: [8]\n"
      "  - Name: vehicle_blue\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 2.000000 0.325000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Link [9]\n"
      "    - Name: chassis\n"
      "    - Parent: vehicle_blue [8]\n"
      "    - Mass (kg): 1.143950\n"
      "    - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [0.000000 0.000000 0.000000]\n"
      "      [0.000000 0.000000 0.000000]\n"
      "    - Inertial Matrix (kg.m^2):\n"
      "      [0.126164 0.000000 0.000000]\n"
      "      [0.000000 0.416519 0.000000]\n"
      "      [0.000000 0.000000 0.481014]\n"
      "    - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [-0.151427 0.000000 0.175000]\n"
      "      [0.000000 0.000000 0.000000]\n"
      "  - Link [12]\n"
      "    - Name: left_wheel\n"
      "    - Parent: vehicle_blue [8]\n"
      "    - Mass (kg): 2.000000\n"
      "    - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [0.000000 0.000000 0.000000]\n"
      "      [0.000000 0.000000 0.000000]\n"
      "    - Inertial Matrix (kg.m^2):\n"
      "      [0.145833 0.000000 0.000000]\n"
      "      [0.000000 0.145833 0.000000]\n"
      "      [0.000000 0.000000 0.125000]\n"
      "    - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [0.554283 0.625029 -0.025000]\n"
      "      [-1.570700 0.000000 0.000000]\n"
      "  - Link [15]\n"
      "    - Name: right_wheel\n"
      "    - Parent: vehicle_blue [8]\n"
      "    - Mass (kg): 2.000000\n"
      "    - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [0.000000 0.000000 0.000000]\n"
      "      [0.000000 0.000000 0.000000]\n"
      "    - Inertial Matrix (kg.m^2):\n"
      "      [0.145833 0.000000 0.000000]\n"
      "      [0.000000 0.145833 0.000000]\n"
      "      [0.000000 0.000000 0.125000]\n"
      "    - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [0.554282 -0.625029 -0.025000]\n"
      "      [-1.570700 0.000000 0.000000]\n"
      "  - Link [18]\n"
      "    - Name: caster\n"
      "    - Parent: vehicle_blue [8]\n"
      "    - Mass (kg): 1.000000\n"
      "    - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [0.000000 0.000000 0.000000]\n"
      "      [0.000000 0.000000 0.000000]\n"
      "    - Inertial Matrix (kg.m^2):\n"
      "      [0.100000 0.000000 0.000000]\n"
      "      [0.000000 0.100000 0.000000]\n"
      "      [0.000000 0.000000 0.100000]\n"
      "    - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [-0.957138 0.000000 -0.125000]\n"
      "      [0.000000 0.000000 0.000000]\n"
      "  - Joint [21]\n"
      "    - Name: left_wheel_joint\n"
      "    - Parent: vehicle_blue [8]\n"
      "    - Type: revolute\n"
      "    - Parent Link: chassis [9]\n"
      "    - Child Link: left_wheel [12]\n"
      "    - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [0.000000 0.000000 0.000000]\n"
      "      [0.000000 0.000000 0.000000]\n"
      "    - Axis unit vector [ XYZ ]:\n"
      "      [0 0 1]\n"
      "  - Joint [22]\n"
      "    - Name: right_wheel_joint\n"
      "    - Parent: vehicle_blue [8]\n"
      "    - Type: revolute\n"
      "    - Parent Link: chassis [9]\n"
      "    - Child Link: right_wheel [15]\n"
      "    - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [0.000000 0.000000 0.000000]\n"
      "      [0.000000 0.000000 0.000000]\n"
      "    - Axis unit vector [ XYZ ]:\n"
      "      [0 0 1]\n"
      "  - Joint [23]\n"
      "    - Name: caster_wheel\n"
      "    - Parent: vehicle_blue [8]\n"
      "    - Type: ball\n"
      "    - Parent Link: chassis [9]\n"
      "    - Child Link: caster [18]\n"
      "    - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "      [0.000000 0.000000 0.000000]\n"
      "      [0.000000 0.000000 0.000000]\n";
    EXPECT_EQ(expectedOutput, output);
  }

  // Tested command: gz model -m vehicle_blue --pose
  {
    const std::string cmd = kGzModelCommand + "-m vehicle_blue --pose ";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [diff_drive]...\n\n"
      "Model: [8]\n"
      "  - Name: vehicle_blue\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 2.000000 0.325000]\n"
      "    [0.000000 0.000000 0.000000]\n";
    EXPECT_EQ(expectedOutput, output);
  }

  // Tested command: gz model -m vehicle_blue --link
  {
    const std::string cmd = kGzModelCommand +
                            "-m vehicle_blue --link";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [diff_drive]...\n\n"
      "- Link [9]\n"
      "  - Name: chassis\n"
      "  - Parent: vehicle_blue [8]\n"
      "  - Mass (kg): 1.143950\n"
      "  - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Inertial Matrix (kg.m^2):\n"
      "    [0.126164 0.000000 0.000000]\n"
      "    [0.000000 0.416519 0.000000]\n"
      "    [0.000000 0.000000 0.481014]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [-0.151427 0.000000 0.175000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "- Link [12]\n"
      "  - Name: left_wheel\n"
      "  - Parent: vehicle_blue [8]\n"
      "  - Mass (kg): 2.000000\n"
      "  - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Inertial Matrix (kg.m^2):\n"
      "    [0.145833 0.000000 0.000000]\n"
      "    [0.000000 0.145833 0.000000]\n"
      "    [0.000000 0.000000 0.125000]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.554283 0.625029 -0.025000]\n"
      "    [-1.570700 0.000000 0.000000]\n"
      "- Link [15]\n"
      "  - Name: right_wheel\n"
      "  - Parent: vehicle_blue [8]\n"
      "  - Mass (kg): 2.000000\n"
      "  - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Inertial Matrix (kg.m^2):\n"
      "    [0.145833 0.000000 0.000000]\n"
      "    [0.000000 0.145833 0.000000]\n"
      "    [0.000000 0.000000 0.125000]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.554282 -0.625029 -0.025000]\n"
      "    [-1.570700 0.000000 0.000000]\n"
      "- Link [18]\n"
      "  - Name: caster\n"
      "  - Parent: vehicle_blue [8]\n"
      "  - Mass (kg): 1.000000\n"
      "  - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Inertial Matrix (kg.m^2):\n"
      "    [0.100000 0.000000 0.000000]\n"
      "    [0.000000 0.100000 0.000000]\n"
      "    [0.000000 0.000000 0.100000]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [-0.957138 0.000000 -0.125000]\n"
      "    [0.000000 0.000000 0.000000]\n";
    EXPECT_EQ(expectedOutput, output);
  }

  // Tested command: gz model -m vehicle_blue --link caster
  {
    const std::string cmd = kGzModelCommand +
                            "-m vehicle_blue --link caster";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [diff_drive]...\n\n"
      "- Link [18]\n"
      "  - Name: caster\n"
      "  - Parent: vehicle_blue [8]\n"
      "  - Mass (kg): 1.000000\n"
      "  - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Inertial Matrix (kg.m^2):\n"
      "    [0.100000 0.000000 0.000000]\n"
      "    [0.000000 0.100000 0.000000]\n"
      "    [0.000000 0.000000 0.100000]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [-0.957138 0.000000 -0.125000]\n"
      "    [0.000000 0.000000 0.000000]\n";
    EXPECT_EQ(expectedOutput, output);
  }

  // Tested command: gz model -m vehicle_blue --joint
  {
    const std::string cmd = kGzModelCommand +
                            "-m vehicle_blue --joint";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [diff_drive]...\n\n"
      "- Joint [21]\n"
      "  - Name: left_wheel_joint\n"
      "  - Parent: vehicle_blue [8]\n"
      "  - Type: revolute\n"
      "  - Parent Link: chassis [9]\n"
      "  - Child Link: left_wheel [12]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Axis unit vector [ XYZ ]:\n"
      "    [0 0 1]\n"
      "- Joint [22]\n"
      "  - Name: right_wheel_joint\n"
      "  - Parent: vehicle_blue [8]\n"
      "  - Type: revolute\n"
      "  - Parent Link: chassis [9]\n"
      "  - Child Link: right_wheel [15]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Axis unit vector [ XYZ ]:\n"
      "    [0 0 1]\n"
      "- Joint [23]\n"
      "  - Name: caster_wheel\n"
      "  - Parent: vehicle_blue [8]\n"
      "  - Type: ball\n"
      "  - Parent Link: chassis [9]\n"
      "  - Child Link: caster [18]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n";
    EXPECT_EQ(expectedOutput, output);
  }

  // Tested command: gz model -m vehicle_blue --joint caster_wheel
  {
    const std::string cmd = kGzModelCommand +
                            "-m vehicle_blue --joint caster_wheel";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [diff_drive]...\n\n"
      "- Joint [23]\n"
      "  - Name: caster_wheel\n"
      "  - Parent: vehicle_blue [8]\n"
      "  - Type: ball\n"
      "  - Parent Link: chassis [9]\n"
      "  - Child Link: caster [18]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n";
    EXPECT_EQ(expectedOutput, output);
  }
}

/////////////////////////////////////////////////
// Tests `gz model -s` command with an airpressure sensor.
TEST(ModelCommandAPI, AirPressureSensor)
{
  gz::sim::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "air_pressure.sdf"));

  gz::sim::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: gz model -m altimeter_mode -l link -s altimeter_sensor
  {
    const std::string cmd = kGzModelCommand
      + "-m air_pressure_model -l link -s air_pressure_sensor";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [air_pressure_sensor]...\n\n"
      "- Sensor [8]\n"
      "  - Name: air_pressure_sensor\n"
      "  - Parent: air_pressure_model [4]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Reference altitude (m): 123\n"
      "  - Pressure noise:\n"
      "    - Mean (Pa): 0\n"
      "    - Bias mean (Pa): 0\n"
      "    - Standard deviation (Pa): 0\n"
      "    - Bias standard deviation (Pa): 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation (Pa): 0\n"
      "    - Dynamic bias correlation time (s): 0\n";
    EXPECT_EQ(expectedOutput, output);
  }
}

/////////////////////////////////////////////////
// Tests `gz model -s` command with an altimeter.
TEST(ModelCommandAPI, AltimeterSensor)
{
  gz::sim::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "altimeter_with_pose.sdf"));

  gz::sim::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: gz model -m altimeter_mode -l link -s altimeter_sensor
  {
    const std::string cmd = kGzModelCommand
      + "-m altimeter_model -l link -s altimeter_sensor";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [altimeter_sensor]...\n\n"
      "- Sensor [12]\n"
      "  - Name: altimeter_sensor\n"
      "  - Parent: altimeter_model [8]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.100000 0.200000 0.300000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Vertical position noise:\n"
      "    - Mean (m): 0\n"
      "    - Bias mean (m): 0\n"
      "    - Standard deviation (m): 0\n"
      "    - Bias standard deviation (m): 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation (m): 0\n"
      "    - Dynamic bias correlation time (s): 0\n"
      "  - Vertical velocity noise:\n"
      "    - Mean (m/s): 0\n"
      "    - Bias mean (m/s): 0\n"
      "    - Standard deviation (m/s): 0\n"
      "    - Bias standard deviation (m/s): 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation (m/s): 0\n"
      "    - Dynamic bias correlation time (s): 0\n";
    EXPECT_EQ(expectedOutput, output);
  }
}

/////////////////////////////////////////////////
// Tests `gz model -s` command with a gpu lidar sensor.
TEST(ModelCommandAPI, GpuLidarSensor)
{
  gz::sim::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "gpu_lidar.sdf"));

  gz::sim::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: gz model -m altimeter_mode -l link -s altimeter_sensor
  {
    const std::string cmd = kGzModelCommand
      + "-m gpu_lidar -l gpu_lidar_link -s gpu_lidar";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [gpu_lidar_sensor]...\n\n"
      "- Sensor [8]\n"
      "  - Name: gpu_lidar\n"
      "  - Parent: gpu_lidar [4]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Range:\n"
      "    - Min (m): 0.08\n"
      "    - Max (m): 10\n"
      "    - Resolution: 0.01\n"
      "  - Horizontal scan:\n"
      "    - Samples: 640\n"
      "    - Resolution: 1\n"
      "    - Min angle (rad): -1.39626\n"
      "    - Max angle (rad): 1.39626\n"
      "  - Vertical scan:\n"
      "    - Samples: 1\n"
      "    - Resolution: 0.01\n"
      "    - Min angle (rad): 0\n"
      "    - Max angle (rad): 0\n"
      "  - Noise:\n"
      "    - Mean (m): 0\n"
      "    - Bias mean (m): 0\n"
      "    - Standard deviation (m): 0\n"
      "    - Bias standard deviation (m): 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation (m): 0\n"
      "    - Dynamic bias correlation time (s): 0\n";
    EXPECT_EQ(expectedOutput, output);
  }
}

/////////////////////////////////////////////////
// Tests `gz model -s` command with a magnetometer.
TEST(ModelCommandAPI, MagnetometerSensor)
{
  gz::sim::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "magnetometer.sdf"));

  gz::sim::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: gz model -m altimeter_mode -l link -s altimeter_sensor
  {
    const std::string cmd = kGzModelCommand
      + "-m magnetometer_model -l link -s magnetometer_sensor";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [magnetometer_sensor]...\n\n"
      "- Sensor [12]\n"
      "  - Name: magnetometer_sensor\n"
      "  - Parent: magnetometer_model [8]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - X-axis noise:\n"
      "    - Mean (T): 0\n"
      "    - Bias mean (T): 0\n"
      "    - Standard deviation (T): 0\n"
      "    - Bias standard deviation (T): 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation (T): 0\n"
      "    - Dynamic bias correlation time (s): 0\n"
      "  - Y-axis noise:\n"
      "    - Mean (T): 0\n"
      "    - Bias mean (T): 0\n"
      "    - Standard deviation (T): 0\n"
      "    - Bias standard deviation (T): 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation (T): 0\n"
      "    - Dynamic bias correlation time (s): 0\n"
      "  - Z-axis noise:\n"
      "    - Mean (T): 0\n"
      "    - Bias mean (T): 0\n"
      "    - Standard deviation (T): 0\n"
      "    - Bias standard deviation (T): 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation (T): 0\n"
      "    - Dynamic bias correlation time (s): 0\n";

      EXPECT_EQ(expectedOutput, output);
  }
}

/////////////////////////////////////////////////
// Tests `gz model -s` command with an rgbd camera.
TEST(ModelCommandAPI, GZ_UTILS_TEST_DISABLED_ON_MAC(RgbdCameraSensor))
{
  gz::sim::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "rgbd_camera_sensor.sdf"));

  gz::sim::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: gz model -m rgbd_camera -l rgbd_camera_link -s rgbd_camera
  {
    const std::string cmd = kGzModelCommand
      + "-m rgbd_camera -l rgbd_camera_link -s rgbd_camera";
    std::string output = customExecStr(cmd);
    ReplaceNegativeZeroValues(output);
    const std::string expectedOutput =
      "\nRequesting state for world [rgbd_camera_sensor]...\n\n"
      "- Sensor [16]\n"
      "  - Name: rgbd_camera\n"
      "  - Parent: rgbd_camera [12]\n"
      "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n"
      "    [0.000000 0.000000 0.000000]\n"
      "    [0.000000 0.000000 0.000000]\n"
      "  - Horizontal field of view (rad): 1.05\n"
      "  - Image width (px): 256\n"
      "  - Image height (px): 256\n"
      "  - Near clip (m): 0.1\n"
      "  - Far clip (m): 10\n"
      "  - Pixel format: RGB_INT8\n"
      "  - Save frames: 0\n"
      "  - Save frames path: \n"
      "  - Image noise:\n"
      "    - Mean: 0\n"
      "    - Bias mean: 0\n"
      "    - Standard deviation: 0\n"
      "    - Bias standard deviation: 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation: 0\n"
      "    - Dynamic bias correlation time (s): 0\n"
      "  - Distortion K1: 0\n"
      "  - Distortion K2: 0\n"
      "  - Distortion K3: 0\n"
      "  - Distortion P1: 0\n"
      "  - Distortion P2: 0\n"
      "  - Distortion center: 0.5 0.5\n"
      "  - Lens type: stereographic\n"
      "  - Lens scale to horizontal field of view (rad): 1\n"
      "  - Lens C1: 1\n"
      "  - Lens C2: 1\n"
      "  - Lens C3: 0\n"
      "  - Lens focal length (m): 1\n"
      "  - Lens function: tan\n"
      "  - Lens cutoff angle (rad): 1.5708\n"
      "  - Lens texture size: 256\n"
      "  - Lens intrinsics Fx: 277\n"
      "  - Lens intrinsics Fy: 277\n"
      "  - Lens intrinsics Cx: 160\n"
      "  - Lens intrinsics Cy: 120\n"
      "  - Lens intrinsics skew: 0\n"
      "  - Visibility mask: 4294967295\n";
      EXPECT_EQ(expectedOutput, output);
  }
}

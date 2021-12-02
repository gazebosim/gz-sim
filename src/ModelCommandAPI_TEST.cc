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

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

static const std::string kIgnModelCommand(
    std::string(BREW_RUBY) + std::string(IGN_PATH) + "/ign model ");


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
// Test `ign model` command when no Gazebo server is running.
TEST(ModelCommandAPI, NoServerRunning)
{
  const std::string cmd = kIgnModelCommand + "--list ";
  const std::string output = customExecStr(cmd);
  const std::string expectedOutput =
        "\nService call to [/gazebo/worlds] timed out\n"
        "Command failed when trying to get the world name "
        "of the running simulation.\n";
  EXPECT_EQ(expectedOutput, output);
}

/////////////////////////////////////////////////
// Tests `ign model` command.
TEST(ModelCommandAPI, Commands)
{
  ignition::gazebo::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      ignition::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "static_diff_drive_vehicle.sdf"));

  ignition::gazebo::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: ign model --list
  {
    const std::string cmd = kIgnModelCommand + "--list";
    const std::string output = customExecStr(cmd);
    const std::string expectedOutput =
      "\nRequesting state for world [diff_drive]..."
      "\n\nAvailable models:\n"
      "    - ground_plane\n"
      "    - vehicle_blue\n";
    EXPECT_EQ(expectedOutput, output);
  }

  // Tested command: ign model -m vehicle_blue
  {
    const std::string cmd = kIgnModelCommand + "-m vehicle_blue";
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

  // Tested command: ign model -m vehicle_blue --pose
  {
    const std::string cmd = kIgnModelCommand + "-m vehicle_blue --pose ";
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

  // Tested command: ign model -m vehicle_blue --link
  {
    const std::string cmd = kIgnModelCommand +
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

  // Tested command: ign model -m vehicle_blue --link caster
  {
    const std::string cmd = kIgnModelCommand +
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

  // Tested command: ign model -m vehicle_blue --joint
  {
    const std::string cmd = kIgnModelCommand +
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

  // Tested command: ign model -m vehicle_blue --joint caster_wheel
  {
    const std::string cmd = kIgnModelCommand +
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
// Tests `ign model -s` command with an altimeter.
TEST(ModelCommandAPI, AltimeterSensor)
{
  ignition::gazebo::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      ignition::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "altimeter_with_pose.sdf"));

  ignition::gazebo::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: ign model -m altimeter_mode -l link -s altimeter_sensor
  {
    const std::string cmd = kIgnModelCommand
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
      "    - Mean: 0\n"
      "    - Bias mean: 0\n"
      "    - Standard deviation: 0\n"
      "    - Bias standard deviation: 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation: 0\n"
      "    - Dynamic bias correlation time: 0\n"
      "  - Vertical velocity noise:\n"
      "    - Mean: 0\n"
      "    - Bias mean: 0\n"
      "    - Standard deviation: 0\n"
      "    - Bias standard deviation: 0\n"
      "    - Precision: 0\n"
      "    - Dynamic bias standard deviation: 0\n"
      "    - Dynamic bias correlation time: 0\n";
    EXPECT_EQ(expectedOutput, output);
  }
}

/////////////////////////////////////////////////
// Tests `ign model -s` command with a magnetometer.
TEST(ModelCommandAPI, MagnetometerSensor)
{
  ignition::gazebo::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      ignition::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "magnetometer.sdf"));

  ignition::gazebo::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: ign model -m altimeter_mode -l link -s altimeter_sensor
  {
    const std::string cmd = kIgnModelCommand
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
      "    [0.000000 0.000000 0.000000]\n";
      EXPECT_EQ(expectedOutput, output);
  }
}

/////////////////////////////////////////////////
// Tests `ign model -s` command with an rgbd camera.
TEST(ModelCommandAPI, RgbdCameraSensor)
{
  ignition::gazebo::ServerConfig serverConfig;
  // Using an static model to avoid any movements in the simulation.
  serverConfig.SetSdfFile(
      ignition::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "rgbd_camera_sensor.sdf"));

  ignition::gazebo::Server server(serverConfig);
  // Run at least one iteration before continuing to guarantee correctly set up.
  ASSERT_TRUE(server.Run(true, 5, false));
  // Run without blocking.
  server.Run(false, 0, false);

  // Tested command: ign model -m altimeter_mode -l link -s altimeter_sensor
  {
    const std::string cmd = kIgnModelCommand
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
      "    - Dynamic bias correlation time: 0\n"
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
      "  - Lens intrinsics skew: 1\n"
      "  - Visibility mask: 4294967295\n";
      EXPECT_EQ(expectedOutput, output);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

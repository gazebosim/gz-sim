/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

/*
  Adapted from https://github.com/ros/ros_tutorials/blob/lunar-devel/turtlesim/tutorials/teleop_turtle_key.cpp
  Original Copyright:
  Software License Agreement (BSD License)
  Copyright (c) 2008, Willow Garage, Inc.
  All rights reserved.
 */


#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <gz/common/Console.hh>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <unistd.h>  // read()

#define KEYCODE_ARR_R 0x43
#define KEYCODE_ARR_L 0x44
#define KEYCODE_ARR_U 0x41
#define KEYCODE_ARR_D 0x42

#define KEYCODE_Q 0x71

#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_W 0x77


gz::transport::Node::Publisher cmdVelPub;
gz::transport::Node::Publisher cmdVelPub2;

gz::math::Vector3d scaleLinear;

gz::math::Vector3d scaleAngular;


class KeyboardTeleop
{
public:

  KeyboardTeleop();
  KeyboardTeleop(double, double);
  void KeyLoop();

private:

  double lScale, aScale;
};


KeyboardTeleop::KeyboardTeleop():
  lScale(1.0),
  aScale(1.0)
{

}

KeyboardTeleop::KeyboardTeleop(double _sl, double _sa):
  lScale(_sl),
  aScale(_sa)
{

}


int kfd = 0;
struct termios cooked, raw;

void Quit(int _sig)
{
  (void)_sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}


void KeyboardTeleop::KeyLoop()
{
  char c;
  bool dirty = false, dirty2 = false;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");


  for (;;)
  {
    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    double linear = 0, linear2 = 0;
    double angular = 0, angular2 = 0;
    fprintf (stderr, "value: 0x%02X\n", c);

    switch (c)
    {
      // robot 1
      case KEYCODE_ARR_L:
        std::cerr << "LEFT" << std::endl;
        angular = 1.0;
        dirty = true;
        break;
      case KEYCODE_ARR_R:
        std::cerr << "RIGHT" << std::endl;
        angular = -1.0;
        dirty = true;
        break;
      case KEYCODE_ARR_U:
        std::cerr << "UP" << std::endl;
        linear = 1.0;
        dirty = true;
        break;
      case KEYCODE_ARR_D:
        std::cerr << "DOWN" << std::endl;
        linear = -1.0;
        dirty = true;
        break;
      // robot 2
      case KEYCODE_A:
        std::cerr << "A" << std::endl;
        angular2 = 1.0;
        dirty2 = true;
        break;
      case KEYCODE_D:
        std::cerr << "D" << std::endl;
        angular2 = -1.0;
        dirty2 = true;
        break;
      case KEYCODE_W:
        std::cerr << "W" << std::endl;
        linear2 = 1.0;
        dirty2 = true;
        break;
      case KEYCODE_S:
        std::cerr << "S" << std::endl;
        linear2 = -1.0;
        dirty2 = true;
        break;
    }

    gz::msgs::Twist cmdVelMsg;
    cmdVelMsg.mutable_linear()->set_x(lScale * linear);
    cmdVelMsg.mutable_angular()->set_z(aScale * angular);

    gz::msgs::Twist cmdVelMsg2;
    cmdVelMsg2.mutable_linear()->set_x(lScale * linear2);
    cmdVelMsg2.mutable_angular()->set_z(aScale * angular2);

    if (dirty)
    {
      cmdVelPub.Publish(cmdVelMsg);
      dirty = false;
    }
    if (dirty2)
    {
      cmdVelPub2.Publish(cmdVelMsg2);
      dirty2 = false;
    }
  }

  return;
}


int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: keyboard <sdf_file>" << std::endl;
    return -1;
  }

  // Get parameters from SDF file
  auto sdf = sdf::readFile(argv[1]);

  if (!sdf)
  {
    std::cerr << "Failed to parse file [" << argv[1] << "]" << std::endl;
    return -1;
  }


  // TODO(louise) This is not a plugin, we need a new SDF tag
  auto plugin = sdf->Root()->GetElement("world")->GetElement("plugin");

  // Set up transport
  gz::transport::Node node;

  auto twistTopic = plugin->Get<std::string>("twist_arrows", "/cmd_vel").first;
  cmdVelPub = node.Advertise<gz::msgs::Twist>(twistTopic);

  auto twistTopic2 = plugin->Get<std::string>("twist_wasd", "/cmd_vel").first;
  cmdVelPub2 = node.Advertise<gz::msgs::Twist>(twistTopic2);

  scaleLinear = plugin->Get<gz::math::Vector3d>("scale_linear",
      gz::math::Vector3d(0.5, 0, 0)).first;

  scaleAngular = plugin->Get<gz::math::Vector3d>("scale_angular",
      gz::math::Vector3d(0, 0, 0.5)).first;


  // Only linear X and angular Z are used
  KeyboardTeleop teleop_turtle = KeyboardTeleop (scaleLinear.X(),
    scaleAngular.Z());
  signal(SIGINT, Quit);
  teleop_turtle.KeyLoop();

  return(0);
}

/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

ignition::transport::Node::Publisher cmdVelPub;

int enableButton;
int enableTurboButton;

std::map<std::string, int> axisLinearMap;
std::map<std::string, double> scaleLinearMap;
std::map<std::string, double> scaleLinearTurboMap;

std::map<std::string, int> axisAngularMap;
std::map<std::string, double> scaleAngularMap;
std::map<std::string, double> scaleAngularTurboMap;

bool sentDisableMsg;

//////////////////////////////////////////////////
void OnJoy(const ignition::msgs::Joy &_msg)
{
  ignition::msgs::Twist cmdVelMsg;

  if (enableTurboButton >= 0 && _msg.buttons(enableTurboButton))
  {
    if (axisLinearMap.find("x") != axisLinearMap.end())
    {
      cmdVelMsg.mutable_linear()->set_x(_msg.axes(axisLinearMap["x"]) * scaleLinearTurboMap["x"]);
    }
    if (axisLinearMap.find("y") != axisLinearMap.end())
    {
      cmdVelMsg.mutable_linear()->set_y(_msg.axes(axisLinearMap["y"]) * scaleLinearTurboMap["y"]);
    }
    if (axisLinearMap.find("z") != axisLinearMap.end())
    {
      cmdVelMsg.mutable_linear()->set_z(_msg.axes(axisLinearMap["z"]) * scaleLinearTurboMap["z"]);
    }
    if (axisAngularMap.find("yaw") != axisAngularMap.end())
    {
      cmdVelMsg.mutable_angular()->set_z(_msg.axes(axisAngularMap["yaw"]) * scaleAngularTurboMap["yaw"]);
    }
    if (axisAngularMap.find("pitch") != axisAngularMap.end())
    {
      cmdVelMsg.mutable_angular()->set_y(_msg.axes(axisAngularMap["pitch"]) * scaleAngularTurboMap["pitch"]);
    }
    if (axisAngularMap.find("roll") != axisAngularMap.end())
    {
      cmdVelMsg.mutable_angular()->set_x(_msg.axes(axisAngularMap["roll"]) * scaleAngularTurboMap["roll"]);
    }

    cmdVelPub.Publish(cmdVelMsg);
    sentDisableMsg = false;
  }
  else if (_msg.buttons(enableButton))
  {
    if (axisLinearMap.find("x") != axisLinearMap.end())
    {
      cmdVelMsg.mutable_linear()->set_x(_msg.axes(axisLinearMap["x"]) * scaleLinearMap["x"]);
    }
    if (axisLinearMap.find("y") != axisLinearMap.end())
    {
      cmdVelMsg.mutable_linear()->set_y(_msg.axes(axisLinearMap["y"]) * scaleLinearMap["y"]);
    }
    if (axisLinearMap.find("z") != axisLinearMap.end())
    {
      cmdVelMsg.mutable_linear()->set_z(_msg.axes(axisLinearMap["z"]) * scaleLinearMap["z"]);
    }
    if (axisAngularMap.find("yaw") != axisAngularMap.end())
    {
      cmdVelMsg.mutable_angular()->set_z(_msg.axes(axisAngularMap["yaw"]) * scaleAngularMap["yaw"]);
    }
    if (axisAngularMap.find("pitch") != axisAngularMap.end())
    {
      cmdVelMsg.mutable_angular()->set_y(_msg.axes(axisAngularMap["pitch"]) * scaleAngularMap["pitch"]);
    }
    if (axisAngularMap.find("roll") != axisAngularMap.end())
    {
      cmdVelMsg.mutable_angular()->set_x(_msg.axes(axisAngularMap["roll"]) * scaleAngularMap["roll"]);
    }

    cmdVelPub.Publish(cmdVelMsg);
    sentDisableMsg = false;
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sentDisableMsg)
    {
      cmdVelPub.Publish(cmdVelMsg);
      sentDisableMsg = true;
    }
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: joy_to_twist <sdf_file>" << std::endl;
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

  // Setup transport
  ignition::transport::Node node;
  cmdVelPub = node.Advertise<ignition::msgs::Twist>("/model/vehicle_blue/cmd_vel");
  node.Subscribe("/joy", OnJoy);

   enableButton = plugin->Get<int>("enable_button", 0).first;
   enableTurboButton = plugin->Get<int>("enable_turbo_button", -1).first;

//  if (nh_param->getParam("axis_linear", axis_linear_map))
//  {
//    nh_param->getParam("axis_linear", axis_linear_map).first;
//    nh_param->getParam("scale_linear", scale_linear_map).first;
//    nh_param->getParam("scale_linear_turbo", scale_linear_turbo_map).first;
//  }
//  else
  {
    axisLinearMap["x"]  = plugin->Get<int>("axis_linear", 1).first;
    scaleLinearMap["x"] = plugin->Get<double>("scale_linear", 0.5).first;
    scaleLinearTurboMap["x"] = plugin->Get<double>("scale_linear_turbo", 1.0).first;
  }

//  if (nh_param->getParam("axis_angular", axis_angular_map))
//  {
//    nh_param->getParam("axis_angular", axis_angular_map).first;
//    nh_param->getParam("scale_angular", scale_angular_map).first;
//    nh_param->getParam("scale_angular_turbo", scale_angular_turbo_map).first;
//  }
//  else
  {
    axisAngularMap["yaw"] = plugin->Get<int>("axis_angular", 0).first;
    scaleAngularMap["yaw"] = plugin->Get<double>("scale_angular", 0.5).first;
    scaleAngularTurboMap["yaw"]  = plugin->Get<double>("scale_angular_turbo",
        scaleAngularMap["yaw"]).first;
  }

  sentDisableMsg = false;

  while (true) {}
}


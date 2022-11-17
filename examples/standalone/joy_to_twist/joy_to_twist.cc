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

/*
 * Adapted from https://github.com/ros-teleop/teleop_twist_joy
 */

#include <gz/msgs/joy.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

gz::transport::Node::Publisher cmdVelPub;

int enableButton;
int enableTurboButton;

gz::math::Vector3d axisLinear;
gz::math::Vector3d scaleLinear;
gz::math::Vector3d scaleLinearTurbo;

gz::math::Vector3d axisAngular;
gz::math::Vector3d scaleAngular;
gz::math::Vector3d scaleAngularTurbo;

bool sentDisableMsg;

//////////////////////////////////////////////////
void OnJoy(const gz::msgs::Joy &_msg)
{
  gz::msgs::Twist cmdVelMsg;

  // Turbo mode
  if (enableTurboButton >= 0 && _msg.buttons(enableTurboButton))
  {
    cmdVelMsg.mutable_linear()->set_x(
        _msg.axes(axisLinear.X()) * scaleLinearTurbo.X());
    cmdVelMsg.mutable_linear()->set_y(
        _msg.axes(axisLinear.Y()) * scaleLinearTurbo.Y());
    cmdVelMsg.mutable_linear()->set_z(
        _msg.axes(axisLinear.Z()) * scaleLinearTurbo.Z());

    cmdVelMsg.mutable_angular()->set_x(
        _msg.axes(axisAngular.X()) * scaleAngularTurbo.X());
    cmdVelMsg.mutable_angular()->set_y(
        _msg.axes(axisAngular.Y()) * scaleAngularTurbo.Y());
    cmdVelMsg.mutable_angular()->set_z(
        _msg.axes(axisAngular.Z()) * scaleAngularTurbo.Z());

    cmdVelPub.Publish(cmdVelMsg);
    sentDisableMsg = false;
  }
  // Normal mode
  else if (_msg.buttons(enableButton))
  {
    cmdVelMsg.mutable_linear()->set_x(
        _msg.axes(axisLinear.X()) * scaleLinear.X());
    cmdVelMsg.mutable_linear()->set_y(
        _msg.axes(axisLinear.Y()) * scaleLinear.Y());
    cmdVelMsg.mutable_linear()->set_z(
        _msg.axes(axisLinear.Z()) * scaleLinear.Z());

    cmdVelMsg.mutable_angular()->set_x(
        _msg.axes(axisAngular.X()) * scaleAngular.X());
    cmdVelMsg.mutable_angular()->set_y(
        _msg.axes(axisAngular.Y()) * scaleAngular.Y());
    cmdVelMsg.mutable_angular()->set_z(
        _msg.axes(axisAngular.Z()) * scaleAngular.Z());

    cmdVelPub.Publish(cmdVelMsg);
    sentDisableMsg = false;
  }
  else
  {
    // When enable button is released, immediately send a single no-motion
    // command in order to stop the robot.
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
  gz::transport::Node node;

  auto twistTopic = plugin->Get<std::string>("twist_topic", "/cmd_vel").first;
  cmdVelPub = node.Advertise<gz::msgs::Twist>(twistTopic);

  auto joyTopic = plugin->Get<std::string>("joy_topic", "/joy").first;
  node.Subscribe("/joy", OnJoy);

  enableButton = plugin->Get<int>("enable_button", 0).first;
  enableTurboButton = plugin->Get<int>("enable_turbo_button", -1).first;

  axisLinear  = plugin->Get<gz::math::Vector3d>("axis_linear",
      gz::math::Vector3d::UnitX).first;
  scaleLinear  = plugin->Get<gz::math::Vector3d>("scale_linear",
      gz::math::Vector3d(0.5, 0, 0)).first;
  scaleLinearTurbo  = plugin->Get<gz::math::Vector3d>(
      "scale_linear_turbo", scaleLinear).first;

  axisAngular = plugin->Get<gz::math::Vector3d>("axis_angular",
      gz::math::Vector3d::Zero).first;
  scaleAngular = plugin->Get<gz::math::Vector3d>("scale_angular",
      gz::math::Vector3d(0, 0, 0.5)).first;
  scaleAngularTurbo  = plugin->Get<gz::math::Vector3d>(
      "scale_angular_turbo", scaleAngular).first;

  sentDisableMsg = false;

  while (true)
  {
  }
}

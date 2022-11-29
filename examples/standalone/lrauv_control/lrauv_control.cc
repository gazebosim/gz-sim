/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
 * Check README for detailed instructions.
 * Usage:
 *   $ gz sim -r worlds/lrauv_control_demo.sdf
 *   $ ./lrauv_control
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#define PI 3.14159265359

using namespace gz;

class Controller
{
  public:
    // Desired state.
    double targetSpeed = 0.5;
    double targetYawAngle = PI / 4;
    double targetPitchAngle = PI / 18;

    // Errors
    double errorSpeed;
    double errorYawAngle;
    double errorPitchAngle;

    // States to be tracked and controlled.
    double speed;
    double yawAngle;
    double pitchAngle;

    // PID gains.
    double kSpeed = -30;
    double kYawAngle = -1.0;
    double kPitchAngle = 1.0;

    void updateErrors()
    {
      errorSpeed = targetSpeed - speed;
      errorYawAngle = targetYawAngle - yawAngle;
      errorPitchAngle = targetPitchAngle - pitchAngle;
    }

    double speedControl()
    {
      return errorSpeed * kSpeed;
    }

    double yawControl()
    {
      return errorYawAngle * kYawAngle;
    }

    double pitchControl()
    {
      return errorPitchAngle * kPitchAngle;
    }
};

int main(int argc, char** argv)
{
  auto control = Controller();

  transport::Node node;

  // Propeller command publisher.
  auto propellerTopicTethys =
    gz::transport::TopicUtils::AsValidTopic(
      "/model/tethys/joint/propeller_joint/cmd_pos");
  auto propellerPubTethys =
    node.Advertise<gz::msgs::Double>(propellerTopicTethys);

  // Subscriber for vehicle pose.
  std::function<void(const msgs::Odometry &)> cbPos =
    [&](const msgs::Odometry &_msg)
  {
    auto orientation = _msg.pose().orientation();

    gz::math::Quaterniond q(
        orientation.w(),
        orientation.x(),
        orientation.y(),
        orientation.z());

    // Update yaw and pitch angles.
    auto rpy = q.Euler();
    control.yawAngle = rpy[2];
    control.pitchAngle = rpy[1];

    gz::math::Vector3d velocity(
        _msg.twist().linear().x(),
        _msg.twist().linear().y(),
        _msg.twist().linear().z());

    control.speed = velocity.Length();
  };

  node.Subscribe("/model/tethys/odometry", cbPos);

  // Rudder command publisher.
  auto rudderTopicTethys =
    gz::transport::TopicUtils::AsValidTopic(
      "/model/tethys/joint/vertical_fins_joint/0/cmd_pos");
  auto rudderPubTethys =
    node.Advertise<gz::msgs::Double>(rudderTopicTethys);

  // Fin command publisher.
  auto finTopicTethys =
    gz::transport::TopicUtils::AsValidTopic(
      "/model/tethys/joint/horizontal_fins_joint/0/cmd_pos");
  auto finPubTethys =
    node.Advertise<gz::msgs::Double>(finTopicTethys);

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Update errors in the controller.
    control.updateErrors();

    // Publish propeller command for speed.
    msgs::Double propellerMsg;
    propellerMsg.set_data(control.speedControl());
    propellerPubTethys.Publish(propellerMsg);

    // Publish rudder command for yaw.
    msgs::Double rudderMsg;
    rudderMsg.set_data(control.yawControl());
    rudderPubTethys.Publish(rudderMsg);

    // Publish fin command for pitch.
    msgs::Double finMsg;
    finMsg.set_data(control.pitchControl());
    finPubTethys.Publish(finMsg);

    // Print the states.
    std::cout << std::setprecision(5) << std::fixed;
    std::cout << "-----------------------" << std::endl;
    std::cout << "States            ( target, current, error) : " << std::endl;

    std::cout << "Speed (m/s)       : " << control.targetSpeed << " " << control.speed << " "
      << control.errorSpeed << std::endl;

    std::cout << "Yaw angle (deg)   : " << control.targetYawAngle * 180/PI << " "
      << control.yawAngle * 180/PI << " " << control.errorYawAngle * 180/PI<< std::endl;

    std::cout << "Pitch angle (deg) : "<< control.targetPitchAngle * 180/PI << " "
      << control.pitchAngle * 180/PI << " " << control.errorPitchAngle * 180/PI<< std::endl;
  }
}

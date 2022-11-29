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
 *   $ # ./lrauv_control speed_m_s yaw_rad pitch_rad
 *   $ ./lrauv_control 0.5 0.78 0.174
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

// Helper class for the PID controller for
// linear speed, pitch and yaw angles.
class Controller
{
  public:
    // Desired state.
    double targetSpeed = 0;
    double targetYawAngle =  0;
    double targetPitchAngle = 0;

    // Errors
    double errorSpeed = 0;
    double errorSpeedIntegral = 0;
    double errorYawAngle = 0;
    double errorPitchAngle = 0;

    // States to be tracked and controlled.
    std::atomic<double> speed;
    std::atomic<double> yawAngle;
    std::atomic<double> pitchAngle;

    // PID gains and error limits.
    // PI for speed.
    double kSpeed = -30;
    double kISpeed = -0.5;
    double errorSpeedIntegralMax = 10;

    // P for yaw and pitch control.
    double kYawAngle = -0.5;
    double kPitchAngle = 0.6;

    // Set the target states to be tracked,
    // i.e. linear speed (m/s), pitch and yaw angles (rad).
    void SetTargets(double _speed,
        double _yaw, double _pitch)
    {
      if (_speed < 0.001 &&
          (_yaw != 0 || _pitch != 0))
      {
        std::cout << "Speed needs to be non zero for non zero"
          " pitch and yaw angles" << std::endl;
        return;
      }

      targetSpeed = std::move(_speed);
      targetYawAngle = std::move(_yaw);
      targetPitchAngle = std::move(_pitch);
    }

    // Update the errors after an iteration of control loop.
    void UpdateErrors()
    {
      errorSpeed = targetSpeed - speed;
      errorSpeedIntegral =
        std::min(errorSpeedIntegral + errorSpeed, errorSpeedIntegralMax);

      errorYawAngle = targetYawAngle - yawAngle;
      errorPitchAngle = targetPitchAngle - pitchAngle;
    }

    // Generate control input to be applied to the thruster.
    double SpeedControl()
    {
      return errorSpeed * kSpeed + errorSpeedIntegral * kISpeed;
    }

    // Generate control input to be supplied to the yaw rudders.
    double YawControl()
    {
      return errorYawAngle * kYawAngle;
    }

    // Generate control input to be supplied to the pitch rudders.
    double PitchControl()
    {
      return errorPitchAngle * kPitchAngle;
    }
};

int main(int argc, char** argv)
{
  auto control = Controller();

  if (argc == 4)
  {
    double targetSpeed = std::stod(argv[1]);
    double targetYaw = std::stod(argv[2]);
    double targetPitch = std::stod(argv[3]);

    // Target state : speed (m/s), yaw angle, pitch angle (rad).
    control.SetTargets(targetSpeed ,targetYaw ,targetPitch);
  }

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
    control.UpdateErrors();

    // Publish propeller command for speed.
    msgs::Double propellerMsg;
    propellerMsg.set_data(control.SpeedControl());
    propellerPubTethys.Publish(propellerMsg);

    // Publish rudder command for yaw.
    msgs::Double rudderMsg;
    rudderMsg.set_data(control.YawControl());
    rudderPubTethys.Publish(rudderMsg);

    // Publish fin command for pitch.
    msgs::Double finMsg;
    finMsg.set_data(control.PitchControl());
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

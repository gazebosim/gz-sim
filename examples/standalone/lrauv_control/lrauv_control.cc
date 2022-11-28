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

using namespace gz;

int main(int argc, char** argv)
{
  transport::Node node;
  int linearSpeed = -10;
  double xPos, yPos;

  // Propeller command publisher.
  auto propellerTopicTethys =
    gz::transport::TopicUtils::AsValidTopic(
      "/model/tethys/joint/propeller_joint/cmd_pos");
  auto propellerPubTethys =
    node.Advertise<gz::msgs::Double>(propellerTopicTethys);

  // Subscriber for vehicle pose.
  std::function<void(const msgs::Pose_V &)> cbPos =
    [&](const msgs::Pose_V &_msg)
  {
    xPos = _msg.pose()[0].position().x();
    yPos = _msg.pose()[0].position().y();
  };

  node.Subscribe("/model/tethys/pose", cbPos);

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Publish propeller command.
    msgs::Double propellerMsg;
    propellerMsg.set_data(linearSpeed);

    propellerPubTethys.Publish(propellerMsg);

    // Publish rudder msg.
    std::cout << "x, y: " << xPos << " " << yPos << std::endl;
  }
}

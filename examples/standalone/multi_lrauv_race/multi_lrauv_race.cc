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

/*
 * In each iteration, for each vehicle, generate a random fin angle and thrust
 * within reasonable limits, and send the command to the vehicle.
 *
 * Usage:
 *   $ multi_lrauv_race
 */

#include <chrono>
#include <thread>

#include <gz/msgs/double.pb.h>

#include <gz/transport/Node.hh>

// Fin joint limits from tethys model.sdf
double random_angle_within_limits(double min=-0.261799, double max=0.261799)
{
  return min + static_cast<float>(rand()) /
    (static_cast<float>(RAND_MAX / (max - min)));
}

// Nominal speed is thruster 300 rpm ~ 31.4 radians per second ~ 6.14 Newtons
double random_thrust_within_limits(double min=-6.14, double max=6.14)
{
  return min + static_cast<float>(rand()) /
    (static_cast<float>(RAND_MAX / (max - min)));
}

int main(int argc, char** argv)
{
  // Initialize random seed
  srand(time(NULL));

  std::vector<std::string> ns;
  ns.push_back("tethys");
  ns.push_back("triton");
  ns.push_back("daphne");

  gz::transport::Node node;

  std::vector<std::string> rudderTopics;
  rudderTopics.resize(ns.size(), "");
  std::vector<gz::transport::Node::Publisher> rudderPubs;
  rudderPubs.resize(ns.size());

  std::vector<std::string> propellerTopics;
  propellerTopics.resize(ns.size(), "");
  std::vector<gz::transport::Node::Publisher> propellerPubs;
  propellerPubs.resize(ns.size());

  // Set up topic names and publishers
  for (int i = 0; i < ns.size(); i++)
  {
    rudderTopics[i] = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns[i] + "/joint/vertical_fins_joint/0/cmd_pos");
    rudderPubs[i] = node.Advertise<gz::msgs::Double>(rudderTopics[i]);

    propellerTopics[i] = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns[i] + "/joint/propeller_joint/cmd_thrust");
    propellerPubs[i] = node.Advertise<gz::msgs::Double>(
      propellerTopics[i]);
  }

  std::vector<double> rudderCmds;
  rudderCmds.resize(ns.size(), 0.0);
  std::vector<double> propellerCmds;
  propellerCmds.resize(ns.size(), 0.0);

  float artificial_speedup = 1;

  while (true)
  {
    for (int i = 0; i < ns.size(); i++)
    {
      rudderCmds[i] = random_angle_within_limits(-0.01, 0.01);
      gz::msgs::Double rudderMsg;
      rudderMsg.set_data(rudderCmds[i]);
      rudderPubs[i].Publish(rudderMsg);

      propellerCmds[i] = random_thrust_within_limits(
        -6.14 * artificial_speedup, 0);
      gz::msgs::Double propellerMsg;
      propellerMsg.set_data(propellerCmds[i]);
      propellerPubs[i].Publish(propellerMsg);

      std::cout << "Commanding " << ns[i] << " rudder angle " << rudderCmds[i]
        << " rad, thrust " << propellerCmds[i] << " Newtons" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

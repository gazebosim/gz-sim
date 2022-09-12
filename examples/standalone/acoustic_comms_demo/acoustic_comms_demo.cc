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
 *   $ gz sim -r worlds/acoustic_comms_demo.sdf
 *   $ acoustic_comms_demo
 */

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs.hh>
#include <gz/transport.hh>

using namespace gz;

std::atomic<bool> tethysMsgReceived = false;
std::atomic<bool> daphneMsgReceived = false;

void cbTethys(const msgs::Dataframe &_msg)
{
  tethysMsgReceived = true;
}

void cbDaphne(const msgs::Dataframe &_msg)
{
  daphneMsgReceived = true;
}

int main(int argc, char** argv)
{
  std::vector<std::string> ns;
  ns.push_back("triton");
  ns.push_back("tethys");
  ns.push_back("daphne");

  transport::Node node;

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
    propellerTopics[i] = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + ns[i] + "/joint/propeller_joint/cmd_pos");
    propellerPubs[i] = node.Advertise<gz::msgs::Double>(
      propellerTopics[i]);
  }

  std::vector<double> propellerCmds = {-20, 0, 0};

  // Setup publishers and callbacks for comms topics.
  auto senderAddressTriton = "1";

  std::string receiverAddressTethys = "2";
  std::string receiverAddressDaphne = "3";

  node.Subscribe("/" + receiverAddressTethys + "/rx", cbTethys);
  node.Subscribe("/" + receiverAddressDaphne + "/rx", cbDaphne);

  // Publisher to send the START msg.
  auto pub = node.Advertise<gz::msgs::Dataframe>(
      "/broker/msgs");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Prepare and send the msgs from triton to tethys and daphne
  msgs::Dataframe msgTethys;
  msgs::Dataframe msgDaphne;

  msgTethys.set_src_address(senderAddressTriton);
  msgDaphne.set_src_address(senderAddressTriton);

  msgTethys.set_dst_address(receiverAddressTethys);
  msgDaphne.set_dst_address(receiverAddressDaphne);

  msgTethys.set_data("START");
  msgDaphne.set_data("START");

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "--------------------------" << std::endl;
    if (tethysMsgReceived)
      propellerCmds[1] = -20.0;
    else
      pub.Publish(msgTethys);

    if (daphneMsgReceived)
      propellerCmds[2] = -20.0;
    else
      pub.Publish(msgDaphne);

    for (int i = 0; i < ns.size(); i++)
    {
      msgs::Double propellerMsg;
      propellerMsg.set_data(propellerCmds[i]);
      propellerPubs[i].Publish(propellerMsg);

      std::cout << "Commanding thrust: " << propellerCmds[i]
        << " N" << std::endl;
    }
  }
}

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
 *   $ ./acoustic_comms_demo
 */

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs.hh>
#include <gz/transport.hh>

using namespace gz;

int main(int argc, char** argv)
{
  transport::Node node;

  auto propellerTopicTriton =
    gz::transport::TopicUtils::AsValidTopic(
      "/model/triton/joint/propeller_joint/cmd_pos");
  auto propellerPubTriton =
    node.Advertise<gz::msgs::Double>(propellerTopicTriton);

  auto propellerTopicTethys =
    gz::transport::TopicUtils::AsValidTopic(
      "/model/tethys/joint/propeller_joint/cmd_pos");
  auto propellerPubTethys =
    node.Advertise<gz::msgs::Double>(propellerTopicTethys);

  auto propellerTopicDaphne =
    gz::transport::TopicUtils::AsValidTopic(
      "/model/daphne/joint/propeller_joint/cmd_pos");
  auto propellerPubDaphne =
    node.Advertise<gz::msgs::Double>(propellerTopicDaphne);

  double propellerCmdTriton = -20;
  double propellerCmdTethys = 0;
  double propellerCmdDaphne = 0;

  // Setup publishers and callbacks for comms topics.
  auto senderAddressTriton = "1";

  std::string receiverAddressTethys = "2";
  std::string receiverAddressDaphne = "3";

  // Set up callbacks
  std::atomic<bool> tethysMsgReceived = false;
  std::atomic<bool> daphneMsgReceived = false;

  std::function<void(const msgs::Dataframe &)> cbTethys =
    [&](const msgs::Dataframe &_msg)
  {
    tethysMsgReceived = true;
  };

  std::function<void(const msgs::Dataframe &)> cbDaphne =
    [&](const msgs::Dataframe &_msg)
  {
    daphneMsgReceived = true;
  };

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
      propellerCmdTethys = -20.0;
    else
      pub.Publish(msgTethys);

    if (daphneMsgReceived)
      propellerCmdDaphne = -20.0;
    else
      pub.Publish(msgDaphne);

    msgs::Double propellerMsg;
    propellerMsg.set_data(propellerCmdTriton);
    propellerPubTriton.Publish(propellerMsg);

    propellerMsg.set_data(propellerCmdTethys);
    propellerPubTethys.Publish(propellerMsg);

    propellerMsg.set_data(propellerCmdDaphne);
    propellerPubDaphne.Publish(propellerMsg);

    std::cout << "Commanding thrust: " << std::endl;
    std::cout << "triton : " << propellerCmdTriton << " N" << std::endl;
    std::cout << "tethys : " << propellerCmdTethys << " N" << std::endl;
    std::cout << "daphne : " << propellerCmdDaphne << " N" << std::endl;
  }
}

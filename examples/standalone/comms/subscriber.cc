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

// Example: ./subscriber addr1 box1 addr1/rx

#include <iostream>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

//////////////////////////////////////////////////
/// \brief Usage function.
void usage()
{
  std::cerr << "./subscriber <src_address> <model_name> <callback_topic>"
            << std::endl;
}

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const ignition::msgs::Dataframe &_msg)
{
  std::cout << "Msg: " << _msg.data() << std::endl << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc != 4)
  {
    usage();
    return -1;
  }

  // Create a transport node.
  ignition::transport::Node node;
  std::string addr  = argv[1];
  std::string model = argv[2];
  std::string topic = argv[3];

  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
    return -1;
  }

  // Prepare the bind parameters.
  ignition::msgs::StringMsg_V bindReq;
  bindReq.add_data(addr);
  bindReq.add_data(model);
  bindReq.add_data(topic);

  // Bind.
  // if (!node.Request("/broker/bind", bindReq))
  //   std::cerr << "Bind call failed" << std::endl;

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  // Prepare the unbind parameters.
  ignition::msgs::StringMsg_V unbindReq;
  unbindReq.add_data(addr);
  unbindReq.add_data(topic);

  // Unbind.
  // if (!node.Request("/broker/unbind", unbindReq))
  //   std::cerr << "Unbind call failed" << std::endl;
}

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

// Example: ./publisher addr2

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include <gz/msgs/dataframe.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> g_terminatePub(false);

//////////////////////////////////////////////////
/// \brief Usage function.
void usage()
{
  std::cerr << "./publisher <dst_address>" << std::endl;
}

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc != 2)
  {
    usage();
    return -1;
  }

  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create a transport node and advertise a topic.
  gz::transport::Node node;
  std::string topic = "/broker/msgs";

  auto pub = node.Advertise<gz::msgs::Dataframe>(topic);
  if (!pub)
  {
    std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Prepare the message.
  gz::msgs::Dataframe msg;
  msg.set_src_address("addr1");
  msg.set_dst_address(argv[1]);

  // Publish messages at 1Hz.
  int counter = 0;
  while (!g_terminatePub)
  {
    // Prepare the payload.
    gz::msgs::StringMsg payload;
    payload.set_data("hello world " + std::to_string(counter));
    std::string serializedData;
    if (!payload.SerializeToString(&serializedData))
    {
      std::cerr << "Error serializing message\n"
                << payload.DebugString() << std::endl;
    }
    msg.set_data(serializedData);

    if (!pub.Publish(msg))
      break;

    ++counter;

    std::cout << "Publishing hello on topic [" << topic << "]" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}

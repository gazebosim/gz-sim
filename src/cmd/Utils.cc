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
#include "Utils.hh"

#include <iostream>

#include <ignition/msgs/stringmsg_v.pb.h>
#include <ignition/transport/Node.hh>

using namespace ignition;

std::string ignition::gazebo::cmd::getWorldName()
{
  // Create a transport node.
  transport::Node node;

  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/gazebo/worlds"};

  // Request and block
  msgs::StringMsg_V res;

  if (!node.Request(service, timeout, res, result))
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return "";
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return "";
  }

  return res.data().Get(0);
}

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

#include <iostream>
#include <gz/msgs/scene.pb.h>
#include <gz/transport/Node.hh>

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cout << "Usage: `./scene_requester <service> <type>`" << std::endl
              << "Where type can be 'info' or 'graph'" << std::endl;
    return -1;
  }

  // Get arguments
  std::string service = argv[1];
  std::string type = argv[2];

  if (type != "info" && type != "graph")
  {
    std::cout << "Usage: `./scene_requester <service> <type>`" << std::endl
              << "Where type can be 'info' or 'graph'" << std::endl;
    return -1;
  }

  // Create a transport node.
  gz::transport::Node node;

  bool executed{false};
  bool result{false};
  unsigned int timeout{5000};

  std::cout << std::endl << "Requesting scene " << type << " from [" << service
            << "]..." << std::endl << std::endl;

  // Request and block
  if (type == "graph")
  {
    gz::msgs::StringMsg res;
    executed = node.Request(service, timeout, res, result);

    if (executed && result)
      std::cout << res.data() << std::endl;
  }
  else
  {
    gz::msgs::Scene res;
    executed = node.Request(service, timeout, res, result);

    if (executed && result)
      std::cout << res.DebugString() << std::endl;
  }

  if (!executed)
    std::cerr << std::endl << "Service call timed out" << std::endl;
  else if (!result)
    std::cout << std::endl<< "Service call failed" << std::endl;
}

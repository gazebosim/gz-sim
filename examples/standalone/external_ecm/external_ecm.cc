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

#include <iostream>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/msgs/serialized.pb.h>
#include <ignition/transport/Node.hh>

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: `./external_ecm <world name>`" << std::endl;
    return -1;
  }

  // Get arguments
  std::string world = argv[1];

  // Create a transport node.
  ignition::transport::Node node;

  bool executed{false};
  bool result{false};
  unsigned int timeout{5000};
  std::string service{"/world/" + world + "/state"};

  std::cout << std::endl << "Requesting state for world [" << world
            << "] on service [" << service << "]..." << std::endl << std::endl;

  // Request and block
  ignition::msgs::SerializedStepMap res;
  executed = node.Request(service, timeout, res, result);

  if (!executed)
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return -1;
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return -1;
  }

  // Instantiate an ECM and populate with data from message
  ignition::gazebo::EntityComponentManager ecm;
  ecm.SetState(res.state());

  // Print some information
  ecm.Each<ignition::gazebo::components::Name>(
      [&](const ignition::gazebo::Entity &_entity,
          const ignition::gazebo::components::Name *_name) -> bool
  {
    auto parentComp =
        ecm.Component<ignition::gazebo::components::ParentEntity>(_entity);

    std::string parentInfo;
    if (parentComp)
    {
      auto parentNameComp =
          ecm.Component<ignition::gazebo::components::Name>(
          parentComp->Data());

      if (parentNameComp)
      {
        parentInfo += parentNameComp->Data() + " ";
      }
      parentInfo += "[" + std::to_string(parentComp->Data()) + "]";
    }

    std::cout << "Entity [" << _entity << "]" << std::endl
              << "  - Name: " << _name->Data() << std::endl
              << "  - Parent: " << parentInfo << std::endl;

    return true;
  });
}

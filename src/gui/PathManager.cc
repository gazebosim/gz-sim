/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "PathManager.hh"

#include <gz/msgs/sdf_generator_config.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/gui/Application.hh>

#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace sim::gui;

//////////////////////////////////////////////////
void onAddResourcePaths(const msgs::StringMsg_V &_msg)
{
  std::vector<std::string> paths;
  for (auto i = 0; i < _msg.data().size(); ++i)
  {
    paths.push_back(_msg.data(i));
  }

  addResourcePaths(paths);
}

//////////////////////////////////////////////////
void onAddResourcePaths(const msgs::StringMsg_V &_res, const bool _result)
{
  if (!_result)
  {
    gzerr << "Failed to get resource paths through service" << std::endl;
    return;
  }
  gzdbg << "Received resource paths." << std::endl;

  onAddResourcePaths(_res);
}

/////////////////////////////////////////////////
PathManager::PathManager()
{
  // Trigger an initial request to get all paths from server
  std::string service{"/gazebo/resource_paths/get"};

  gzdbg << "Requesting resource paths through [" << service << "]"
         << std::endl;
  this->node.Request(service, onAddResourcePaths);

  // Get path updates through this topic
  this->node.Subscribe("/gazebo/resource_paths", onAddResourcePaths);
}

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

#include <ignition/msgs/sdf_generator_config.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>

#include "ignition/gazebo/Util.hh"
#include "PathManager.hh"

using namespace ignition;
using namespace gazebo;
using namespace gazebo::gui;

//////////////////////////////////////////////////
void onAddResourcePaths(const msgs::StringMsg_V &_msg)
{
igndbg << "A" << std::endl;
  std::vector<std::string> paths;
  for (auto i = 0; i < _msg.data().size(); ++i)
  {
    paths.push_back(_msg.data(i));
  }

  addResourcePaths(paths);
}

/////////////////////////////////////////////////
PathManager::PathManager()
{
  // Get resource paths
  std::string service{"/gazebo/resource_paths/get"};
  msgs::StringMsg_V res;
  bool result{false};
  auto executed = this->node.Request(service, 5000, res, result);

  if (!executed)
    ignerr << "Service call timed out for [" << service << "]" << std::endl;
  else if (!result)
    ignerr << "Service call failed for [" << service << "]" << std::endl;

  onAddResourcePaths(res);

igndbg << "A" << std::endl;
  this->node.Subscribe("/gazebo/resource_paths", onAddResourcePaths);
}


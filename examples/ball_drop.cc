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

#include <ignition/gazebo.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>

namespace ign = ignition;

int main()
{
  // Create an SDF model programmatically.
  sdf::Model model;
  model.SetName("ball");
  sdf::Link *link = model.AddLink({"link", ign::math::Pose3d::Zero});

  // Create the Gazebo server
  ign::gazebo::Server server;

  // Add a ball model to the server
  ign::gazebo::Entity ball = server.CreateEntity(model);

  // Run the server
  return server.Step(1);
}

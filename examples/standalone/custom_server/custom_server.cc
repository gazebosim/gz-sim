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

#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>

/////////////////////////////////////////////////
int main()
{
  // Set verbosity level - defaults to 1 if unset
  gz::common::Console::SetVerbosity(4);

  // Object to pass custom configuration to the server
  gz::sim::ServerConfig serverConfig;

  // Populate with some configuration, for example, the SDF file to load
  serverConfig.SetSdfFile("shapes.sdf");

  // Instantiate server
  gz::sim::Server server(serverConfig);

  // Run the server unpaused for 100 iterations, blocking
  server.Run(true /*blocking*/, 100 /*iterations*/, false /*paused*/);

  return 0;
}

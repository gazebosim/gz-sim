/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <array>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/world_stats.pb.h>

#include <gz/math/Stopwatch.hh>
#include <gz/common/Console.hh>

#include "gz/transport/Node.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"  // NOLINT(build/include)

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
int main(int _argc, char** _argv)
{
  common::Console::SetVerbosity(4);

  std::string sdfFile{""};
  if (_argc >= 2)
  {
    sdfFile = _argv[1];
  }
  gzdbg << "SDF file: " << sdfFile << std::endl;

  unsigned int iterations{10000};
  if (_argc >= 3)
  {
    iterations = atoi(_argv[2]);
  }
  gzdbg << "Iterations: " << iterations << std::endl;

  double updateRate{-1};
  if (_argc >= 4)
  {
    updateRate = atoi(_argv[3]);
  }
  gzdbg << "Update rate: " << updateRate << std::endl;

  ServerConfig serverConfig;
  if (!serverConfig.SetSdfFile(sdfFile))
  {
    gzerr << "Failed to set SDF file [" << sdfFile << "]" << std::endl;
    return -1;
  }

  // Set the update rate.
  if (updateRate > 0.0)
    serverConfig.SetUpdateRate(updateRate);

  // Create the Gazebo server
  Server server(serverConfig);

  transport::Node node;

  std::vector<msgs::Clock> msgs;
  msgs.reserve(iterations);

  std::function<void(const msgs::Clock&)> cb =
    [&](const msgs::Clock &_msg)
    {
      msgs.push_back(_msg);
    };

  double progress = 0;

  std::function<void(const msgs::WorldStatistics &)> cb2 =
    [&](const msgs::WorldStatistics &_msg)
    {
      double nIters = static_cast<double>(_msg.iterations());
      nIters = nIters / iterations * 100;
      if (nIters >= progress)
      {
        std::cout << "Simulation Progress: " << nIters << "%" << std::endl;
        progress += 1.0;
      }
    };

  node.Subscribe("/clock", cb);
  node.Subscribe("/stats", cb2);

  // Run the server
  server.Run(true, iterations, false);

  std::ofstream ofs("data.csv", std::ofstream::out);

  ofs << "# Filename: " << sdfFile << std::endl;
  ofs << "# Iterations: " << iterations << std::endl;
  ofs << "# Rate: " << updateRate << std::endl;
  ofs << "# Real s, Real ns, sim s, sim ns" << std::endl;

  for (auto &msg : msgs)
  {
    ofs << msg.real().sec() << ", " << msg.real().nsec() << ", "
        << msg.sim().sec() << ", " << msg.sim().nsec() << std::endl;
  }

  return 0;
}

/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <algorithm>
#include <atomic>
#include <cctype>
#include <iostream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/SignalHandler.hh>
#include <gz/sim/config.hh>
#include <gz/utils/cli/CLI.hpp>

#include "LockStepRuntime.hh"
#include "RuntimeConfig.hh"

using namespace gz;

namespace {

std::string MakeAlNumWithUnderscore(const std::string& _input)
{
  std::string output = _input;
  std::replace_if(output.begin(), output.end(),
                  [](unsigned char _c){ return !std::isalnum(_c); },
                  '_');
  return output;
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app("Gz sim lock step plugin runtime");
  std::string configFilename;
  app.add_option(
         "config_filename", configFilename,
         "Path to runtime config file. Cannot be empty")
      ->required();
  app.add_flag_callback("-v,--verbose",
                        [] { gz::common::Console::SetVerbosity(4); });

  CLI11_PARSE(app, argc, argv);

  if (configFilename.empty())
  {
    // Use of std::cerr is intentional here since the log file has not yet been
    // initialized.
    std::cerr << "Config filename cannot be empty. "
                 "Call with --help for usage.\n";
    return 0;
  }

  // Initialize logger
  std::string logPath;
  common::env(GZ_HOMEDIR, logPath);
  logPath = common::joinPaths(logPath, ".gz", "sim", "runtime");
  common::createDirectories(logPath);
  std::string logFilename = MakeAlNumWithUnderscore(configFilename);
  logFilename += ".log";
  gzLogInit(logPath, logFilename);

  gzmsg << "Starting lock step System runtime with config from "
        << configFilename << std::endl;

  // Load runtime config.
  sim::RuntimeConfig config = sim::parseRuntimeConfig(configFilename);
  if (config.plugin.Filename().empty())
  {
    gzerr << "Failed to load plugin from runtime config. Aborting.\n";
    return 1;
  }

  // Create runtime.
  std::unique_ptr<sim::LockStepRuntime> runtime =
      sim::LockStepRuntime::Create(std::move(config));
  if (runtime == nullptr)
  {
    gzerr << "Failed to create runtime from config. Aborting.\n";
    return 1;
  }

  // Periodically publish stats until stopped.
  std::atomic<bool> stopped{false};
  gz::common::SignalHandler handler;
  handler.AddCallback([&stopped] (int _sig)
  {
    stopped = true;
  });
  while (!stopped)
  {
    runtime->PublishStats();
  }

  gzmsg << "Exiting runtime.\n";
  return 0;
}

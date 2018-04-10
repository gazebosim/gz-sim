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

#include "SigHandler.hh"
#include <signal.h>
#include <functional>
#include <vector>
#include <ignition/common/Console.hh>

using namespace ignition;
using namespace gazebo;

// A wrapper for the sigaction sa_handler.
// This will allow us to avoid static variables.
std::function<void(int)> onSignalWrapper;
void onSignal(int _value)
{
  onSignalWrapper(_value);
}

/////////////////////////////////////////////////
SigHandler::SigHandler()
{
  onSignalWrapper = std::bind(&SigHandler::OnSignal, this,
    std::placeholders::_1);

#ifndef _WIN32
  /// \todo(nkoenig) Add a mock interface to test failure of signal creation.
  struct sigaction sigact;
  sigact.sa_flags = 0;
  sigact.sa_handler = onSignal;
  if (sigemptyset(&sigact.sa_mask) != 0)
    std::cerr << "sigemptyset failed while setting up for SIGINT" << std::endl;

  if (sigaction(SIGINT, &sigact, nullptr))
  {
    ignerr << "Unable to catch SIGINT.\n"
      << " Please visit http://community.gazebosim.org for help.\n";
    return;
  }
  if (sigaction(SIGTERM, &sigact, nullptr))
  {
    ignerr << "Unable to catch SIGTERM.\n";
    return;
  }
#endif

  this->initialized = true;
}

//////////////////////////////////////////////////
bool SigHandler::Initialized() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
void SigHandler::OnSignal(int _sig)
{
  igndbg << "Received signal[" << _sig << "]. Quitting.\n";
  for (std::function<void(int)> cb : this->callbacks)
  {
    cb(_sig);
  }
}

/////////////////////////////////////////////////
void SigHandler::AddCallback(std::function<void(int)> _cb)
{
  this->callbacks.push_back(_cb);
}

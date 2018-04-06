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

#include "ServerPrivate.hh"
#include <signal.h>
#include <ignition/common/Console.hh>

using namespace ignition;
using namespace gazebo;

// By default running is false
std::atomic<bool> ServerPrivate::running(false);

/////////////////////////////////////////////////
ServerPrivate::~ServerPrivate()
{
  if (this->runThread.joinable())
  {
    this->running = false;
    this->runThread.join();
  }
}

/////////////////////////////////////////////////
void ServerPrivate::UpdateSystems()
{
  /// \todo(nkoenig) Update systems
}

/////////////////////////////////////////////////
void ServerPrivate::Run(const uint64_t _iterations)
{
#ifndef _WIN32
  /// \todo(nkoenig) Create an adaptor class for signal handling. This will
  /// allow a mock interface to be created for testing purposes.
  struct sigaction sigact;
  sigact.sa_flags = 0;
  sigact.sa_handler = ServerPrivate::SigHandler;
  if (sigemptyset(&sigact.sa_mask) != 0)
    std::cerr << "sigemptyset failed while setting up for SIGINT" << std::endl;

  if (sigaction(SIGINT, &sigact, NULL))
  {
    std::cerr << "Stopping. Unable to catch SIGINT.\n"
      << " Please visit http://gazebosim.org/support.html for help.\n";
    return;
  }
  if (sigaction(SIGTERM, &sigact, NULL))
  {
    std::cerr << "Stopping. Unable to catch SIGTERM.\n";
    return;
  }
#endif

  this->running = true;

  uint64_t startingIterations = this->iterations;
  // Execute all the system until we are told to stop, or the number of
  // iterations is reached.
  for (; this->running && (_iterations == 0 ||
                           this->iterations < _iterations + startingIterations);
       ++this->iterations)
  {
    this->UpdateSystems();
  }
  this->running = false;
}

//////////////////////////////////////////////////
bool ServerPrivate::SceneService(ignition::msgs::Scene &_rep)
{
  /// \todo(nkoenig) Replace hardcoded values.
  _rep.set_name("gazebo");
  ignition::msgs::Model *model = _rep.add_model();

  model->set_name("sphere");
  model->set_id(0);
  ignition::msgs::Set(model->mutable_pose(),
                      ignition::math::Pose3d(0, 0, 1, 0, 0, 0));

  ignition::msgs::Link *link = model->add_link();
  link->set_name("link");

  ignition::msgs::Visual *visual = link->add_visual();
  visual->set_name("visual");

  ignition::msgs::Geometry *geom = visual->mutable_geometry();
  geom->set_type(ignition::msgs::Geometry::SPHERE);
  ignition::msgs::SphereGeom *sphere = geom->mutable_sphere();
  sphere->set_radius(1.0);

  return true;
}

//////////////////////////////////////////////////
void ServerPrivate::SigHandler(int _sig)
{
  igndbg << "Received signal[" << _sig << "]. Quitting.\n";
  running = false;
}

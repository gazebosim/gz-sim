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
#include <signal.h>
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>
#include <atomic>
#include <thread>
#include <vector>


#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/PhysicsSystem.hh"
#include "ignition/gazebo/Entity.hh"

using namespace ignition::gazebo;

class ignition::gazebo::ServerPrivate
{
  public: ServerPrivate() = default;
  public: ~ServerPrivate();

  /// \brief Update all the systems
  public: void UpdateSystems();

  /// \brief Run the server.
  public: void Run();

  public: bool SceneService(ignition::msgs::Scene &_rep);

  // cppcheck-suppress unusedPrivateFunction
  private: static void SigHandler(int)
  {
    running = false;
  }

  /// \brief This is used to indicate that Run has been called, and the
  /// server is in the run state.
  public: static std::atomic<bool> running;

  public: std::thread runThread;

  public: std::vector<std::unique_ptr<System>> systems;

  public: std::vector<Entity> entities;

  /// \brief Communication node.
  public: ignition::transport::Node node;
};

std::atomic<bool> ServerPrivate::running(false);

/////////////////////////////////////////////////
Server::Server()
  : dataPtr(new ServerPrivate)
{
  this->dataPtr->systems.push_back(
      std::unique_ptr<System>(new PhysicsSystem));

  this->dataPtr->node.Advertise("/ign/gazebo/scene",
      &ServerPrivate::SceneService, this->dataPtr.get());
}
/////////////////////////////////////////////////
Server::~Server()
{
}

/////////////////////////////////////////////////
Entity Server::CreateEntity(const sdf::Model & /*_model*/)
{
  Entity entity;

  // Notify systems that an entity has been created.
  // \todo: We could move this into batch style process that happens
  // once before systems are updated.
  for (std::unique_ptr<System> &system : this->dataPtr->systems)
  {
    system->EntityCreated(entity);
  }

  this->dataPtr->entities.push_back(entity);
  return this->dataPtr->entities.back();
}

/////////////////////////////////////////////////
void Server::Run(const bool _blocking)
{
  if (_blocking)
    this->dataPtr->Run();
  else
    this->dataPtr->runThread =
      std::thread(&ServerPrivate::Run, this->dataPtr.get());
}

/////////////////////////////////////////////////
bool Server::Step(const unsigned int _iterations)
{
  if (!this->dataPtr->running)
  {
    // For each iteration ...
    for (unsigned int iter = 0; iter < _iterations; ++iter)
    {
      this->dataPtr->UpdateSystems();
    }
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
/// Server Private functions
/////////////////////////////////////////////////

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
  // Update systems
  for (std::unique_ptr<System> &system : this->systems)
    system->Update();
}

/////////////////////////////////////////////////
void ServerPrivate::Run()
{
#ifndef _WIN32
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
  while (this->running)
  {
    this->UpdateSystems();
  }
}

//////////////////////////////////////////////////
bool ServerPrivate::SceneService(ignition::msgs::Scene &_rep)
{
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

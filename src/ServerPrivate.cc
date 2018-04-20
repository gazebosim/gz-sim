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

#include <functional>
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include "ignition/gazebo/TestSystem.hh"
#include "ignition/gazebo/ComponentFactory.hh"
#include "SignalHandler.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
ServerPrivate::ServerPrivate()
{
  // Windows does not support setting std::atmoic in the class definition.
  // By default running is false
  this->running = false;
  this->sigHandler.AddCallback(
      std::bind(&ServerPrivate::OnSignal, this, std::placeholders::_1));

  // \todo(nkoenig) Remove this once we can dynamically load systems.
  this->systems.push_back(std::unique_ptr<System>(new TestSystem));

  // This is the scene service
  this->node.Advertise("/ign/gazebo/scene", &ServerPrivate::SceneService, this);
}

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
  if (!this->sigHandler.Initialized())
  {
    ignerr << "Signal handlers were not created. The server won't run.\n";
    return;
  }

  this->runMutex.lock();
  // Can't run twice.
  if (this->running)
  {
    ignwarn << "The server is already runnnng.\n";
    return;
  }
  this->running = true;
  this->runMutex.unlock();


  uint64_t startingIterations = this->iterations;
  // Execute all the systems until we are told to stop, or the number of
  // iterations is reached.
  for (; this->running && (_iterations == 0 ||
                           this->iterations < _iterations + startingIterations);
       ++this->iterations)
  {
    this->UpdateSystems();
  }

  this->runMutex.lock();
  this->running = false;
  this->runMutex.unlock();
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
void ServerPrivate::OnSignal(int _sig)
{
  igndbg << "Server received signal[" << _sig  << "]\n";
  this->running = false;
}

//////////////////////////////////////////////////
void ServerPrivate::EraseEntities()
{
  this->entities.clear();
}

//////////////////////////////////////////////////
void ServerPrivate::CreateEntities(const sdf::Root &_root)
{
  for (uint64_t i = 0; i < _root.ModelCount(); ++i)
  {
    /// \todo(nkoenig) This should use free entity slots.
    EntityId entityId = this->entities.size();
    this->entities.push_back(Entity(entityId));

    // Get the SDF model
    const sdf::Model *model = _root.ModelByIndex(i);

    // Create the pose component for the model.
    ComponentKey compKey = this->componentFactory.CreateComponent(
        model->Pose());
    std::cout << "Created component[" << compKey.first << ":"
              << compKey.second << "]\n";

    this->entityComponents[entityId].push_back(compKey);
  }

  // Never compare to zero.
  for (const std::pair<EntityId, std::vector<ComponentKey>> &ec :
       this->entityComponents)
  {
    for (ComponentKey compKey : ec.second)
    {
      std::cout << *this->componentFactory.Component<ignition::math::Pose3d>(
          compKey) << std::endl;
    }
  }
}

/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <iostream>
#include <mutex>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>

#include "ApplyForceTorque.hh"

namespace gz
{
namespace sim
{
  class ApplyForceTorquePrivate
  {
    /// \brief X component of force
    // TODO: change to Vector3d
    public: float forceX{0.0};

    /// \brief Y component of force
    public: float forceY{0.0};
    
    /// \brief Z component of force
    public: float forceZ{0.0};
    
    /// \brief X component of torque
    public: float torqueX{0.0};
    
    /// \brief Y component of torque
    public: float torqueY{0.0};
    
    /// \brief Z component of torque
    public: float torqueZ{0.0};

    /// \brief True if a force should be applied
    public: bool applyForce{false};

    /// \brief True if a torque should be applied
    public: bool applyTorque{false};

    /// \brief A mutex to protect wrenches
    public: std::mutex mutex;
  };
}
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
ApplyForceTorque::ApplyForceTorque()
  : GuiSystem(), dataPtr(std::make_unique<ApplyForceTorquePrivate>())
{
}

/////////////////////////////////////////////////
ApplyForceTorque::~ApplyForceTorque() = default;

/////////////////////////////////////////////////
void ApplyForceTorque::LoadConfig(const tinyxml2::XMLElement */*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "Apply force and torque";
}

/////////////////////////////////////////////////
void ApplyForceTorque::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->applyForce)
  {
    std::string entityName = "box";
    auto entities = entitiesFromScopedName(entityName, _ecm);
    if (entities.empty())
    {
      gzerr << "No entity named [" << entityName << "]" << std::endl;
    }
    auto entity = *entities.begin();

    Model model(entity);
    if (!model.Valid(_ecm))
    {
      gzerr << "Entity is not a model." << std::endl;
    }

    Link link(model.CanonicalLink(_ecm));
    math::Vector3d force{10000.0, 0.0, 0.0};
    link.AddWorldForce(_ecm, force);

    this->dataPtr->applyForce = false;
    gzdbg << "Applied force to " << entityName << std::endl;
  }
}

/////////////////////////////////////////////////
void ApplyForceTorque::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &/*_ecm*/)
{
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateForceX(float _forceX)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->forceX = _forceX;
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateForceY(float _forceY)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->forceY = _forceY;
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateForceZ(float _forceZ)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->forceZ = _forceZ;
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateTorqueX(float _torqueX)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->torqueX = _torqueX;
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateTorqueY(float _torqueY)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->torqueY = _torqueY;
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateTorqueZ(float _torqueZ)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->torqueZ = _torqueZ;
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyForce()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  
  gzdbg << "Force: (" << this->dataPtr->forceX << ", " <<
          this->dataPtr->forceY << ", " <<
          this->dataPtr->forceZ <<
          ")" << std::endl;
  if (this->dataPtr->applyForce)
  {
    gzdbg << "Previous force not yet applied" << std::endl;
  }
  else
  {
    gzdbg << "Applying force" << std::endl;
    this->dataPtr->applyForce = true;
  }
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyTorque()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  
  gzdbg << "Torque: (" << this->dataPtr->torqueX << ", " <<
          this->dataPtr->torqueY << ", " <<
          this->dataPtr->torqueZ <<
          ")" << std::endl;
  if (this->dataPtr->applyTorque)
  {
    gzdbg << "Previous torque not yet applied" << std::endl;
  }
  else
  {
    gzdbg << "Applying torque" << std::endl;
    this->dataPtr->applyTorque = true;
  }
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyAll()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  
  gzdbg << "Force: (" << this->dataPtr->forceX << ", " <<
          this->dataPtr->forceY << ", " <<
          this->dataPtr->forceZ <<
          ") Torque: (" << this->dataPtr->torqueX << ", " <<
          this->dataPtr->torqueY << ", " <<
          this->dataPtr->torqueZ <<
          ")" << std::endl;
  if (this->dataPtr->applyForce || this->dataPtr->applyTorque)
  {
    gzdbg << "Previous wrench not yet applied" << std::endl;
  }
  else
  {
    gzdbg << "Applying wrench" << std::endl;
    this->dataPtr->applyForce = true;
    this->dataPtr->applyTorque = true;
  }
}

// Register this plugin
GZ_ADD_PLUGIN(ApplyForceTorque,
              gz::gui::Plugin,
              System,
              ApplyForceTorque::ISystemPreUpdate);

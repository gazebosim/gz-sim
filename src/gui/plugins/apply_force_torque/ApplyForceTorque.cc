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
#include <gz/sim/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/entity.pb.h>

#include "ApplyForceTorque.hh"

namespace gz
{
namespace sim
{
  class ApplyForceTorquePrivate
  {
    /// \brief Publish wrench messages
    public: void Publish(bool _applyForce, bool _applyTorque);

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Publisher for EntityWrench messages
    public: transport::Node::Publisher pub;

    /// \brief A mutex to protect wrenches
    public: std::mutex mutex;

    /// \brief Force to be applied
    public: math::Vector3d force{0.0, 0.0, 0.0};

    /// \brief Torque to be applied
    public: math::Vector3d torque{0.0, 0.0, 0.0};

    /// \brief Entity to which the wrench should be applied
    public: Entity entity;
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

  // if (this->dataPtr->applyForce)
  // {
  //   std::string entityName = "box";
  //   auto entities = entitiesFromScopedName(entityName, _ecm);
  //   if (entities.empty())
  //   {
  //     gzerr << "No entity named [" << entityName << "]" << std::endl;
  //   }
  //   auto entity = *entities.begin();

  //   Model model(entity);
  //   if (!model.Valid(_ecm))
  //   {
  //     gzerr << "Entity is not a model." << std::endl;
  //   }

  //   Link link(model.CanonicalLink(_ecm));
  //   math::Vector3d force{10000.0, 0.0, 0.0};
  //   link.AddWorldForce(_ecm, force);

  //   this->dataPtr->applyForce = false;
  //   gzdbg << "Applied force to " << entityName << std::endl;
  // }
}

/////////////////////////////////////////////////
void ApplyForceTorque::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  // Get entity to apply force to
  if (this->dataPtr->entity == kNullEntity)
  {
    std::string entityName = "cylinder";
    auto entities = entitiesFromScopedName(entityName, _ecm);
    this->dataPtr->entity = *entities.begin();
  }

  // Create publisher if not yet created
  if (!this->dataPtr->pub.Valid())
  {
    std::string worldName{""};
    _ecm.Each<components::World, components::Name>(
        [&](const Entity &_entity,
          const components::World * /* _world */ ,
          const components::Name *_name)->bool
        {
          World world(_entity);
          for (auto &model : world.Models(_ecm))
          {
            if (model == this->dataPtr->entity)
            {
              worldName = _name->Data();
              return true;
            }
          }
          gzerr << "World not found" << std::endl;
          return false;
        });

    auto topic = transport::TopicUtils::AsValidTopic(
      "/world/" + worldName + "/wrench");
    this->dataPtr->pub = this->dataPtr->node.Advertise<msgs::EntityWrench>(topic);
    gzdbg << "Created publisher to " << topic << std::endl;
  }
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateForce(double _x, double _y, double _z)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->force = {_x, _y, _z};
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateTorque(double _x, double _y, double _z)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->torque = {_x, _y, _z};
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyForce()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  gzdbg << "Force: (" << this->dataPtr->force[0] << ", " <<
          this->dataPtr->force[1] << ", " <<
          this->dataPtr->force[2] <<
          ")" << std::endl;
  this->dataPtr->Publish(true, false);
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyTorque()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  
  gzdbg << "Torque: (" << this->dataPtr->torque[0] << ", " <<
          this->dataPtr->torque[1] << ", " <<
          this->dataPtr->torque[2] <<
          ")" << std::endl;
  this->dataPtr->Publish(false, true);
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyAll()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  gzdbg << "Force: (" << this->dataPtr->force[0] << ", " <<
          this->dataPtr->force[1] << ", " <<
          this->dataPtr->force[2] <<
          ") Torque: (" << this->dataPtr->torque[0] << ", " <<
          this->dataPtr->torque[1] << ", " <<
          this->dataPtr->torque[2] <<
          ")" << std::endl;
  this->dataPtr->Publish(true, true);
}

/////////////////////////////////////////////////
void ApplyForceTorquePrivate::Publish(bool _applyForce, bool _applyTorque)
{
  msgs::EntityWrench msg;

  msg.mutable_entity()->set_id(this->entity);

  math::Vector3d zeros{0.0, 0.0, 0.0};
  msgs::Set(msg.mutable_wrench()->mutable_force(),
      _applyForce ? this->force : zeros);
  msgs::Set(msg.mutable_wrench()->mutable_torque(),
      _applyTorque ? this->torque : zeros);

  this->pub.Publish(msg);
}

// Register this plugin
GZ_ADD_PLUGIN(ApplyForceTorque,
              gz::gui::Plugin,
              System,
              ApplyForceTorque::ISystemPreUpdate);

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

#include "Contact.hh"

#include <gz/msgs/contact.pb.h>
#include <gz/msgs/contacts.pb.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

#include <gz/transport/Node.hh>

#include "gz/sim/Conversions.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class ContactSensor
{
  /// \brief Load the Contact sensor from an sdf element
  /// \param[in] _sdf SDF element describing the Contact sensor
  /// \param[in] _topic string with topic name
  /// \param[in] _collisionEntities A list of entities that act as contact
  /// sensors
  public: void Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                    const std::vector<Entity> &_collisionEntities);

  /// \brief Add contacts to the list to be published
  /// \param[in] _stamp Time stamp of the sensor measurement
  /// \param[in] _contacts A contact message to be added to the list
  public: void AddContacts(const std::chrono::steady_clock::duration &_stamp,
                           const msgs::Contacts &_contacts);

  /// \brief Publish sensor data over gz transport
  public: void Publish();

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief Message to publish
  public: msgs::Contacts contactsMsg;

  /// \brief Gazebo transport node
  public: transport::Node node;

  /// \brief Gazebo transport publisher
  public: transport::Node::Publisher pub;

  /// \brief Entities for which this sensor publishes data
  public: std::vector<Entity> collisionEntities;
};

class gz::sim::systems::ContactPrivate
{
  /// \brief Create sensors that correspond to entities in the simulation
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateSensors(EntityComponentManager &_ecm);

  /// \brief Update and publish sensor data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateSensors(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm);

  /// \brief Remove sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveSensors(const EntityComponentManager &_ecm);

  /// \brief A map of Contact entity to its Contact sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<ContactSensor>> entitySensorMap;
};

//////////////////////////////////////////////////
void ContactSensor::Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                         const std::vector<Entity> &_collisionEntities)
{
  this->collisionEntities = _collisionEntities;

  auto contactElem = _sdf->GetElement("contact");
  auto tmpTopic =
      contactElem->Get<std::string>("topic", "__default_topic__").first;

  if (tmpTopic == "__default_topic__")
  {
    // use default topic for sensor
    this->topic = _topic;
  }
  else
  {
    this->topic = tmpTopic;
  }

  gzmsg << "Contact system publishing on " << this->topic << std::endl;
  this->pub = this->node.Advertise<msgs::Contacts>(this->topic);
}

//////////////////////////////////////////////////
void ContactSensor::AddContacts(
    const std::chrono::steady_clock::duration &_stamp,
    const msgs::Contacts &_contacts)
{
  auto stamp = convert<msgs::Time>(_stamp);
  for (const auto &contact : _contacts.contact())
  {
    auto *newContact = this->contactsMsg.add_contact();
    newContact->CopyFrom(contact);
    newContact->mutable_header()->mutable_stamp()->CopyFrom(stamp);
  }

  this->contactsMsg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
}

//////////////////////////////////////////////////
void ContactSensor::Publish()
{
  // Only publish if there are contacts
  if (this->contactsMsg.contact_size() > 0)
  {
    this->pub.Publish(this->contactsMsg);
    this->contactsMsg.Clear();
  }
}

//////////////////////////////////////////////////
void ContactPrivate::CreateSensors(EntityComponentManager &_ecm)
{
  GZ_PROFILE("ContactPrivate::CreateSensors");
  _ecm.EachNew<components::ContactSensor>(
      [&](const Entity &_entity,
          const components::ContactSensor *_contact) -> bool
      {
        // Check if the parent entity is a link
        auto *parentEntity = _ecm.Component<components::ParentEntity>(_entity);
        if (nullptr == parentEntity)
          return true;

        auto *linkComp = _ecm.Component<components::Link>(parentEntity->Data());
        if (nullptr == linkComp)
        {
          // Contact sensors should only be attached to links
          return true;
        }

        auto collisionElem =
            _contact->Data()->GetElement("contact")->GetElement("collision");

        std::vector<Entity> collisionEntities;
        // Get all the collision elements
        for (; collisionElem;
             collisionElem = collisionElem->GetNextElement("collision"))
        {
          auto collisionName = collisionElem->Get<std::string>();
          // Get collision entity that matches the name given by the sensor's
          // configuration.
          auto childEntities = _ecm.ChildrenByComponents(
              parentEntity->Data(), components::Collision(),
              components::Name(collisionName));

          if (!childEntities.empty())
          {
            // We assume that if childEntities is not empty, it only has one
            // element.
            collisionEntities.push_back(childEntities.front());

            // Create component to be filled by physics.
            _ecm.CreateComponent(childEntities.front(),
                                 components::ContactSensorData());
          }
        }

        std::string defaultTopic = scopedName(_entity, _ecm, "/") + "/contact";

        auto sensor = std::make_unique<ContactSensor>();
        sensor->Load(_contact->Data(), defaultTopic, collisionEntities);
        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void ContactPrivate::UpdateSensors(const UpdateInfo &_info,
                                   const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ContactPrivate::UpdateSensors");
  for (const auto &item : this->entitySensorMap)
  {
    for (const Entity &entity : item.second->collisionEntities)
    {
      auto contacts = _ecm.Component<components::ContactSensorData>(entity);

      // We will assume that the ContactData component will have been created if
      // this entity is in the collisionEntities list
      if (contacts->Data().contact_size() > 0)
      {
        item.second->AddContacts(_info.simTime, contacts->Data());
      }
    }
  }
}

//////////////////////////////////////////////////
void ContactPrivate::RemoveSensors(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ContactPrivate::RemoveSensors");
  _ecm.EachRemoved<components::ContactSensor>(
    [&](const Entity &_entity,
        const components::ContactSensor *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing Contact sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}
//////////////////////////////////////////////////
Contact::Contact() : System(), dataPtr(std::make_unique<ContactPrivate>())
{
}

//////////////////////////////////////////////////
void Contact::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  GZ_PROFILE("Contact::PreUpdate");
  this->dataPtr->CreateSensors(_ecm);
}

//////////////////////////////////////////////////
void Contact::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Contact::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (!_info.paused)
  {
    this->dataPtr->UpdateSensors(_info, _ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Publish sensor data
      it.second->Publish();
    }
  }

  this->dataPtr->RemoveSensors(_ecm);
}

GZ_ADD_PLUGIN(Contact, System,
  Contact::ISystemPreUpdate,
  Contact::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Contact, "gz::sim::systems::Contact")

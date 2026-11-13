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

#include <google/protobuf/arena.h>

#include <gz/msgs/contact.pb.h>
#include <gz/msgs/contacts.pb.h>

#include <chrono>
#include <memory>
#include <optional>
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

  /// \brief Publish the contacts gathered since the last publication over
  /// gz transport, if any, when the update period has elapsed.
  /// \param[in] _simTime Current simulation time
  public: void Publish(const std::chrono::steady_clock::duration &_simTime);

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief Arena for contact message allocation. Reset() is called after
  /// each publish, retaining the arena's initial block for reuse.
  public: google::protobuf::Arena arena;

  /// \brief Message to publish (arena-owned). Reallocated on the arena
  /// after every Reset().
  public: msgs::Contacts *contactsMsg{
#if GOOGLE_PROTOBUF_VERSION >= 4022000
    google::protobuf::Arena::Create<msgs::Contacts>(&arena)
#else
    google::protobuf::Arena::CreateMessage<msgs::Contacts>(&arena)
#endif
  };

  /// \brief Gazebo transport node
  public: transport::Node node;

  /// \brief Gazebo transport publisher
  public: transport::Node::Publisher pub;

  /// \brief Entities for which this sensor publishes data
  public: std::vector<Entity> collisionEntities;

  /// \brief Simulation time between two publications, from the sensor's
  /// <update_rate>. Zero publishes at every iteration with contacts.
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Simulation time of the next publication, empty before the
  /// first. It advances by whole update periods, so the average rate is the
  /// update rate even when the period is not a multiple of the physics step.
  public: std::optional<std::chrono::steady_clock::duration> nextPublishTime;
};

class gz::sim::systems::ContactPrivate
{
  /// \brief Create sensors that correspond to entities in the simulation
  /// \param[in] _ecm Mutable reference to ECM.
  /// \param[in] _includeExisting True to scan existing contact sensor
  /// components as well as newly-created ones.
  public: void CreateSensors(EntityComponentManager &_ecm,
              bool _includeExisting = false);

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

  /// \brief Resolved topic names
  public: std::unordered_map<Entity, std::string> resolvedTopicNames;
};

//////////////////////////////////////////////////
void ContactSensor::Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                         const std::vector<Entity> &_collisionEntities)
{
  this->collisionEntities = _collisionEntities;
  this->topic = _topic;

  const double updateRate = _sdf->Get<double>("update_rate", 0.0).first;
  if (updateRate < 0.0)
  {
    gzwarn << "Contact sensor on [" << this->topic << "] has a negative "
           << "<update_rate> [" << updateRate << "]; publishing at every "
           << "iteration with contacts." << std::endl;
  }
  else if (updateRate > 0.0)
  {
    this->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / updateRate));
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
    auto *newContact = this->contactsMsg->add_contact();
    newContact->CopyFrom(contact);
    newContact->mutable_header()->mutable_stamp()->CopyFrom(stamp);
  }
}

//////////////////////////////////////////////////
void ContactSensor::Publish(
    const std::chrono::steady_clock::duration &_simTime)
{
  if (this->updatePeriod > std::chrono::steady_clock::duration::zero())
  {
    // A jump back in time (before the previous publication) restarts the
    // schedule
    if (this->nextPublishTime &&
        _simTime < *this->nextPublishTime - this->updatePeriod)
    {
      this->nextPublishTime.reset();
    }

    // Keep gathering contacts until the next publication time
    if (this->nextPublishTime && _simTime < *this->nextPublishTime)
      return;

    // Advance by whole periods; a step longer than the period publishes at
    // every step instead of catching up with a burst
    this->nextPublishTime =
        this->nextPublishTime.value_or(_simTime) + this->updatePeriod;
    if (*this->nextPublishTime <= _simTime)
      this->nextPublishTime = _simTime + this->updatePeriod;
  }

  // Only publish if there are contacts
  if (this->contactsMsg->contact_size() > 0)
  {
    // The publication time; each contact keeps the time of its iteration
    this->contactsMsg->mutable_header()->mutable_stamp()->CopyFrom(
        convert<msgs::Time>(_simTime));
    this->pub.Publish(*this->contactsMsg);
    // Reset (not Clear): Clear() walks the field tree, while arena.Reset()
    // is O(1) and lets the next add_contact() bump-allocate fresh entries.
    this->arena.Reset();
#if GOOGLE_PROTOBUF_VERSION >= 4022000
    this->contactsMsg =
      google::protobuf::Arena::Create<msgs::Contacts>(&this->arena);
#else
    this->contactsMsg =
      google::protobuf::Arena::CreateMessage<msgs::Contacts>(&this->arena);
#endif
  }
}

//////////////////////////////////////////////////
void ContactPrivate::CreateSensors(EntityComponentManager &_ecm,
    bool _includeExisting)
{
  GZ_PROFILE("ContactPrivate::CreateSensors");

  auto createSensor =
    [&](const Entity &_entity,
        const components::ContactSensor *_contact) -> bool
    {
      if (this->entitySensorMap.find(_entity) != this->entitySensorMap.end())
        return true;

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
          if (nullptr == _ecm.Component<components::ContactSensorData>(
              childEntities.front()))
          {
            _ecm.CreateComponent(childEntities.front(),
                                 components::ContactSensorData());
          }
        }
      }

      // generate namespace
      std::string ns = scopedNamespace(_ecm, _entity);
      std::string defaultPrefix = scopedName(_entity, _ecm, "/");

      // check topic
      TopicNameOptions topicNameOptions;
      topicNameOptions.sdfElementName = "topic";
      topicNameOptions.topicNamespace = ns;
      topicNameOptions.defaultTopicPrefix = defaultPrefix;
      topicNameOptions.defaultTopicSuffix = "contact";

      auto contactElem = _contact->Data()->GetElement("contact");
      auto customTopic =
          contactElem->Get<std::string>("topic", "__default_topic__").first;
      auto topicElem = (customTopic == "__default_topic__") ?
          std::make_shared<sdf::Element>() : contactElem;
      auto topic = resolvedTopicName(topicElem, topicNameOptions);
      this->resolvedTopicNames.insert(std::make_pair(_entity, topic));

      auto sensor = std::make_unique<ContactSensor>();
      sensor->Load(_contact->Data(), topic, collisionEntities);
      this->entitySensorMap.insert(
          std::make_pair(_entity, std::move(sensor)));

      return true;
    };

  if (_includeExisting)
  {
    _ecm.Each<components::ContactSensor>(createSensor);
  }
  else
  {
    _ecm.EachNew<components::ContactSensor>(createSensor);
  }
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

        auto topicId = this->resolvedTopicNames.find(_entity);
        if (topicId == this->resolvedTopicNames.end())
        {
          gzerr << "Internal error, missing resolved contact sensor topic"
                << " name for entity [" << _entity << "]" << std::endl;
          return true;
        }

        this->resolvedTopicNames.erase(topicId);

        return true;
      });
}
//////////////////////////////////////////////////
Contact::Contact() : System(), dataPtr(std::make_unique<ContactPrivate>())
{
}

//////////////////////////////////////////////////
Contact::~Contact() = default;

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
      it.second->Publish(_info.simTime);
    }
  }

  this->dataPtr->RemoveSensors(_ecm);
}

//////////////////////////////////////////////////
void Contact::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  this->dataPtr->entitySensorMap.clear();
  this->dataPtr->resolvedTopicNames.clear();
  // After reset, existing sensor entities are not reported as new entities.
  // Re-scan them so contact sensors keep publishing in the next episode.
  this->dataPtr->CreateSensors(_ecm, true);
}

//////////////////////////////////////////////////
std::unordered_map<Entity, std::string> Contact::ResolvedTopicNames() const
{
  return this->dataPtr->resolvedTopicNames;
}

GZ_ADD_PLUGIN(Contact, System,
  Contact::ISystemPreUpdate,
  Contact::ISystemPostUpdate,
  Contact::ISystemReset
)

GZ_ADD_PLUGIN_ALIAS(Contact, "gz::sim::systems::Contact")

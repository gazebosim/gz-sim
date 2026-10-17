/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "ContactWriter.hh"

#include <gz/msgs/contact.pb.h>
#include <gz/msgs/entity.pb.h>

#include <gz/math/Helpers.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensorData.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::physics_system
{
namespace
{
/// \brief Check whether a message vector matches a vector to within 1e-6.
/// \tparam MsgVectorT Type of the message vector.
/// \param[in] _msg Message vector.
/// \param[in] _vec Vector.
/// \return True if the components are equal.
template<typename MsgVectorT>
bool VectorsEqual(const MsgVectorT &_msg, const math::Vector3d &_vec)
{
  return math::equal(_msg.x(), _vec.X(), 1e-6) &&
         math::equal(_msg.y(), _vec.Y(), 1e-6) &&
         math::equal(_msg.z(), _vec.Z(), 1e-6);
}
}

//////////////////////////////////////////////////
bool ContactsEqual(const msgs::Contacts &_msg, const EntityContactMap &_map)
{
  if (_msg.contact_size() != static_cast<int>(_map.size()))
    return false;

  auto mapIt = _map.begin();
  for (int i = 0; i < _msg.contact_size(); ++i, ++mapIt)
  {
    const auto &contactData = mapIt->second;
    if (_msg.contact(i).position_size() !=
        static_cast<int>(contactData.size()))
    {
      return false;
    }

    for (int j = 0; j < _msg.contact(i).position_size(); ++j)
    {
      const auto &contact = contactData[j];
      if (!VectorsEqual(_msg.contact(i).position(j), contact.point))
        return false;

      if (contact.extra)
      {
        if (!VectorsEqual(_msg.contact(i).normal(j), contact.extra->normal))
          return false;

        // Compare body1 and body2 forces
        if (!VectorsEqual(_msg.contact(i).wrench(j).body_1_wrench().force(),
                          contact.extra->force))
        {
          return false;
        }

        if (!VectorsEqual(_msg.contact(i).wrench(j).body_2_wrench().force(),
                          -contact.extra->force))
        {
          return false;
        }
      }
    }
  }
  return true;
}

//////////////////////////////////////////////////
void WriteContacts(EntityComponentManager &_ecm, const OwnerFilter &_owner,
    const ContactMap &_contacts, bool _entityNames)
{
  // Go through each collision entity that has a ContactData component and
  // set the component value to the list of contacts that correspond to
  // the collision entity
  _ecm.Each<components::Collision, components::ContactSensorData>(
      [&](const Entity &_collEntity1, components::Collision *,
          components::ContactSensorData *_contactData) -> bool
      {
        if (!_owner.Owns(_collEntity1))
          return true;

        const auto contactMapIt = _contacts.find(_collEntity1);
        if (contactMapIt == _contacts.end())
        {
          // Clear the last contact data
          bool changed = _contactData->Data().contact_size() > 0;
          if (changed)
          {
            _contactData->Data().Clear();
          }

          auto state = changed ?
            ComponentState::PeriodicChange :
            ComponentState::NoChange;
          _ecm.SetChanged(
            _collEntity1, components::ContactSensorData::typeId, state);
          return true;
        }

        const auto &contactMap = contactMapIt->second;

        bool changed = !ContactsEqual(_contactData->Data(), contactMap);
        auto state = changed ?
          ComponentState::PeriodicChange :
          ComponentState::NoChange;
        _ecm.SetChanged(
          _collEntity1, components::ContactSensorData::typeId, state);

        // If contacts are unchanged, no need to update them again
        if (!changed)
        {
          return true;
        }

        // If contacts have changed, first clear data then add contacts
        _contactData->Data().Clear();
        for (const auto &[collEntity2, contactData] : contactMap)
        {
          msgs::Contact *contactMsg = _contactData->Data().add_contact();
          contactMsg->mutable_collision1()->set_id(_collEntity1);
          contactMsg->mutable_collision2()->set_id(collEntity2);
          if (_entityNames)
          {
            contactMsg->mutable_collision1()->set_name(
              removeParentScope(scopedName(_collEntity1, _ecm, "::", 0), "::"));
            contactMsg->mutable_collision2()->set_name(
              removeParentScope(scopedName(collEntity2, _ecm, "::", 0), "::"));
          }
          for (const auto &contact : contactData)
          {
            auto *position = contactMsg->add_position();
            position->set_x(contact.point.X());
            position->set_y(contact.point.Y());
            position->set_z(contact.point.Z());

            // Not all physics engines report the extra contact data.
            if (contact.extra)
            {
              auto *normal = contactMsg->add_normal();
              normal->set_x(contact.extra->normal.X());
              normal->set_y(contact.extra->normal.Y());
              normal->set_z(contact.extra->normal.Z());

              auto *wrench = contactMsg->add_wrench();
              auto *body1Wrench = wrench->mutable_body_1_wrench();
              auto *body1Force = body1Wrench->mutable_force();
              body1Force->set_x(contact.extra->force.X());
              body1Force->set_y(contact.extra->force.Y());
              body1Force->set_z(contact.extra->force.Z());

              // The force on the second body is equal and opposite
              auto *body2Wrench = wrench->mutable_body_2_wrench();
              auto *body2Force = body2Wrench->mutable_force();
              body2Force->set_x(-contact.extra->force.X());
              body2Force->set_y(-contact.extra->force.Y());
              body2Force->set_z(-contact.extra->force.Z());

              contactMsg->add_depth(contact.extra->depth);
            }
          }
        }
        return true;
      });
}
}
}
}

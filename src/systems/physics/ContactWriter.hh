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
#ifndef GZ_SIM_SYSTEMS_PHYSICS_CONTACT_WRITER_HH_
#define GZ_SIM_SYSTEMS_PHYSICS_CONTACT_WRITER_HH_

#include <deque>
#include <optional>
#include <unordered_map>

#include <gz/math/Vector3.hh>
#include <gz/msgs/contacts.pb.h>

#include "OwnerFilter.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::physics_system
{
  /// \brief One contact point between two collisions.
  struct ContactPointData
  {
    /// \brief Normal, force and depth of a contact. Not every physics engine
    /// reports them.
    struct Extra
    {
      /// \brief Contact normal.
      math::Vector3d normal;

      /// \brief Force on the first collision. The force on the second one is
      /// equal and opposite.
      math::Vector3d force;

      /// \brief Penetration depth.
      // cppcheck-suppress unusedStructMember
      double depth{0.0};
    };

    /// \brief Contact position in the world frame.
    math::Vector3d point;

    /// \brief Normal, force and depth, if the engine reports them.
    std::optional<Extra> extra;
  };

  /// \brief All contact points of one collision, grouped by the other
  /// collision that is involved.
  using EntityContactMap =
      std::unordered_map<Entity, std::deque<ContactPointData>>;

  /// \brief Contact points of every collision that touched another one, by
  /// collision. A contact between collisions A and B appears under both A and
  /// B.
  using ContactMap = std::unordered_map<Entity, EntityContactMap>;

  /// \brief Check whether contact messages hold the same data as a map of
  /// contact points, to within a small tolerance.
  /// \param[in] _msg Contacts of one collision, in the order of the map.
  /// \param[in] _map Contact points of the same collision.
  /// \return True if the message already holds the contents of the map.
  bool ContactsEqual(const msgs::Contacts &_msg, const EntityContactMap &_map);

  /// \brief Write the ContactSensorData component of every collision that has
  /// one and that the filter accepts.
  ///
  /// A collision that is not in `_contacts` gets an empty component, so a
  /// provider must pass in the contacts of every collision it simulates. The
  /// filter keeps a provider from clearing the contacts that another
  /// provider wrote. The component is marked as changed only if its contents
  /// changed.
  ///
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in] _owner Selects the collisions to write.
  /// \param[in] _contacts Contacts found in the last step.
  /// \param[in] _entityNames True to fill the names of the collisions in
  /// the messages, in addition to their ids.
  void WriteContacts(EntityComponentManager &_ecm, const OwnerFilter &_owner,
      const ContactMap &_contacts, bool _entityNames);
}
}
}

#endif

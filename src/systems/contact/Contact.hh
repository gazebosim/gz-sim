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
#ifndef GZ_SIM_SYSTEMS_CONTACT_HH_
#define GZ_SIM_SYSTEMS_CONTACT_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class ContactPrivate;

  /** \class Contact Contact.hh \
   * gz/sim/systems/Contact/Contact.hh
  **/
  /// \brief Contact sensor system which manages all contact sensors in
  /// simulation
  ///
  /// A sensor with a positive `<update_rate>` publishes at most once per
  /// period of simulation time, with all the contacts gathered since its
  /// last publication. Publications follow a fixed schedule, so the average
  /// rate matches `<update_rate>` even when its period is not a multiple of
  /// the physics step, and a contact that starts between two publications
  /// is published with the next one. Without it (or with 0), a sensor
  /// publishes at every iteration that has contacts. A message is stamped
  /// with its publication time; each contact keeps the time of its own
  /// iteration.
  class Contact final:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate,
    public ISystemReset
  {
    /// \brief Constructor
    public: Contact();

    /// \brief Destructor
    public: ~Contact() final = default;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void Reset(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) final;

    /// \brief Private data pointer
    private: std::unique_ptr<ContactPrivate> dataPtr;
  };
}
}
}  // namespace sim
}  // namespace gz
#endif

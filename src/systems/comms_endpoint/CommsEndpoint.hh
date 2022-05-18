/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_GAZEBO_SYSTEMS_COMMSENDPOINT_HH_
#define GZ_GAZEBO_SYSTEMS_COMMSENDPOINT_HH_

#include <memory>

#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>
#include "gz/sim/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A system that registers in the comms broker an endpoint.
  /// You're creating an address attached to the model where the plugin is
  /// running. The system will bind this address in the broker automatically
  /// for you and unbind it when the model is destroyed.
  ///
  /// The endpoint can be configured with the following SDF parameters:
  ///
  /// * Required parameters:
  /// <address> An identifier used to receive messages (string).
  /// <topic> The topic name where you want to receive the messages targeted to
  /// this address.
  ///
  /// * Optional parameters:
  /// <broker> Element used to capture where are the broker services.
  ///          This block can contain any of the next optional parameters:
  ///    <bind_service>: Service name used to bind an address.
  ///                    The default value is "/broker/bind"
  ///    <unbind_service>: Service name used to unbind from an address.
  ///                      The default value is "/broker/unbind"
  ///
  /// Here's an example:
  /// <plugin
  ///   filename="ignition-gazebo-comms-endpoint-system"
  ///   name="ignition::gazebo::systems::CommsEndpoint">
  ///   <address>addr1</address>
  ///   <topic>addr1/rx</topic>
  ///   <broker>
  ///     <bind_service>/broker/bind</bind_service>
  ///     <unbind_service>/broker/unbind</unbind_service>
  ///   </broker>
  /// </plugin>
  class CommsEndpoint
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: CommsEndpoint();

    /// \brief Destructor
    public: ~CommsEndpoint();

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
  }
}
}
}

#endif

/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_PHYSICS_EVENTS_HH_
#define IGNITION_GAZEBO_PHYSICS_EVENTS_HH_

#include <ignition/common/Event.hh>

#include <ignition/physics/ContactJointProperties.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"

#include <Eigen/Geometry>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

    namespace events
    {
      using Policy = physics::FeaturePolicy3d;

      /// \brief This event is called when the physics engine needs to collect
      /// what customizations it should do to the surface of a contact point. It
      /// is called during the Update phase after collision checking has been
      /// finished and before the physics update has happened. The event
      /// subscribers are expected to change the `params` argument.
      using CollectContactSurfaceProperties = ignition::common::EventT<
        void(
          const Entity& /* collision1 */,
          const Entity& /* collision2 */,
          const Eigen::Vector3d &  /* point */,
          const Eigen::Vector3d * const /* force */,
          const Eigen::Vector3d * const /* normal */,
          const double * const /* depth */,
          const size_t /* numContactsOnCollision */,
          physics::SetContactJointPropertiesCallbackFeature::
            ContactSurfaceParams<Policy>& /* params */
        ),
        struct CollectContactSurfacePropertiesTag>;
      }
    }  // namespace events
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_PHYSICS_EVENTS_HH_

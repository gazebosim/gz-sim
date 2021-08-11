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
#ifndef IGNITION_GAZEBO_SYSTEMS_PHYSICS_HH_
#define IGNITION_GAZEBO_SYSTEMS_PHYSICS_HH_

#include <memory>
#include <unordered_map>
#include <utility>
#include <ignition/physics/FindFeatures.hh>
#include <ignition/physics/RequestFeatures.hh>

// Features need to be defined ahead of entityCast
#include <ignition/physics/BoxShape.hh>
#include <ignition/physics/CylinderShape.hh>
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/FreeGroup.hh>
#include <ignition/physics/FixedJoint.hh>
#include <ignition/physics/GetContacts.hh>
#include <ignition/physics/GetBoundingBox.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/Link.hh>
#include <ignition/physics/RemoveEntities.hh>
#include <ignition/physics/Shape.hh>
#include <ignition/physics/SphereShape.hh>
#include <ignition/physics/mesh/MeshShape.hh>
#include <ignition/physics/sdf/ConstructCollision.hh>
#include <ignition/physics/sdf/ConstructJoint.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructNestedModel.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class PhysicsPrivate;

  /// \class Physics Physics.hh ignition/gazebo/systems/Physics.hh
  /// \brief Base class for a System.
  class Physics:
    public System,
    public ISystemConfigure,
    public ISystemUpdate
  {
    /// \brief Constructor
    public: explicit Physics();

    /// \brief Destructor
    public: ~Physics() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void Update(const UpdateInfo &_info,
                EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<PhysicsPrivate> dataPtr;
  };
}
}
}
}
#endif

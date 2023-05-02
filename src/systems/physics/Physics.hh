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
#ifndef GZ_GAZEBO_SYSTEMS_PHYSICS_HH_
#define GZ_GAZEBO_SYSTEMS_PHYSICS_HH_

#include <memory>
#include <unordered_map>
#include <utility>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/RequestFeatures.hh>

// Features need to be defined ahead of entityCast
#include <gz/physics/BoxShape.hh>
#include <gz/physics/CylinderShape.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/SphereShape.hh>
#include <gz/physics/mesh/MeshShape.hh>
#include <gz/physics/sdf/ConstructCollision.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructNestedModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

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

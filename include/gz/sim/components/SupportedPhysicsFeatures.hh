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
#ifndef GZ_SIM_COMPONENTS_SUPPORTEDPHYSICSFEATURES_HH_
#define GZ_SIM_COMPONENTS_SUPPORTEDPHYSICSFEATURES_HH_

#include <string>
#include <vector>

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief The physics features that the physics provider of a model, or of
  /// every model in a world, supports.
  ///
  /// The data is a list of feature names. The order does not matter to
  /// `gz::sim::physicsSupports`, but two components compare equal only if
  /// their lists match in order. A name must not contain whitespace, because
  /// the serialized form is split on it. A feature name is a string whose
  /// meaning is agreed between the provider that lists it and the systems that
  /// require it, see ISystemPhysicsRequirements.
  ///
  /// The component can sit on a top-level model or on the world:
  ///
  /// * On a top-level model, it applies to that model and all of its
  ///   descendants.
  /// * On the world, it is the default for every model that does not carry its
  ///   own component. A model's component, even an empty one, takes
  ///   precedence over the world's.
  ///
  /// A provider must create the component no later than the Configure call of
  /// the model's plugins. The system manager reads it when it activates those
  /// plugins, which is before the next PreUpdate.
  ///
  /// If the component is absent everywhere, the provider is assumed to support
  /// every feature. Use `gz::sim::physicsSupports` from `gz/sim/Util.hh` to
  /// query it.
  using SupportedPhysicsFeatures = Component<std::vector<std::string>,
      class SupportedPhysicsFeaturesTag,
      serializers::VectorSerializer<std::string>>;
  GZ_SIM_DECLARE_COMPONENT("gz_sim_components.SupportedPhysicsFeatures",
      SupportedPhysicsFeatures)
}
}
}
}

#endif

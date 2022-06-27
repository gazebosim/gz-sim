/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SDFGENERATOR_HH_
#define GZ_SIM_SDFGENERATOR_HH_

#include <gz/msgs/sdf_generator_config.pb.h>

#include <sdf/Element.hh>
#include <optional>
#include <string>
#include <unordered_map>

#include "gz/sim/EntityComponentManager.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace sdf_generator
{
  using IncludeUriMap = std::unordered_map<std::string, std::string>;

  /// \brief Generate the SDFormat representation of a world
  /// \input[in] _ecm Immutable reference to the Entity Component Manager
  /// \input[in] _entity World entity
  /// \input[in] _includeUriMap Map from file paths to URIs used to preserve
  /// included Fuel models
  /// \input[in] _config Configuration for the world generator
  /// \returns Generated world string if generation succeeded.
  /// Otherwise, nullopt
  GZ_SIM_VISIBLE
  std::optional<std::string> generateWorld(
      const EntityComponentManager &_ecm, const Entity &_entity,
      const IncludeUriMap &_includeUriMap = IncludeUriMap(),
      const msgs::SdfGeneratorConfig &_config = msgs::SdfGeneratorConfig());

  /// \brief Update a sdf::Element of a world. Intended for internal use.
  /// \input[in, out] _elem sdf::Element to update
  /// \input[in] _ecm Immutable reference to the Entity Component Manager
  /// \input[in] _entity World entity
  /// \input[in] _includeUriMap Map from file paths to URIs used to preserve
  /// included Fuel models
  /// \input[in] _config Configuration for the world generator
  GZ_SIM_VISIBLE
  bool updateWorldElement(
      sdf::ElementPtr _elem,
      const EntityComponentManager &_ecm, const Entity &_entity,
      const IncludeUriMap &_includeUriMap = IncludeUriMap(),
      const msgs::SdfGeneratorConfig &_config = msgs::SdfGeneratorConfig());

  /// \brief Update a sdf::Element of an inlined model.
  /// Intended for internal use.
  /// \input[in, out] _elem sdf::Element to update
  /// \input[in] _ecm Immutable reference to the Entity Component Manager
  /// \input[in] _entity Model entity
  /// \returns true if update succeeded.
  GZ_SIM_VISIBLE
  bool updateModelElement(const sdf::ElementPtr &_elem,
                          const EntityComponentManager &_ecm,
                          const Entity &_entity);

  /// \brief Update a sdf::Element model to use //include instead of expanded
  /// model (to be used when expand_include_tags is disabled)
  /// \input[in, out] _elem sdf::Element to update
  /// \input[in] _saveFuelVersion True if "Save Fuel model versions" is enabled
  /// \input[in] _includeUriMap Map from file paths to URIs used to preserve
  /// included Fuel models
  GZ_SIM_VISIBLE
  void updateModelElementWithNestedInclude(sdf::ElementPtr &_elem,
                                           const bool _saveFuelVersion,
                                           const IncludeUriMap &_includeUriMap);

  /// \brief Update a sdf::Element of an included resource.
  /// Intended for internal use.
  /// \input[in, out] _elem sdf::Element to update
  /// \input[in] _ecm Immutable reference to the Entity Component Manager
  /// \input[in] _entity Entity of included resource
  /// \input[in] _uri Uri of the resource
  /// \returns true if update succeeded.
  GZ_SIM_VISIBLE
  bool updateIncludeElement(const sdf::ElementPtr &_elem,
                            const EntityComponentManager &_ecm,
                            const Entity &_entity, const std::string &_uri);

  /// \brief Update an sdf::Element of a link.
  /// Intended for internal use.
  /// \input[in, out] _elem sdf::Element to update
  /// \input[in] _ecm Immutable reference to the Entity Component Manager
  /// \input[in] _entity Link entity
  /// \returns true if update succeeded.
  GZ_SIM_VISIBLE
  bool updateLinkElement(const sdf::ElementPtr &_elem,
                         const EntityComponentManager &_ecm,
                         const Entity &_entity);

  /// \brief Update an sdf::Element of a sensor.
  /// Intended for internal use.
  /// \input[in, out] _elem sdf::Element to update
  /// \input[in] _ecm Immutable reference to the Entity Component Manager
  /// \input[in] _entity Sensor entity
  /// \returns true if update succeeded.
  GZ_SIM_VISIBLE
  bool updateSensorElement(sdf::ElementPtr _elem,
                           const EntityComponentManager &_ecm,
                           const Entity &_entity);

  /// \brief Update an sdf::Element of a light.
  /// Intended for internal use.
  /// \input[in, out] _elem sdf::Element to update
  /// \input[in] _ecm Immutable reference to the Entity Component Manager
  /// \input[in] _entity Light entity
  /// \returns true if update succeeded.
  GZ_SIM_VISIBLE
  bool updateLightElement(sdf::ElementPtr _elem,
                           const EntityComponentManager &_ecm,
                           const Entity &_entity);

  /// \brief Update an sdf::Element of a joint.
  /// Intended for internal use.
  /// \input[in, out] _elem sdf::Element to update
  /// \input[in] _ecm Immutable reference to the Entity Component Manager
  /// \input[in] _entity joint entity
  /// \returns true if update succeeded.
  GZ_SIM_VISIBLE
  bool updateJointElement(sdf::ElementPtr _elem,
                           const EntityComponentManager &_ecm,
                           const Entity &_entity);
}  // namespace sdf_generator
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif /* end of include guard: GZ_SIM_SDFGENERATOR_HH_ */

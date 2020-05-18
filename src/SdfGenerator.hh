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
#ifndef IGNITION_GAZEBO_SDFGENERATOR_HH_
#define IGNITION_GAZEBO_SDFGENERATOR_HH_

#include <ignition/msgs/sdf_generator_config.pb.h>

#include <sdf/Element.hh>
#include <string>
#include <unordered_map>

#include "ignition/gazebo/EntityComponentManager.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
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
  bool updateModelElement(const sdf::ElementPtr &_elem,
                          const EntityComponentManager &_ecm,
                          const Entity &_entity);

  /// \brief Update a sdf::Element of an included resource.
  /// Intended for internal use.
  /// \input[in, out] _elem sdf::Element to update
  /// \input[in] _ecm Immutable reference to the Entity Component Manager
  /// \input[in] _entity Entity of included resource
  /// \input[in] _uri Uri of the resource
  /// \returns true if update succeeded.
  bool updateIncludeElement(const sdf::ElementPtr &_elem,
                            const EntityComponentManager &_ecm,
                            const Entity &_entity, const std::string &_uri);

}  // namespace sdf_generator
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif /* end of include guard: IGNITION_GAZEBO_SDFGENERATOR_HH_ */

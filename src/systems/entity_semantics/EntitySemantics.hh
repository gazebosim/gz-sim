/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_ENTITYSEMANTICS_HH_
#define GZ_SIM_SYSTEMS_ENTITYSEMANTICS_HH_

#include <gz/sim/System.hh>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
// Forward declaration
class EntitySemanticsPrivate;

/// \brief System for assigning semantics to entities, such as models,
/// by classifying them into categories and assigning them tags.
/// Reference:
/// https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityInfo.msg,
/// https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityCategory.msg,
/// https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/TagsFilter.msg
///
/// ## System Parameters
///
/// This system is meant to be attached to a `<world>`, but it parses parameters
/// found in entities such as models.
///
/// ## Entity Parameters
///
/// - `<gz:semantics>`: Root element to be specified inside entities, e.g.
/// <model>. It has the following child parameters:
///
///   - `<category>`: string representation of the value as defined in
///   https://github.com/ros-simulation/simulation_interfaces/blob/1.0.0/msg/EntityCategory.msg.
///   The string is formed by removing the prefix `CATEGORY_` from the
///   categories listed in the EntityCategory message. Example: for
///   `CATEGORY_ROBOT`, it would be `<category>ROBOT</category>`. Lowercase
///   values will also be accepted. There can only be one instance of the
///   `<category>` element for an entity.
///
///   - `<tag>`: string labels that can be assigned according to the user's
///   classification scheme. They serve as a secondary layer of classification
///   in on top of categories. Multiple instances of the `<tag>` elements are
///   allowed for an entity.
///
///   - `<description>`: freeform string that describes the entity. There can
///   only be one instance of the `<description>` element for an entity.
///
/// Example:
///
/// ```xml
/// <model name="robot1">
///   <gz:semantics>
///     <category>ROBOT</category>
///     <tag>mobile</tag>
///     <tag>diff_drive</tag>
///     <description>Food delivery mobile robot</description>
///   </gz:semantics>
/// </model>
///
///
/// ```
///

class EntitySemantics : public System,
                        public ISystemPreUpdate
{
  // Documentation inherited
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override;
};
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif

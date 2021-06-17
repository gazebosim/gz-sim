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
#ifndef IGNITION_GAZEBO_SYSTEMS_LABEL_HH_
#define IGNITION_GAZEBO_SYSTEMS_LABEL_HH_

#include <memory>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A label plugin that annotates models by setting the label
  /// for the parent entity's visuals
  ///
  /// Must contain exactly one <label> tag with a value of the label to
  /// set it to the visuals's user data
  /// Ex: "<label>1</label>" with a label = 1
  /// Label value must be in [0-255] range
  class Label:
    public System,
    public ISystemConfigure
  {
    /// \brief Constructor
    public: explicit Label();

    /// \brief Destructor
    public: ~Label() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           gazebo::EventManager &_eventMgr) final;
  };
  }
}
}
}
#endif

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

#ifndef MARITIME_WAVEVISUAL_HH_
#define MARITIME_WAVEVISUAL_HH_

#include <memory>
#include <gz/utils/ImplPtr.hh>

#include "gz/sim/System.hh"

namespace maritime
{
  /// \brief A plugin for setting shaders to a visual and its params
  ///
  /// Plugin parameters:
  ///
  /// <shader>
  ///   <vertex>   Path to wave vertex program
  ///   <fragment> Path to wave fragment program
  /// <wavefield>  Wavefield parameters - see Wavefield.hh
  ///
  class WaveVisual
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: WaveVisual();

    /// \brief Destructor
    public: ~WaveVisual() override;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}

#endif

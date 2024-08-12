/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#ifndef GZ_SIM_LENSFLARE_SYSTEM_HH_
#define GZ_SIM_LENSFLARE_SYSTEM_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief Private data class for LensFlare Plugin
  class LensFlarePrivate;

  /** \class LensFlare LensFlare.hh \
   * gz/sim/systems/LensFlare.hh
  **/
  /// \brief Add lens flare effects to the camera output as a render pass
  ///
  /// ## System Parameters
  ///
  /// - `<scale>`: Sets the scale of the lens flare. If this is
  ///   not specified, the value defaults to 1.0
  ///
  /// - `<color>`: Sets the color of the lens flare. The default
  ///   is {1.4, 1.2, 1.0}
  ///
  /// - `<occlusion_steps>`: Sets the number of steps to take in
  ///   each direction to check for occlusions.
  ///   The default value is set to 10. Use 0 to disable
  ///
  /// - `<light_name>`: Sets the light associated with the lens flares.
  ///   If not specified. The first light in the scene will
  ///   be used.

  class LensFlare:
    public System,
    public ISystemConfigure
  {
    /// \brief Constructor
    public: LensFlare();

    /// \brief Destructor
    public: ~LensFlare() override = default;

    /// Document Inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// \brief Private data pointer
    private: std::unique_ptr<LensFlarePrivate> dataPtr;
  };
}
}
}
}
#endif

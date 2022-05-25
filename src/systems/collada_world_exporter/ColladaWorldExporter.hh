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

#ifndef GZ_SIM_SYSTEMS_COLLADAWORLDEXPORTER_HH_
#define GZ_SIM_SYSTEMS_COLLADAWORLDEXPORTER_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class ColladaWorldExporterPrivate;

  /// \brief A plugin that exports a world to a mesh.
  /// When loaded the plugin will dump a mesh containing all the models in
  /// the world to the current directory.
  class ColladaWorldExporter final:
    public System,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: ColladaWorldExporter();

    /// \brief Destructor
    public: ~ColladaWorldExporter() final;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm);

    /// \brief Private data pointer.
    private: std::unique_ptr<ColladaWorldExporterPrivate> dataPtr;
  };
}
}
}
}
#endif

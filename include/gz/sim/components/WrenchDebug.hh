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
#ifndef GZ_SIM_COMPONENTS_WRENCHDEBUG_HH_
#define GZ_SIM_COMPONENTS_WRENCHDEBUG_HH_

#include <memory>
#include <string>
#include <vector>
#include <gz/math/Vector3.hh>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief
  struct GZ_SIM_VISIBLE WrenchDebugData
  {
    std::string label;
    math::Vector3d force;
    math::Vector3d torque;
    math::Vector3d position;
  };

  /// \brief
  struct GZ_SIM_VISIBLE WrenchDebugListData
  {
    std::vector<WrenchDebugData> moments;

    static std::shared_ptr<WrenchDebugListData> make_shared(
      const std::vector<WrenchDebugData>& moments);
  };

  using WrenchDebugList =
    Component<std::shared_ptr<WrenchDebugListData>,
      class WrenchDebugListDataTag>;

  using WrenchDebugEnable =
    Component<NoData, class WrenchEnableDataTag>;

  GZ_SIM_REGISTER_COMPONENT(
    "gz_sim_components.WrenchDebugList", WrenchDebugList)
  GZ_SIM_REGISTER_COMPONENT(
    "gz_sim_components.WrenchDebugEnable", WrenchDebugEnable)
}
}
}
}

#endif

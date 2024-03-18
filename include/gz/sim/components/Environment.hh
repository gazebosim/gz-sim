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

#ifndef GZ_SIM_ENVIRONMENT_HH_
#define GZ_SIM_ENVIRONMENT_HH_

#include <memory>
#include <string>
#include <utility>

#include <gz/common/DataFrame.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/TimeVaryingVolumetricGrid.hh>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief Environment data across time and space. This is useful to
  /// introduce physical quantities that may be of interest even if not
  /// modelled in simulation.
  struct GZ_SIM_VISIBLE EnvironmentalData
  {
    using T = math::InMemoryTimeVaryingVolumetricGrid<double>;
    using FrameT = common::DataFrame<std::string, T>;
    using ReferenceT = math::SphericalCoordinates::CoordinateType;

    /// \brief Reference units
    enum class ReferenceUnits {
      RADIANS = 0,
      DEGREES
    };

    /// \brief Instantiate environmental data.
    ///
    /// An std::make_shared equivalent that ensures
    /// dynamically loaded call sites use a template
    /// instantiation that is guaranteed to outlive
    /// them.
    static std::shared_ptr<EnvironmentalData>
    MakeShared(FrameT _frame, ReferenceT _reference,
      ReferenceUnits _units = ReferenceUnits::RADIANS,
      bool _ignoreTimeStep = false)
    {
      auto data = std::make_shared<EnvironmentalData>();
      data->frame = std::move(_frame);
      data->reference = _reference;
      data->units = _units;
      data->staticTime = _ignoreTimeStep;
      return data;
    }

    /// \brief Environmental data frame.
    FrameT frame;

    /// \brief Spatial reference for data coordinates.
    ReferenceT reference;

    /// \brief The units to be used (only for spherical coordinates)
    ReferenceUnits units;

    /// \brief Use time axis or not.
    bool staticTime;
  };

  /// \brief A component type that contains a environment data.
  /// Ownership is shared to avoid data copies unless necessary.
  using Environment =
      Component<std::shared_ptr<EnvironmentalData>, class EnvironmentalDataTag>;

  GZ_SIM_REGISTER_COMPONENT(
      "gz_sim_components.Environment", Environment)
}
}
}
}

#endif

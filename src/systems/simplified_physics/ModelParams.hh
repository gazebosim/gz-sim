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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_MODEL_PARAMS_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_MODEL_PARAMS_HH_

#include <optional>
#include <string>
#include <vector>

#include <sdf/Element.hh>

#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
  /// \brief A `<wheel>` element: a wheel joint that the model names.
  struct WheelOverride
  {
    /// \brief Name of the joint, relative to the model.
    std::string joint;

    /// \brief Radius from the `radius` attribute, if the element has one.
    std::optional<double> radius;
  };

  /// \brief The parameters of the SimplifiedPhysicsModel plugin that the
  /// SimplifiedPhysics system reads, besides the mode.
  ///
  /// The system reads them from the model's SDF, where the plugin and its
  /// parameters are, so that a model needs no extra component.
  struct ModelParams
  {
    /// \brief Wheel joints named by `<wheel>` elements. When the list is not
    /// empty it replaces wheel inference.
    std::vector<WheelOverride> wheels;

    /// \brief Drift fraction of `<odometry_noise>`. Zero is off.
    double odometryNoise{0.0};

    /// \brief Value of `<animate_wheels>`. Not implemented yet.
    bool animateWheels{false};
  };

  /// \brief Name of the SimplifiedPhysicsModel plugin as the `name` attribute
  /// of its `<plugin>` element.
  constexpr const char *kModelPluginName{
      "gz::sim::systems::SimplifiedPhysicsModel"};

  /// \brief The child elements of a plugin element, which are the
  /// parameters.
  /// \param[in] _plugin The `<plugin>` element.
  /// \return Its children.
  std::vector<sdf::ElementPtr> PluginContents(const sdf::Element &_plugin);

  /// \brief Read the parameters from the contents of the plugin element.
  ///
  /// A value that cannot be used (a negative or non-numeric noise, a
  /// `<wheel>` without a name) is skipped and reported with a warning if
  /// `_warn` is set.
  ///
  /// \param[in] _contents The children of the `<plugin>` element, as
  /// sdf::Plugin::Contents() or PluginContents() give them.
  /// \param[in] _model Name of the model, for the messages.
  /// \param[in] _warn True to print warnings. The plugin that configures the
  /// model prints them. The system reads the same element again and stays
  /// quiet.
  /// \return The parameters.
  ModelParams ParseModelParams(const std::vector<sdf::ElementPtr> &_contents,
      const std::string &_model, bool _warn);
}
}
}

#endif

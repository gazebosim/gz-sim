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
#ifndef IGNITION_GAZEBO_SYSTEMS_SURFACE_HH_
#define IGNITION_GAZEBO_SYSTEMS_SURFACE_HH_

#include <memory>
#include <ignition/gazebo/System.hh>
#include <sdf/sdf.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class SurfacePrivate;

  /// \brief A system that simulates the buoyancy of an object at the surface of
  /// a fluid. This system must be attached to a model and the system will apply
  /// buoyancy to a few sample points around a given link.
  ///
  /// ## Required system parameters
  ///
  /// * `<link_name>` is the name of the link used to apply forces.
  /// * `<vehicle_length>` is the length of the vessel [m].
  /// * `<vehicle_width>` is the width of the vessel [m].
  /// * `<hull_radius>` is the radius of the vessel's hull [m].
  ///
  /// ## Optional system parameters
  ///
  /// * `<num_samples>` is the number of samples where forces will be applied.
  /// * `<fluid_level>` is the depth at which the fluid should be in the vehicle
  /// * `<fluid_density>` is the density of the fluid.
  ///
  /// ## Example
  /// <plugin
  ///   filename="ignition-gazebo-surface-system"
  ///   name="ignition::gazebo::systems::Surface">
  ///   <link_name>base_link</link_name>
  ///   <vehicle_length>4.9</vehicle_length>
  ///   <vehicle_width>2.4</vehicle_width>
  ///   <hull_radius>0.213</hull_radius>
  /// </plugin>
  class Surface
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor.
    public: Surface();

    /// \brief Destructor.
    public: ~Surface() override = default;

    // Documentation inherited.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Convenience function for calculating the area of circle segment.
    /// \param[in] _r Radius of circle.
    /// \param[in] _h Height of the chord line.
    /// \return The area.
    /// \ref https://www.mathopenref.com/segmentareaht.html
    private: double CircleSegment(double _r, double _h) const;

    /// \brief Private data pointer.
    private: std::unique_ptr<SurfacePrivate> dataPtr;
  };
  }
}
}
}

#endif

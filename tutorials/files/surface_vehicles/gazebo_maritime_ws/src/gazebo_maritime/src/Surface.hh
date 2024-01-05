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

#ifndef MARITIME_SURFACE_HH_
#define MARITIME_SURFACE_HH_

#include <memory>
#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace maritime
{
  /// \brief A system that simulates the buoyancy of an object at the surface of
  /// a fluid. This system must be attached to a model and the system will apply
  /// buoyancy to a collection of points around a given link.
  ///
  /// This system models the vehicle's buoyancy assuming a single hull with a
  /// cylindrical shape. It's possible to derive from this plugin and provide
  /// your own buoyancy function at each point. For this purpose you
  /// should override `BuoyancyAtPoint()` in the derived plugin.
  ///
  /// This plugin also supports waves. If you provide a wavefield via SDF, the
  /// plugin will account for the delta Z that the waves generate at each point.
  ///
  /// ## Required system parameters
  ///
  /// * `<link_name>` is the name of the link used to apply forces.
  ///
  /// ## Optional system parameters
  ///
  /// * `<vehicle_length>` is the length of the vessel [m].
  /// * `<hull_radius>` is the radius of the vessel's hull [m].
  /// * `<fluid_level>` is the depth at which the fluid should be in the vehicle
  /// * `<fluid_density>` is the density of the fluid.
  /// * `<points>` contains a collection of points where the forces generated
  ///              by this plugin will be applied. See the format of each point
  ///              next:
  /// *   `<point><position>` Relative position of the point relative to
  ///                         `link_name`.
  /// * <wavefield>: The wavefield parameters. See `Wavefield.hh`.
  ///
  /// ## Example
  /// <plugin
  ///   filename="gz-sim-surface-system"
  ///   name="gz::sim::systems::Surface">
  ///   <link_name>base_link</link_name>
  ///   <vehicle_length>4.9</vehicle_length>
  ///   <vehicle_width>2.4</vehicle_width>
  ///   <hull_radius>0.213</hull_radius>
  ///   <fluid_level>0</fluid_level>

  ///   <!-- Points -->
  ///   <points>
  ///     <point>
  ///       <position>1.225 1.2 0</position>
  ///     </point>
  ///     <point>
  ///       <position>1.225 -1.2 0</position>
  ///     </point>
  ///     <point>
  ///       <position>-1.225 1.2 0</position>
  ///     </point>
  ///     <point>
  ///       <position>-1.225 -1.2 0</position>
  ///     </point>
  ///   </points>

  ///   <!-- Waves -->
  ///   <wavefield>
  ///     <size><%= $wavefield_size%> <%= $wavefield_size%></size>
  ///     <cell_count><%= $wavefield_cell_count%> <%=$wavefield_cell_count%></cell_count>
  ///     <wave>
  ///       <model>PMS</model>
  ///       <period>5.0</period>
  ///       <number>3</number>
  ///       <scale>1.1</scale>
  ///       <gain>0.5</gain>
  ///       <direction>1 0</direction>
  ///       <angle>0.4</angle>
  ///       <tau>2.0</tau>
  ///       <amplitude>0.0</amplitude>
  ///       <steepness>0.0</steepness>
  ///     </wave>
  ///   </wavefield>
  /// </plugin>
  class Surface
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor.
    public: Surface();

    /// \brief Destructor.
    public: ~Surface() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Get the gravity component.
    /// \return Gravity vector.
    public: gz::math::Vector3d Gravity() const;

    /// \brief Get the vehicle length.
    /// \return Vechicle length in m.
    public: double HullLength() const;

    /// \brief Get the hull radius.
    /// \return The hull radius in m.
    public: double HullRadius() const;

    /// \brief Get the fluid density.
    /// \return The fluid density in kg/m^3.
    public: double FluidDensity() const;

    /// \brief Compute the buoyancy generated at a point in the vehicle.
    /// \param[in] _info Simulator information about the current timestep.
    /// \param[in] _point The point.
    /// \param[in] _deltaZ Total Z location of the point relative to fluid
    ///            surface.
    /// \param[in] _ecm Ignition's ECM.
    // public: virtual double BuoyancyAtPoint(
    //                             const gz::sim::UpdateInfo &_info,
    //                             const gz::math::Vector3d &_point,
    //                             const uint16_t _pointsPerHull,
    //                             double _deltaZ,
    //                             gz::sim::EntityComponentManager &_ecm);

    /// \brief Convenience function for calculating the area of circle segment.
    /// \param[in] _r Radius of circle.
    /// \param[in] _h Height of the chord line.
    /// \return The area.
    /// \ref https://www.mathopenref.com/segmentareaht.html
    public: double CircleSegment(double _r,
                                 double _h) const;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}

#endif
